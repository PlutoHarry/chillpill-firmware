/**
 * @file estimator.c
 * @brief State estimator for the ChillPill control FSM.
 *
 * The estimator consumes raw sensor telemetry and produces filtered
 * measurements that are easier for the finite state machine and PID
 * controller to consume.  It fuses temperatures, motor current and
 * auger activity into higher level signals such as slush texture,
 * condenser load, bowl volume and evaporator icing.  Most of the
 * heuristics are intentionally simple first-order filters so that the
 * behaviour is predictable and can be tuned via @ref g_control_config.
 */

#include "build_config.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "debug_log.h"

#include "stm32f1xx_hal.h"

#include "estimator.h"

#if ENABLE_CONTROL_FSM

#include "control_config.h"
#include "pid_controller.h"
#include "sensors.h"
#include "actuators.h"
#include "user_settings.h"

#define ESTIMATOR_DEBUG_PERIOD_MS 1000U

typedef struct
{
    bool      seeded;               /**< True once the filters have been seeded. */
    bool      volume_ready;         /**< Volume estimator has an initial temperature. */
    bool      torque_ready;         /**< Torque baseline has been captured. */
    bool      needs_deicing;        /**< Sticky icing flag exposed to the FSM. */
    uint32_t  last_update_ms;       /**< Timestamp of the previous update. */
    uint32_t  icing_accum_ms;       /**< Time above icing threshold. */
    uint32_t  clear_accum_ms;       /**< Time below icing clear threshold. */
    uint32_t  volume_elapsed_ms;    /**< Elapsed time since last volume refresh. */
    float     bowl_temp_filtered;   /**< Low-pass filtered bowl temperature (°C). */
    float     prev_bowl_temp;       /**< Previous filtered bowl temperature for slope. */
    float     evap_in_filtered;     /**< Filtered evaporator inlet temperature (°C). */
    float     evap_out_filtered;    /**< Filtered evaporator outlet temperature (°C). */
    float     evap_delta_filtered;  /**< Filtered evaporator delta (°C). */
    float     motor_current_filtered; /**< Filtered motor current draw (A). */
    float     texture_index;        /**< Estimated slush texture index [0,1]. */
    float     condenser_load;       /**< Normalised condenser load [0,1]. */
    float     ice_index;            /**< Icing index [0,1]. */
    float     volume_fraction;      /**< Estimated bowl volume [0,1]. */
    float     volume_last_temp;     /**< Bowl temperature when volume last sampled. */
    uint8_t   volume_level;         /**< Discrete volume level (0 low, 1 med, 2 high). */
    float     torque_baseline;      /**< Reference motor current for torque. */
} estimator_state_t;

static estimator_state_t s_state;

static control_mode_t estimator_mode_to_control_mode(pid_controller_mode_t mode)
{
    switch (mode) {
        case PID_CONTROLLER_MODE_COLD_DRINK:
            return CONTROL_MODE_COLD_DRINK;
        case PID_CONTROLLER_MODE_LIGHT_SLUSH:
            return CONTROL_MODE_LIGHT_SLUSH;
        case PID_CONTROLLER_MODE_MEDIUM_SLUSH:
            return CONTROL_MODE_MEDIUM_SLUSH;
        case PID_CONTROLLER_MODE_HEAVY_SLUSH:
            return CONTROL_MODE_HEAVY_SLUSH;
        default:
            break;
    }
    return CONTROL_MODE_LIGHT_SLUSH;
}

static const control_mode_profile_t *estimator_active_profile(void)
{
    pid_controller_mode_t mode = pid_controller_get_current_mode();
    return control_config_get_profile(estimator_mode_to_control_mode(mode));
}

/* ------------------------------------------------------------------------- */

static inline float clampf(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

static float lowpass(float previous, float sample, float tau_s, float dt_s)
{
    if (!isfinite(sample)) {
        return previous;
    }
    if (tau_s <= 0.0f) {
        return sample;
    }
    if (!isfinite(previous)) {
        previous = sample;
    }
    float alpha = dt_s / (tau_s + dt_s);
    alpha = clampf(alpha, 0.0f, 1.0f);
    return previous + alpha * (sample - previous);
}

static bool fetch_sensor(uint8_t (*getter)(float *), float *out)
{
    if (getter == NULL || out == NULL) {
        return false;
    }
    return getter(out) == 0U;
}

/* ------------------------------------------------------------------------- */

void estimator_init(void)
{
    (void)memset(&s_state, 0, sizeof(s_state));
    s_state.volume_fraction = 1.0f;
    s_state.volume_level    = 2U;
    s_state.torque_baseline = 1.0f;
    s_state.prev_bowl_temp  = 0.0f;
}

void estimator_update(void)
{
    uint32_t now_ms = HAL_GetTick();
    uint32_t elapsed_ms = (s_state.last_update_ms == 0U) ? 20U : (now_ms - s_state.last_update_ms);
    if (elapsed_ms > 1000U) {
        elapsed_ms = 1000U;
    }
    s_state.last_update_ms = now_ms;

    bool prev_deicing = s_state.needs_deicing;
#if !ENABLE_DEBUG_LOGGING
    UNUSED(prev_deicing);
#endif

    float dt_s = (float)elapsed_ms / 1000.0f;
    if (dt_s <= 0.0f) {
        dt_s = 0.02f;
    }

    float bowl_sample = 0.0f;
    float evap_in_sample = 0.0f;
    float evap_out_sample = 0.0f;
    float motor_current_sample = 0.0f;

    bool have_bowl       = fetch_sensor(get_bowl_temp, &bowl_sample);
    bool have_evap_in    = fetch_sensor(get_evap_in_temp, &evap_in_sample);
    bool have_evap_out   = fetch_sensor(get_evap_out_temp, &evap_out_sample);
    bool have_motor_curr = fetch_sensor(get_motor_current, &motor_current_sample);

    if (!s_state.seeded) {
        if (have_bowl) {
            s_state.bowl_temp_filtered = bowl_sample;
            s_state.prev_bowl_temp     = bowl_sample;
            s_state.volume_last_temp   = bowl_sample;
        }
        if (have_evap_in) {
            s_state.evap_in_filtered = evap_in_sample;
        }
        if (have_evap_out) {
            s_state.evap_out_filtered = evap_out_sample;
        }
        if (have_motor_curr) {
            s_state.motor_current_filtered = motor_current_sample;
            s_state.torque_baseline        = fmaxf(motor_current_sample, 0.2f);
            s_state.torque_ready           = true;
        }
        if (have_bowl && have_evap_in && have_evap_out && have_motor_curr) {
            s_state.seeded = true;
        }
    }

    if (have_bowl) {
        s_state.bowl_temp_filtered = lowpass(s_state.bowl_temp_filtered, bowl_sample, 5.0f, dt_s);
    }
    if (have_evap_in) {
        s_state.evap_in_filtered = lowpass(s_state.evap_in_filtered, evap_in_sample, 4.0f, dt_s);
    }
    if (have_evap_out) {
        s_state.evap_out_filtered = lowpass(s_state.evap_out_filtered, evap_out_sample, 4.0f, dt_s);
    }
    if (have_motor_curr) {
        s_state.motor_current_filtered = lowpass(s_state.motor_current_filtered, motor_current_sample, 3.0f, dt_s);
        if (!s_state.torque_ready) {
            s_state.torque_baseline = fmaxf(s_state.motor_current_filtered,
                                            g_control_config.estimator_torque_baseline_floor);
            s_state.torque_ready    = true;
        }
    }

    /* Derive evaporator delta and condenser load. */
    float evap_delta = s_state.evap_in_filtered - s_state.evap_out_filtered;
    if (!isfinite(evap_delta)) {
        evap_delta = 0.0f;
    }
    if (evap_delta < 0.0f) {
        evap_delta = 0.0f;
    }
    s_state.evap_delta_filtered = lowpass(s_state.evap_delta_filtered, evap_delta, 6.0f, dt_s);

    float condenser_norm = 0.0f;
    if (g_control_config.condenser_hot_delta_threshold > 0.01f) {
        condenser_norm = s_state.evap_delta_filtered / g_control_config.condenser_hot_delta_threshold;
    }
    s_state.condenser_load = lowpass(s_state.condenser_load, clampf(condenser_norm, 0.0f, 1.0f), 6.0f, dt_s);

    /* Estimate torque / texture */
    float torque_ref = fmaxf(s_state.torque_baseline,
                             g_control_config.estimator_torque_baseline_floor);
    float torque_norm = 0.0f;
    if (torque_ref > 0.0f) {
        float scale = g_control_config.estimator_torque_norm_scale;
        if (!isfinite(scale) || scale <= 0.0f) {
            scale = 1.0f;
        }
        torque_norm = (s_state.motor_current_filtered - torque_ref) / (torque_ref * scale);
        torque_norm = clampf(torque_norm, 0.0f, 1.0f);
    }
    float freeze_factor = clampf((0.0f - s_state.bowl_temp_filtered) / 4.0f, 0.0f, 1.0f);
    float torque_weight = g_control_config.estimator_texture_torque_weight;
    float freeze_weight = g_control_config.estimator_texture_freeze_weight;
    float weight_sum = torque_weight + freeze_weight;
    if (!isfinite(weight_sum) || weight_sum <= 0.0f) {
        weight_sum = 1.0f;
    }
    float texture_target = ((torque_weight * torque_norm) + (freeze_weight * freeze_factor)) / weight_sum;
    texture_target = clampf(texture_target, 0.0f, 1.0f);
    float texture_tau = g_control_config.estimator_texture_lowpass_hz;
    if (!isfinite(texture_tau) || texture_tau <= 0.0f) {
        texture_tau = 2.5f;
    }
    s_state.texture_index = lowpass(s_state.texture_index, texture_target, texture_tau, dt_s);

    /* Icing index combines evaporator delta and torque rise. */
    float icing_span = g_control_config.icing_delta_threshold - g_control_config.icing_delta_exit_threshold;
    if (icing_span < 1.0f) {
        icing_span = 1.0f;
    }
    float icing_norm = (s_state.evap_delta_filtered - g_control_config.icing_delta_exit_threshold) / icing_span;
    icing_norm = clampf(icing_norm, 0.0f, 1.0f);
    float ice_target = (g_control_config.estimator_ice_delta_weight * icing_norm) +
                       (g_control_config.estimator_ice_torque_weight * torque_norm);
    ice_target = clampf(ice_target, 0.0f, 1.0f);
    float ice_tau = g_control_config.estimator_ice_lowpass_hz;
    if (!isfinite(ice_tau) || ice_tau <= 0.0f) {
        ice_tau = 5.0f;
    }
    s_state.ice_index = lowpass(s_state.ice_index, ice_target, ice_tau, dt_s);

    const control_mode_profile_t *profile = estimator_active_profile();
    float deice_enter = profile->deice_enter_ice_index;
    float deice_exit  = profile->deice_exit_ice_index;
    if (!isfinite(deice_enter)) {
        deice_enter = 0.6f;
    }
    if (!isfinite(deice_exit)) {
        deice_exit = 0.35f;
    }
    if (deice_enter < deice_exit) {
        deice_enter = deice_exit + 0.05f;
    }
    uint32_t enter_ms = g_control_config.estimator_ice_detect_ms;
    uint32_t clear_ms = g_control_config.estimator_ice_clear_ms;
    if (elapsed_ms > 0U) {
        if (s_state.ice_index > deice_enter) {
            s_state.icing_accum_ms += elapsed_ms;
            s_state.clear_accum_ms = 0U;
            if (enter_ms == 0U || s_state.icing_accum_ms > enter_ms) {
                s_state.needs_deicing = true;
            }
        } else if (s_state.ice_index < deice_exit) {
            s_state.clear_accum_ms += elapsed_ms;
            if (clear_ms == 0U || s_state.clear_accum_ms > clear_ms) {
                s_state.needs_deicing = false;
                s_state.icing_accum_ms = 0U;
            }
        } else {
            s_state.clear_accum_ms = 0U;
        }
    }

#if ENABLE_DEBUG_LOGGING
    if (!prev_deicing && s_state.needs_deicing) {
        int32_t ice_idx_centi = (int32_t)lroundf(s_state.ice_index * 100.0f);
        LOG_INFO("Estimator: icing threshold exceeded (index=%ld.%02ld)\n",
                 ice_idx_centi / 100L,
                 labs(ice_idx_centi % 100L));
    } else if (prev_deicing && !s_state.needs_deicing) {
        int32_t ice_clear_centi = (int32_t)lroundf(s_state.ice_index * 100.0f);
        LOG_INFO("Estimator: ice cleared (index=%ld.%02ld)\n",
                 ice_clear_centi / 100L,
                 labs(ice_clear_centi % 100L));
    }
#endif

    /* Volume estimator: update on slower cadence once seeded. */
    if (have_bowl) {
        s_state.volume_elapsed_ms += elapsed_ms;
        if (!s_state.volume_ready) {
            s_state.volume_last_temp = s_state.bowl_temp_filtered;
            s_state.volume_elapsed_ms = 0U;
            s_state.volume_ready = true;
        }

        if (s_state.volume_elapsed_ms >= g_control_config.volume_update_interval_ms) {
            float interval_s = (float)s_state.volume_elapsed_ms / 1000.0f;
            if (interval_s < 0.1f) {
                interval_s = 0.1f;
            }

            float delta_temp = s_state.bowl_temp_filtered - s_state.volume_last_temp;
            float slope = fabsf(delta_temp) / interval_s;

            float low_thresh = fabsf(g_control_config.volume_low_slope_threshold);
            float med_thresh = fabsf(g_control_config.volume_med_slope_threshold);
            if (low_thresh < med_thresh) {
                float tmp = low_thresh;
                low_thresh = med_thresh;
                med_thresh = tmp;
            }

            float target_volume = 1.0f;
            if (slope >= low_thresh) {
                target_volume = 0.0f;
            } else if (slope <= med_thresh) {
                target_volume = 1.0f;
            } else {
                float span = low_thresh - med_thresh;
                float ratio = (low_thresh - slope) / span;
                target_volume = clampf(ratio, 0.0f, 1.0f);
            }

            /* Thicker slush implies less available product. */
            float texture_factor = g_control_config.estimator_volume_texture_factor;
            target_volume = clampf(target_volume * (1.0f - texture_factor * s_state.texture_index), 0.0f, 1.0f);

            const float VOLUME_TAU_S = 25.0f;
            s_state.volume_fraction = clampf(lowpass(s_state.volume_fraction,
                                                     target_volume,
                                                     VOLUME_TAU_S,
                                                     interval_s),
                                             0.0f,
                                             1.0f);

            if (s_state.volume_fraction >= 0.66f) {
                s_state.volume_level = 2U;
            } else if (s_state.volume_fraction >= 0.33f) {
                s_state.volume_level = 1U;
            } else {
                s_state.volume_level = 0U;
            }

            s_state.volume_last_temp = s_state.bowl_temp_filtered;
            s_state.volume_elapsed_ms = 0U;
        }
    }

    s_state.prev_bowl_temp = s_state.bowl_temp_filtered;
}

float estimator_get_real_bowl_temp(void)
{
    return s_state.bowl_temp_filtered;
}

float estimator_get_texture_index(void)
{
    return clampf(s_state.texture_index, 0.0f, 1.0f);
}

float estimator_get_condenser_load(void)
{
    return clampf(s_state.condenser_load, 0.0f, 1.0f);
}

float estimator_get_volume(void)
{
    return clampf(s_state.volume_fraction, 0.0f, 1.0f);
}

uint8_t estimator_get_volume_estimate(void)
{
    return s_state.volume_level;
}

float estimator_get_ice_index(void)
{
    return clampf(s_state.ice_index, 0.0f, 1.0f);
}

bool estimator_needs_deicing(void)
{
    return s_state.needs_deicing;
}

void estimator_reset_volume(void)
{
    s_state.volume_fraction   = 1.0f;
    s_state.volume_level      = 2U;
    s_state.volume_elapsed_ms = 0U;
    s_state.volume_ready      = false;
    s_state.volume_last_temp  = s_state.bowl_temp_filtered;
}

void update_slush_torque_reference(void)
{
    if (s_state.motor_current_filtered > 0.05f) {
        s_state.torque_baseline = s_state.motor_current_filtered;
        s_state.torque_ready    = true;
    }
    s_state.needs_deicing   = false;
    s_state.icing_accum_ms  = 0U;
    s_state.clear_accum_ms  = 0U;
}

float estimator_get_torque_baseline(void)
{
    return s_state.torque_baseline;
}

/**
 * @brief Emit a once-per-second debug snapshot used by the manual FSM mode.
 *
 * The output intentionally keeps labels terse so that the full line fits
 * comfortably inside a serial console while still communicating the same
 * information.  The abbreviations used are documented below for quick
 * reference:
 * - MC  : motor current (A)
 * - BT  : filtered bowl temperature (°C)
 * - EI  : evaporator inlet temperature (°C)
 * - EO  : evaporator outlet temperature (°C)
 * - AM  : auger motor speed (RPM)
 * - CP  : compressor speed (RPM)
 * - LUX : ring light brightness level
 * - FM  : freeze mode mnemonic (OFF/GRN/WHT/LBL/DBL)
 * - EB  : estimated true bowl temperature (°C)
 * - VOL : estimated volume fraction [0,1]
 * - TXT : texture index [0,1]
 * - CON : condenser load [0,1]
 * - ICE : icing index [0,1]
 * - TB  : torque baseline (A)
 */
void estimator_print_debug_data(uint32_t now_ms)
{
    (void)now_ms;
#if ENABLE_DEBUG_LOGGING
    static uint32_t last_print_ms = 0U;
    /* Throttle the print to once every ESTIMATOR_DEBUG_PERIOD_MS milliseconds. */
    if ((now_ms - last_print_ms) < ESTIMATOR_DEBUG_PERIOD_MS && last_print_ms != 0U) {
        return;
    }
    last_print_ms = now_ms;

    /* Break the millisecond counter into human-readable time fields. */
    uint32_t total_ms = now_ms;
    uint32_t ms   = total_ms % 1000U;
    uint32_t sec  = (total_ms / 1000U) % 60U;
    uint32_t min  = (total_ms / 60000U) % 60U;
    uint32_t hour = (total_ms / 3600000U) % 24U;

    /* Query live actuator speeds to accompany the filtered sensor values. */
    float motor_rpm = 0.0f;
    (void)get_auger_speed(&motor_rpm);
    uint16_t compressor_rpm = get_compressor_speed();

    /* Pull the current user-selected visuals to display alongside estimates. */
    uint8_t light_level = user_settings_get_ring_brightness();
    uint8_t freeze_mode = user_settings_get_effective_freeze_mode();
    if (freeze_mode > 4U) {
        freeze_mode = 0U;
    }
    static const char *freeze_labels[5] = {
        "OFF", "GRN", "WHT", "LBL", "DBL"
    };

    /* Emit the compact snapshot with all of the requested telemetry fields. */
    int32_t mc_milli     = (int32_t)lroundf(s_state.motor_current_filtered * 1000.0f);
    int32_t bowl_centi   = (int32_t)lroundf(s_state.bowl_temp_filtered * 100.0f);
    int32_t evap_in_centi  = (int32_t)lroundf(s_state.evap_in_filtered * 100.0f);
    int32_t evap_out_centi = (int32_t)lroundf(s_state.evap_out_filtered * 100.0f);
    int32_t motor_rpm_int  = (int32_t)lroundf(motor_rpm);
    int32_t real_bowl_centi = (int32_t)lroundf(estimator_get_real_bowl_temp() * 100.0f);
    int32_t volume_permil   = (int32_t)lroundf(estimator_get_volume() * 1000.0f);
    int32_t texture_permil  = (int32_t)lroundf(estimator_get_texture_index() * 1000.0f);
    int32_t condenser_permil = (int32_t)lroundf(estimator_get_condenser_load() * 1000.0f);
    int32_t ice_permil      = (int32_t)lroundf(estimator_get_ice_index() * 1000.0f);
    int32_t torque_milli    = (int32_t)lroundf(estimator_get_torque_baseline() * 1000.0f);

    LOG_INFO(
        "DBG [%02lu,%02lu,%02lu,%03lu] MC=%ld.%03ldA BT=%ld.%02ldC EI=%ld.%02ldC "
        "EO=%ld.%02ldC AM=%ldRPM CP=%04uRPM LUX=%03u FM=%s EB=%ld.%02ldC VOL=%ld.%03ld "
        "TXT=%ld.%03ld CON=%ld.%03ld ICE=%ld.%03ld TB=%ld.%03ld\n",
        (unsigned long)hour,
        (unsigned long)min,
        (unsigned long)sec,
        (unsigned long)ms,
        mc_milli / 1000L, labs(mc_milli % 1000L),
        bowl_centi / 100L, labs(bowl_centi % 100L),
        evap_in_centi / 100L, labs(evap_in_centi % 100L),
        evap_out_centi / 100L, labs(evap_out_centi % 100L),
        motor_rpm_int,
        (unsigned)compressor_rpm,
        (unsigned)light_level,
        freeze_labels[freeze_mode],
        real_bowl_centi / 100L, labs(real_bowl_centi % 100L),
        volume_permil / 1000L, labs(volume_permil % 1000L),
        texture_permil / 1000L, labs(texture_permil % 1000L),
        condenser_permil / 1000L, labs(condenser_permil % 1000L),
        ice_permil / 1000L, labs(ice_permil % 1000L),
        torque_milli / 1000L, labs(torque_milli % 1000L));
#endif /* ENABLE_DEBUG_LOGGING */
}

#else /* ENABLE_CONTROL_FSM */

void estimator_init(void) {}
void estimator_update(void) {}
float estimator_get_real_bowl_temp(void) { return 0.0f; }
float estimator_get_texture_index(void) { return 0.0f; }
float estimator_get_condenser_load(void) { return 0.0f; }
float estimator_get_volume(void) { return 0.0f; }
uint8_t estimator_get_volume_estimate(void) { return 0U; }
float estimator_get_ice_index(void) { return 0.0f; }
bool estimator_needs_deicing(void) { return false; }
void estimator_reset_volume(void) {}
void update_slush_torque_reference(void) {}
float estimator_get_torque_baseline(void) { return 0.0f; }
void estimator_print_debug_data(uint32_t now_ms) {(void)now_ms;}

#endif /* ENABLE_CONTROL_FSM */

