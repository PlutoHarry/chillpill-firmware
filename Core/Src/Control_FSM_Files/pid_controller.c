/**
 * @file pid_controller.c
 * @brief Compressor and auger controller for the ChillPill FSM.
 *
 * The controller combines temperature and texture regulation to command a
 * compressor frequency and auger speed every scheduler tick.  Gains and
 * operating limits are pulled from the shared ::g_control_config instance so
 * that tuning can be performed without recompiling the firmware.  The
 * implementation supports multiple dispense modes (cold drink plus three
 * slush textures), handles anti-windup, and responds to dispensing/refill
 * events detected by the volume estimator.
 */

#include "build_config.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "Control_FSM_Files/pid_controller.h"

#if ENABLE_CONTROL_FSM

#include "Control_FSM_Files/estimator.h"
#include "control_config.h"

/* Helpers ----------------------------------------------------------------- */

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

static float apply_deadband(float error, float band)
{
    if (band <= 0.0f) {
        return error;
    }
    float magnitude = fabsf(error);
    if (magnitude <= band) {
        return 0.0f;
    }
    if (error > 0.0f) {
        return error - band;
    }
    return error + band;
}

typedef struct
{
    float kp;
    float ki;
    float integrator;
} pid_axis_t;

typedef struct
{
    float temperature_c;
    float texture_index;
    float min_freq_hz;
    float max_freq_hz;
    float motor_rpm;
} pid_mode_config_t;

typedef struct
{
    float temperature_c;
    float texture_index;
    float volume_fraction;
    bool  valid;
} pid_measurement_t;

typedef struct
{
    pid_controller_mode_t mode;
    pid_mode_config_t     modes[PID_CONTROLLER_MODE_COUNT];
    pid_axis_t            temp_axis;
    pid_axis_t            texture_axis;
    pid_measurement_t     measurement;
    float                 compressor_freq_hz;
    float                 motor_rpm;
    bool                  saturated;
    uint32_t              last_update_ms;
    uint32_t              dispense_timer_ms;
    uint32_t              refill_hold_timer_ms;
    float                 previous_volume;
} pid_state_t;

static pid_state_t s_state;

/* Internal functions ------------------------------------------------------- */

static void pid_load_mode_config(void)
{
    const control_config_t *cfg = &g_control_config;

    /* Cold drink mode */
    pid_mode_config_t *cold = &s_state.modes[PID_CONTROLLER_MODE_COLD_DRINK];
    cold->temperature_c = cfg->cold_drink_temp_setpoint;
    cold->texture_index = 0.0f;
    cold->min_freq_hz   = cfg->steady_min_freq;
    cold->max_freq_hz   = fminf(cfg->steady_max_freq, cfg->cold_pull_down_max_freq);
    cold->motor_rpm     = cfg->cold_pull_down_motor_speed;

    /* Slush modes */
    pid_mode_config_t *light = &s_state.modes[PID_CONTROLLER_MODE_LIGHT_SLUSH];
    light->temperature_c = cfg->freeze_temp_light_setpoint;
    light->texture_index = cfg->texture_target_light;
    light->min_freq_hz   = cfg->steady_min_freq;
    light->max_freq_hz   = cfg->steady_max_freq;
    light->motor_rpm     = cfg->deice_motor_speed;

    pid_mode_config_t *medium = &s_state.modes[PID_CONTROLLER_MODE_MEDIUM_SLUSH];
    *medium = *light;
    medium->temperature_c = cfg->freeze_temp_medium_setpoint;
    medium->texture_index = cfg->texture_target_medium;

    pid_mode_config_t *heavy = &s_state.modes[PID_CONTROLLER_MODE_HEAVY_SLUSH];
    *heavy = *light;
    heavy->temperature_c = cfg->freeze_temp_heavy_setpoint;
    heavy->texture_index = cfg->texture_target_heavy;

    /* Gains shared across modes */
    s_state.temp_axis.kp = cfg->pid_temp_kp;
    s_state.temp_axis.ki = cfg->pid_temp_ki;
    s_state.temp_axis.integrator = 0.0f;

    s_state.texture_axis.kp = cfg->pid_texture_kp;
    s_state.texture_axis.ki = cfg->pid_texture_ki;
    s_state.texture_axis.integrator = 0.0f;
}

static const pid_mode_config_t *pid_current_mode_cfg(void)
{
    if (s_state.mode >= PID_CONTROLLER_MODE_COUNT) {
        s_state.mode = PID_CONTROLLER_MODE_LIGHT_SLUSH;
    }
    return &s_state.modes[s_state.mode];
}

static void pid_apply_volume_event(uint32_t elapsed_ms)
{
    const control_config_t *cfg = &g_control_config;

    if (s_state.dispense_timer_ms > 0U) {
        if (elapsed_ms >= s_state.dispense_timer_ms) {
            s_state.dispense_timer_ms = 0U;
        } else {
            s_state.dispense_timer_ms -= elapsed_ms;
        }
    }

    if (s_state.refill_hold_timer_ms > 0U) {
        if (elapsed_ms >= s_state.refill_hold_timer_ms) {
            s_state.refill_hold_timer_ms = 0U;
        } else {
            s_state.refill_hold_timer_ms -= elapsed_ms;
        }
    }

    /* Monitor new volume readings for dispense/refill events */
    float volume = s_state.measurement.volume_fraction;
    if (volume < 0.0f || volume > 1.0f || !s_state.measurement.valid) {
        return;
    }

    if (s_state.previous_volume < 0.0f) {
        s_state.previous_volume = volume;
        return;
    }

    float delta = volume - s_state.previous_volume;
    s_state.previous_volume = volume;

    if (cfg->pid_volume_dispense_threshold > 0.0f &&
        delta < -cfg->pid_volume_dispense_threshold) {
        s_state.dispense_timer_ms = cfg->pid_dispense_boost_decay_ms;
    } else if (cfg->pid_volume_refill_threshold > 0.0f &&
               delta > cfg->pid_volume_refill_threshold) {
        s_state.refill_hold_timer_ms = cfg->pid_refill_hold_ms;
        s_state.temp_axis.integrator = 0.0f;
        s_state.texture_axis.integrator = 0.0f;
    }
}

/* Public interface --------------------------------------------------------- */

void pid_controller_init(void)
{
    (void)memset(&s_state, 0, sizeof(s_state));
    s_state.mode = PID_CONTROLLER_MODE_LIGHT_SLUSH;
    s_state.previous_volume = -1.0f;
    pid_load_mode_config();
    pid_controller_reset();
}

void pid_controller_reset(void)
{
    const pid_mode_config_t *mode = pid_current_mode_cfg();
    s_state.temp_axis.integrator = 0.0f;
    s_state.texture_axis.integrator = 0.0f;
    s_state.measurement.valid = false;
    s_state.measurement.temperature_c = mode->temperature_c;
    s_state.measurement.texture_index = mode->texture_index;
    s_state.measurement.volume_fraction = 1.0f;
    s_state.compressor_freq_hz = mode->min_freq_hz;
    s_state.motor_rpm = mode->motor_rpm;
    s_state.saturated = false;
    s_state.last_update_ms = 0U;
    s_state.dispense_timer_ms = 0U;
    s_state.refill_hold_timer_ms = 0U;
    s_state.previous_volume = -1.0f;
}

void pid_controller_set_mode(pid_controller_mode_t mode)
{
    if (mode >= PID_CONTROLLER_MODE_COUNT) {
        mode = PID_CONTROLLER_MODE_LIGHT_SLUSH;
    }
    if (s_state.mode != mode) {
        s_state.mode = mode;
        s_state.temp_axis.integrator = 0.0f;
        s_state.texture_axis.integrator = 0.0f;
    }
    const pid_mode_config_t *cfg = pid_current_mode_cfg();
    s_state.compressor_freq_hz = clampf(s_state.compressor_freq_hz, cfg->min_freq_hz, cfg->max_freq_hz);
    s_state.motor_rpm = cfg->motor_rpm;
}

void pid_controller_set_targets(const pid_controller_targets_t *targets)
{
    if (targets == NULL) {
        return;
    }
    pid_mode_config_t *cfg = &s_state.modes[s_state.mode];
    cfg->temperature_c = targets->temperature_c;
    cfg->texture_index = clampf(targets->texture_index, 0.0f, 1.0f);
}

void pid_controller_set_target(int32_t temperature_c)
{
    pid_controller_targets_t targets = {
        .temperature_c = (float)temperature_c / 100.0f,
        .texture_index = s_state.modes[s_state.mode].texture_index,
    };
    pid_controller_set_targets(&targets);
}

void pid_controller_set_texture_target(int32_t texture_permille)
{
    pid_controller_targets_t targets = {
        .temperature_c = s_state.modes[s_state.mode].temperature_c,
        .texture_index = clampf((float)texture_permille / 1000.0f, 0.0f, 1.0f),
    };
    pid_controller_set_targets(&targets);
}

void pid_controller_update_state(float bowl_temp_c, float texture_index, float volume_fraction)
{
    s_state.measurement.temperature_c = bowl_temp_c;
    s_state.measurement.texture_index = clampf(texture_index, 0.0f, 1.0f);
    if (volume_fraction >= 0.0f) {
        s_state.measurement.volume_fraction = clampf(volume_fraction, 0.0f, 1.0f);
    }
    s_state.measurement.valid = true;
}

void pid_controller_update_measurements(int32_t cabinet_temp_c, int32_t evaporator_temp_c)
{
    /* Interpret inputs as centi-degrees and permille respectively. */
    float temp = (float)cabinet_temp_c / 100.0f;
    float texture = (float)evaporator_temp_c / 1000.0f;
    pid_controller_update_state(temp, texture, -1.0f);
}

void pid_controller_run(uint32_t now_ms)
{
    if (!s_state.measurement.valid) {
        return;
    }

    const control_config_t *cfg = &g_control_config;
    const pid_mode_config_t *mode = pid_current_mode_cfg();

    uint32_t elapsed_ms = (s_state.last_update_ms == 0U) ? 20U : (now_ms - s_state.last_update_ms);
    if (elapsed_ms > 500U) {
        elapsed_ms = 500U;
    }
    s_state.last_update_ms = now_ms;
    float dt = (float)elapsed_ms / 1000.0f;
    if (dt <= 0.0f) {
        dt = 0.02f;
    }

    /* Update volume estimate from estimator if available. */
    float volume_est = estimator_get_volume();
    if (volume_est >= 0.0f) {
        s_state.measurement.volume_fraction = clampf(volume_est, 0.0f, 1.0f);
    } else if (s_state.measurement.volume_fraction < 0.0f ||
               s_state.measurement.volume_fraction > 1.0f) {
        s_state.measurement.volume_fraction = 0.0f;
    }
    pid_apply_volume_event(elapsed_ms);

    float event_boost = 0.0f;
    if (cfg->pid_dispense_boost_decay_ms > 0U && s_state.dispense_timer_ms > 0U) {
        float ratio = (float)s_state.dispense_timer_ms / (float)cfg->pid_dispense_boost_decay_ms;
        event_boost += cfg->pid_dispense_boost_hz * clampf(ratio, 0.0f, 1.0f);
    }
    if (s_state.refill_hold_timer_ms > 0U) {
        event_boost += cfg->pid_refill_boost_hz;
    }

    float temp_error = mode->temperature_c - s_state.measurement.temperature_c;
    temp_error = apply_deadband(temp_error, cfg->pid_temp_band);
    float temp_p = s_state.temp_axis.kp * temp_error;

    float texture_error = 0.0f;
    float texture_p = 0.0f;
    bool  use_texture = mode->texture_index > 0.0f;
    if (use_texture) {
        texture_error = mode->texture_index - s_state.measurement.texture_index;
        texture_error = apply_deadband(texture_error, cfg->pid_texture_band);
        texture_p = s_state.texture_axis.kp * texture_error;
    }

    float base_output = mode->min_freq_hz + event_boost + temp_p + s_state.temp_axis.integrator;
    if (use_texture) {
        base_output += texture_p + s_state.texture_axis.integrator;
    }

    float unclamped = base_output;
    float clamped = clampf(unclamped, mode->min_freq_hz, mode->max_freq_hz);
    bool sat_high = (clamped >= mode->max_freq_hz - 0.05f) && (unclamped > clamped);
    bool sat_low  = (clamped <= mode->min_freq_hz + 0.05f) && (unclamped < clamped);

    bool allow_temp_integration = (s_state.refill_hold_timer_ms == 0U);
    if (allow_temp_integration) {
        if ((sat_high && temp_error > 0.0f) || (sat_low && temp_error < 0.0f)) {
            allow_temp_integration = false;
        }
    }
    if (allow_temp_integration) {
        s_state.temp_axis.integrator += temp_error * s_state.temp_axis.ki * dt;
        s_state.temp_axis.integrator = clampf(s_state.temp_axis.integrator,
                                             -cfg->pid_integrator_limit,
                                             cfg->pid_integrator_limit);
    }

    bool allow_texture_integration = use_texture && (s_state.refill_hold_timer_ms == 0U);
    if (allow_texture_integration) {
        if ((sat_high && texture_error > 0.0f) || (sat_low && texture_error < 0.0f)) {
            allow_texture_integration = false;
        }
    }
    if (allow_texture_integration) {
        s_state.texture_axis.integrator += texture_error * s_state.texture_axis.ki * dt;
        s_state.texture_axis.integrator = clampf(s_state.texture_axis.integrator,
                                                -cfg->pid_integrator_limit,
                                                cfg->pid_integrator_limit);
    }

    float command = mode->min_freq_hz + event_boost + temp_p + s_state.temp_axis.integrator;
    if (use_texture) {
        command += texture_p + s_state.texture_axis.integrator;
    }

    float limited = clampf(command, mode->min_freq_hz, mode->max_freq_hz);
    float delta   = limited - s_state.compressor_freq_hz;
    float max_delta = cfg->pid_freq_step;
    if (delta > max_delta) {
        limited = s_state.compressor_freq_hz + max_delta;
    } else if (delta < -max_delta) {
        limited = s_state.compressor_freq_hz - max_delta;
    }
    limited = clampf(limited, mode->min_freq_hz, mode->max_freq_hz);

    s_state.saturated = (limited != command);
    s_state.compressor_freq_hz = limited;

    float rpm = mode->motor_rpm;
    if (use_texture) {
        float rpm_adjust = clampf(texture_error * 10.0f, -5.0f, 5.0f);
        rpm = clampf(mode->motor_rpm + rpm_adjust, 0.0f, 120.0f);
    }
    s_state.motor_rpm = rpm;
}

pid_controller_output_t pid_controller_get_output_struct(void)
{
    pid_controller_output_t out = {
        .compressor_frequency_hz = s_state.compressor_freq_hz,
        .motor_rpm               = s_state.motor_rpm,
    };
    return out;
}

int32_t pid_controller_get_output(void)
{
    return (int32_t)lrintf(s_state.compressor_freq_hz);
}

float pid_controller_get_motor_speed(void)
{
    return s_state.motor_rpm;
}

bool pid_controller_is_saturated(void)
{
    return s_state.saturated;
}

#else /* ENABLE_CONTROL_FSM */

void pid_controller_init(void) {}
void pid_controller_reset(void) {}
void pid_controller_run(uint32_t now_ms) {(void)now_ms;}
void pid_controller_set_mode(pid_controller_mode_t mode) {(void)mode;}
void pid_controller_set_targets(const pid_controller_targets_t *targets)
{
    (void)targets;
}
void pid_controller_set_target(int32_t temperature_c)
{
    (void)temperature_c;
}
void pid_controller_set_texture_target(int32_t texture_permille)
{
    (void)texture_permille;
}
void pid_controller_update_state(float bowl_temp_c, float texture_index, float volume_fraction)
{
    (void)bowl_temp_c;
    (void)texture_index;
    (void)volume_fraction;
}
void pid_controller_update_measurements(int32_t cabinet_temp_c, int32_t evaporator_temp_c)
{
    (void)cabinet_temp_c;
    (void)evaporator_temp_c;
}
pid_controller_output_t pid_controller_get_output_struct(void)
{
    pid_controller_output_t out = {0.0f, 0.0f};
    return out;
}
int32_t pid_controller_get_output(void)
{
    return 0;
}
float pid_controller_get_motor_speed(void)
{
    return 0.0f;
}
bool pid_controller_is_saturated(void)
{
    return false;
}

#endif /* ENABLE_CONTROL_FSM */

