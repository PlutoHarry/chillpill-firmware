/**
 * @file estimator.c
 * @brief Sensor fusion and state estimators for the slush machine controller.
 *
 * The estimator derives higher level metrics from the raw sensors so the FSM
 * and control loops can make informed decisions.  Implementation details are
 * intentionally kept private to this file; other modules must rely on the
 * public API in estimator.h.
 */

#include "build_config.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "Control_FSM_Files/estimator.h"

#if ENABLE_CONTROL_FSM

#include "Core_Files/actuators.h"
#include "Core_Files/sensors.h"

#ifndef CLAMP
#define CLAMP(v, lo, hi) (((v) < (lo)) ? (lo) : (((v) > (hi)) ? (hi) : (v)))
#endif

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

#define ESTIMATOR_UPDATE_PERIOD_S  (0.02f)
#define ESTIMATOR_UPDATE_PERIOD_MS (20U)
#define COMP_BASELINE_BINS         (5U)

/* ------------------------------------------------------------------------- */
/* Configuration                                                             */
/* ------------------------------------------------------------------------- */
typedef struct
{
    float nominal_compressor_rpm;       /**< Reference compressor speed used for normalisation. */
    float nominal_fan_rpm;              /**< Reference fan speed to weight condenser assistance. */
    float bowl_filter_alpha;            /**< Exponential smoothing factor for the noisy bowl NTC. */
    float ambient_correction_gain;      /**< Compensation for ambient heating of the bowl wall. */
    float led_heat_gain;                /**< Adjustment for LED heating interacting with the evaporator. */
    float evap_delta_weight;            /**< Weight of evaporator delta when fusing the temperatures. */
    float bowl_weight;                  /**< Weight applied to the filtered bowl temperature in the fusion. */
    float derivative_alpha;             /**< Low pass filter on dT/dt to avoid derivative noise. */
    float delta_filter_alpha;           /**< Filter for the normalised evaporator delta. */
    float torque_filter_alpha;          /**< Filter applied to the torque index. */
    float volume_filter_alpha;          /**< Filter applied to the continuous volume estimate. */
    float condenser_filter_alpha;       /**< Filter applied to the condenser load metric. */
    float slope_scale;                  /**< Scale used to normalise the bowl temperature slope. */
    float delta_min;                    /**< Lower bound for normalised evaporator delta. */
    float delta_max;                    /**< Upper bound for normalised evaporator delta. */
    float torque_baseline_default;      /**< Default auger current baseline at power-up. */
    float baseline_adapt_alpha;         /**< Slow adaptation factor for evaporator delta baselines. */
    float torque_baseline_adapt_alpha;  /**< Adaptation factor for torque baseline in low texture states. */
    float baseline_learn_min_comp_rpm;  /**< Minimum compressor RPM before baseline learning occurs. */
    float baseline_learn_min_delta;     /**< Minimum delta required to trust baseline learning. */
    float ice_weight;                   /**< Relative weighting between delta and torque in ice index. */
    float ice_threshold;                /**< Ice index threshold before declaring a de-icing request. */
    uint32_t ice_duration_ms;           /**< Duration above threshold required to flag de-icing. */
    float volume_weight_delta;          /**< Weight for evaporator coverage in volume estimate. */
    float volume_weight_slope;          /**< Weight for bowl cooling slope in volume estimate. */
    float volume_weight_torque;         /**< Weight for torque contribution in volume estimate. */
    float volume_medium_threshold;      /**< Threshold for reporting medium volume. */
    float volume_high_threshold;        /**< Threshold for reporting high volume. */
    float refill_temp_jump;             /**< Temperature step that signals a likely refill event. */
    float refill_delta_drop;            /**< Drop in delta_norm considered evidence of refill. */
    float refill_torque_drop;           /**< Torque drop considered evidence of refill. */
    uint32_t refill_grace_ms;           /**< Cool-down before another refill trigger is accepted. */
    float steady_slope_threshold;       /**< |dT/dt| threshold for considering the bowl steady. */
    uint32_t steady_time_ms;            /**< Time the bowl slope must stay steady. */
    float texture_torque_weight;        /**< Blend between torque and steady-state for texture. */
    float texture_min_auger_rpm;        /**< Ignore torque adaptation when the auger is stopped. */
} estimator_config_t;

/* Compressor speed bins for storing different evaporator delta baselines. */
typedef struct
{
    float rpm_min;         /**< Inclusive lower bound for the bin. */
    float rpm_max;         /**< Exclusive upper bound for the bin. */
    float baseline_delta;  /**< Learnt evaporator delta at the given speed range. */
} baseline_slot_t;

/* Most recent sensor inputs (already filtered for communication hiccups). */
typedef struct
{
    float evap_bottom;     /**< Evaporator inlet temperature (°C). */
    float evap_top;        /**< Evaporator outlet temperature (°C). */
    float bowl_probe;      /**< Raw bowl temperature (°C). */
    float motor_current;   /**< Auger motor current (A). */
    float fan_rpm;         /**< Combined condenser fan speed (RPM). */
    float auger_rpm;       /**< Auger motor speed feedback (RPM). */
    float compressor_rpm;  /**< Commanded compressor speed (RPM). */
} estimator_inputs_t;

/* Cached copies of the last valid sensor samples. */
typedef struct
{
    float evap_bottom;
    float evap_top;
    float bowl_probe;
    float motor_current;
    float fan_one;
    float fan_two;
    float auger_rpm;
} sensor_cache_t;

/* Internal estimator state snapshot. */
typedef struct
{
    bool initialised;                    /**< Guard flag for lazy initialisation. */

    sensor_cache_t cache;                /**< Last valid raw sensor readings. */
    estimator_inputs_t inputs;           /**< Latest fused sensor inputs. */

    float filtered_bowl_probe;           /**< Low pass filtered bowl probe temperature. */
    float real_bowl_temp;                /**< Reconstructed estimate of the true liquid temperature. */
    float last_bowl_temp;                /**< Previous real bowl temperature for derivative. */
    float real_bowl_slope;               /**< Filtered derivative of the real bowl temperature. */

    float delta_norm_instant;            /**< Instantaneous normalised evaporator delta. */
    float delta_norm_filtered;           /**< Filtered normalised evaporator delta. */

    float condenser_load;                /**< Filtered condenser load index. */

    float torque_baseline;               /**< Learnt torque baseline from recent calibration. */
    float torque_index_instant;          /**< Instantaneous torque index. */
    float torque_index_filtered;         /**< Filtered torque index. */

    float texture_index;                 /**< Slush texture estimate in the [0,1] interval. */

    float volume_raw;                    /**< Raw volume estimate before smoothing. */
    float volume_filtered;               /**< Filtered continuous volume estimate. */
    uint8_t volume_level;                /**< Discrete volume level (0=low,1=med,2=high). */

    float ice_index;                     /**< Ice accumulation metric. */

    float last_delta_norm;               /**< Last filtered delta used for refill detection. */
    float last_torque_index;             /**< Last filtered torque used for refill detection. */

    uint32_t steady_timer_ms;            /**< Time the bowl slope has stayed within steady bounds. */
    uint32_t ice_timer_ms;               /**< Time the ice index has remained above the threshold. */
    uint32_t refill_grace_ms;            /**< Cool-down timer after a detected refill. */

    baseline_slot_t delta_slots[COMP_BASELINE_BINS]; /**< Adaptable baseline slots. */
} estimator_state_t;

static estimator_state_t s_estimator;

static const estimator_config_t s_config =
{
    .nominal_compressor_rpm = 3600.0f,   /* ≈60 Hz */
    .nominal_fan_rpm = 2000.0f,
    .bowl_filter_alpha = 0.18f,
    .ambient_correction_gain = 0.22f,
    .led_heat_gain = 0.10f,
    .evap_delta_weight = 0.35f,
    .bowl_weight = 0.62f,
    .derivative_alpha = 0.12f,
    .delta_filter_alpha = 0.20f,
    .torque_filter_alpha = 0.20f,
    .volume_filter_alpha = 0.08f,
    .condenser_filter_alpha = 0.15f,
    .slope_scale = 0.05f,                /* °C/s */
    .delta_min = 1.0f,
    .delta_max = 8.0f,
    .torque_baseline_default = 2.0f,     /* A */
    .baseline_adapt_alpha = 0.002f,
    .torque_baseline_adapt_alpha = 0.01f,
    .baseline_learn_min_comp_rpm = 500.0f,
    .baseline_learn_min_delta = 0.10f,
    .ice_weight = 0.6f,
    .ice_threshold = 0.7f,
    .ice_duration_ms = 300000U,
    .volume_weight_delta = 0.45f,
    .volume_weight_slope = 0.35f,
    .volume_weight_torque = 0.20f,
    .volume_medium_threshold = 0.45f,
    .volume_high_threshold = 0.70f,
    .refill_temp_jump = 1.5f,
    .refill_delta_drop = 0.35f,
    .refill_torque_drop = 0.20f,
    .refill_grace_ms = 60000U,
    .steady_slope_threshold = 0.02f,
    .steady_time_ms = 120000U,
    .texture_torque_weight = 0.65f,
    .texture_min_auger_rpm = 20.0f,
};

/* ------------------------------------------------------------------------- */
/* Helper forward declarations                                              */
/* ------------------------------------------------------------------------- */
static float sample_sensor(uint8_t (*fn)(float *), float *cache);
static void fetch_inputs(estimator_inputs_t *inputs);
static size_t pick_baseline_slot(float rpm);
static void adapt_baseline(size_t slot_index, float delta_norm);
static float compute_delta_norm(float evap_in, float evap_out, float compressor_rpm);
static void update_real_bowl_temperature(const estimator_inputs_t *inputs, float dt);
static void update_condenser_load(const estimator_inputs_t *inputs);
static void update_texture_and_torque(const estimator_inputs_t *inputs);
static void update_volume_estimate(void);
static void update_ice_index(const estimator_inputs_t *inputs, float dt);
static void detect_refill(const estimator_inputs_t *inputs);

/* ------------------------------------------------------------------------- */
/* Public API                                                                */
/* ------------------------------------------------------------------------- */
void estimator_init(void)
{
    memset(&s_estimator, 0, sizeof(s_estimator));

    const baseline_slot_t defaults[COMP_BASELINE_BINS] =
    {
        { .rpm_min = 0.0f,    .rpm_max = 1200.0f, .baseline_delta = 2.0f },
        { .rpm_min = 1200.0f, .rpm_max = 2000.0f, .baseline_delta = 3.0f },
        { .rpm_min = 2000.0f, .rpm_max = 2800.0f, .baseline_delta = 4.0f },
        { .rpm_min = 2800.0f, .rpm_max = 3600.0f, .baseline_delta = 5.0f },
        { .rpm_min = 3600.0f, .rpm_max = 4800.0f, .baseline_delta = 6.0f },
    };

    for (size_t i = 0; i < ARRAY_LEN(defaults); ++i)
    {
        s_estimator.delta_slots[i] = defaults[i];
    }

    s_estimator.torque_baseline = s_config.torque_baseline_default;
    s_estimator.initialised = true;
}

void estimator_update(void)
{
    if (!s_estimator.initialised)
    {
        estimator_init();
    }

    estimator_inputs_t inputs = {0};
    fetch_inputs(&inputs);
    s_estimator.inputs = inputs;

    const float dt = ESTIMATOR_UPDATE_PERIOD_S;

    float delta_norm = compute_delta_norm(inputs.evap_bottom, inputs.evap_top, inputs.compressor_rpm);
    s_estimator.delta_norm_instant = delta_norm;
    s_estimator.delta_norm_filtered = (1.0f - s_config.delta_filter_alpha) * s_estimator.delta_norm_filtered +
                                      s_config.delta_filter_alpha * delta_norm;

    update_real_bowl_temperature(&inputs, dt);
    update_condenser_load(&inputs);
    update_texture_and_torque(&inputs);
    update_ice_index(&inputs, dt);
    update_volume_estimate();
    detect_refill(&inputs);

    s_estimator.last_bowl_temp = s_estimator.real_bowl_temp;
    s_estimator.last_delta_norm = s_estimator.delta_norm_filtered;
    s_estimator.last_torque_index = s_estimator.torque_index_filtered;

    if (s_estimator.refill_grace_ms >= ESTIMATOR_UPDATE_PERIOD_MS)
    {
        s_estimator.refill_grace_ms -= ESTIMATOR_UPDATE_PERIOD_MS;
    }
    else
    {
        s_estimator.refill_grace_ms = 0U;
    }
}

float estimator_get_real_bowl_temp(void)
{
    return s_estimator.real_bowl_temp;
}

float estimator_get_texture_index(void)
{
    return s_estimator.texture_index;
}

float estimator_get_condenser_load(void)
{
    return s_estimator.condenser_load;
}

float estimator_get_volume(void)
{
    return s_estimator.volume_filtered;
}

uint8_t estimator_get_volume_estimate(void)
{
    return s_estimator.volume_level;
}

float estimator_get_ice_index(void)
{
    return s_estimator.ice_index;
}

bool estimator_needs_deicing(void)
{
    return s_estimator.ice_timer_ms >= s_config.ice_duration_ms;
}

void estimator_reset_volume(void)
{
    s_estimator.volume_raw = 0.0f;
    s_estimator.volume_filtered = 0.0f;
    s_estimator.volume_level = 0U;
    s_estimator.refill_grace_ms = s_config.refill_grace_ms;
}

void update_slush_torque_reference(void)
{
    float current = s_estimator.inputs.motor_current;
    if (current <= 0.0f)
    {
        current = sample_sensor(get_motor_current, &s_estimator.cache.motor_current);
    }

    if (current > 0.1f)
    {
        s_estimator.torque_baseline = current;
    }
}

/* ------------------------------------------------------------------------- */
/* Helper implementations                                                    */
/* ------------------------------------------------------------------------- */
static float sample_sensor(uint8_t (*fn)(float *), float *cache)
{
    float value = (cache != NULL) ? *cache : 0.0f;

    if ((fn != NULL) && (cache != NULL))
    {
        float sample = *cache;
        if (fn(&sample) == 0U)
        {
            *cache = sample;
            value = sample;
        }
    }

    return value;
}

static void fetch_inputs(estimator_inputs_t *inputs)
{
    if (inputs == NULL)
    {
        return;
    }

    inputs->evap_bottom = sample_sensor(get_evap_in_temp, &s_estimator.cache.evap_bottom);
    inputs->evap_top = sample_sensor(get_evap_out_temp, &s_estimator.cache.evap_top);
    inputs->bowl_probe = sample_sensor(get_bowl_temp, &s_estimator.cache.bowl_probe);
    inputs->motor_current = sample_sensor(get_motor_current, &s_estimator.cache.motor_current);

    float fan_one = sample_sensor(get_fan_one_speed, &s_estimator.cache.fan_one);
    float fan_two = sample_sensor(get_fan_two_speed, &s_estimator.cache.fan_two);
    if ((fan_one > 0.0f) && (fan_two > 0.0f))
    {
        inputs->fan_rpm = 0.5f * (fan_one + fan_two);
    }
    else if (fan_one > 0.0f)
    {
        inputs->fan_rpm = fan_one;
    }
    else
    {
        inputs->fan_rpm = fan_two;
    }

    inputs->auger_rpm = sample_sensor(get_auger_speed, &s_estimator.cache.auger_rpm);
    inputs->compressor_rpm = (float)get_compressor_speed();
}

static size_t pick_baseline_slot(float rpm)
{
    for (size_t i = 0; i < ARRAY_LEN(s_estimator.delta_slots); ++i)
    {
        if ((rpm >= s_estimator.delta_slots[i].rpm_min) && (rpm < s_estimator.delta_slots[i].rpm_max))
        {
            return i;
        }
    }
    return ARRAY_LEN(s_estimator.delta_slots) - 1U;
}

static void adapt_baseline(size_t slot_index, float delta_norm)
{
    if (slot_index >= ARRAY_LEN(s_estimator.delta_slots))
    {
        return;
    }

    float *baseline = &s_estimator.delta_slots[slot_index].baseline_delta;
    *baseline = (1.0f - s_config.baseline_adapt_alpha) * (*baseline) + s_config.baseline_adapt_alpha * delta_norm;
}

static float compute_delta_norm(float evap_in, float evap_out, float compressor_rpm)
{
    const float delta = evap_out - evap_in;
    if (compressor_rpm < 100.0f)
    {
        return 0.0f;
    }

    const float rpm = CLAMP(compressor_rpm, 200.0f, 6000.0f);
    return delta * (s_config.nominal_compressor_rpm / rpm);
}

static void update_real_bowl_temperature(const estimator_inputs_t *inputs, float dt)
{
    if (inputs == NULL)
    {
        return;
    }

    s_estimator.filtered_bowl_probe = (1.0f - s_config.bowl_filter_alpha) * s_estimator.filtered_bowl_probe +
                                      s_config.bowl_filter_alpha * inputs->bowl_probe;

    const float fan_norm = (s_config.nominal_fan_rpm > 0.0f) ?
                               CLAMP(inputs->fan_rpm / s_config.nominal_fan_rpm, 0.0f, 2.0f) :
                               1.0f;
    const float comp_norm = (s_config.nominal_compressor_rpm > 0.0f) ?
                                CLAMP(inputs->compressor_rpm / s_config.nominal_compressor_rpm, 0.1f, 2.5f) :
                                1.0f;

    const float delta_evp = inputs->evap_top - inputs->evap_bottom;
    const float evaporator_component = 0.5f * (inputs->evap_top + inputs->evap_bottom) +
                                       s_config.evap_delta_weight * delta_evp;

    float ambient_offset = s_config.ambient_correction_gain * (s_estimator.filtered_bowl_probe - inputs->evap_bottom);
    float led_offset = s_config.led_heat_gain * delta_evp * (1.0f - CLAMP(fan_norm, 0.0f, 1.0f));

    float fused = s_config.bowl_weight * s_estimator.filtered_bowl_probe +
                  (1.0f - s_config.bowl_weight) * evaporator_component;

    fused -= ambient_offset;
    fused -= led_offset;
    fused -= 0.5f * (comp_norm - 1.0f);

    s_estimator.real_bowl_temp = fused;

    float slope = 0.0f;
    if (dt > 0.0f)
    {
        slope = (s_estimator.real_bowl_temp - s_estimator.last_bowl_temp) / dt;
    }

    s_estimator.real_bowl_slope = (1.0f - s_config.derivative_alpha) * s_estimator.real_bowl_slope +
                                  s_config.derivative_alpha * slope;

    if (fabsf(s_estimator.real_bowl_slope) < s_config.steady_slope_threshold)
    {
        if (s_estimator.steady_timer_ms < s_config.steady_time_ms)
        {
            s_estimator.steady_timer_ms += ESTIMATOR_UPDATE_PERIOD_MS;
        }
    }
    else
    {
        s_estimator.steady_timer_ms = 0U;
    }
}

static void update_condenser_load(const estimator_inputs_t *inputs)
{
    if (inputs == NULL)
    {
        return;
    }

    const float comp_scale = (s_config.nominal_compressor_rpm > 0.0f) ?
                                 CLAMP(inputs->compressor_rpm / s_config.nominal_compressor_rpm, 0.2f, 2.5f) :
                                 1.0f;
    const float fan_scale = (s_config.nominal_fan_rpm > 0.0f) ?
                                CLAMP(inputs->fan_rpm / s_config.nominal_fan_rpm, 0.0f, 1.5f) :
                                1.0f;

    const float load_instant = s_estimator.delta_norm_filtered * comp_scale *
                               (1.0f + 0.3f * (1.0f - CLAMP(fan_scale, 0.0f, 1.0f)));

    s_estimator.condenser_load = (1.0f - s_config.condenser_filter_alpha) * s_estimator.condenser_load +
                                 s_config.condenser_filter_alpha * CLAMP(load_instant, 0.0f, 10.0f);
}

static void update_texture_and_torque(const estimator_inputs_t *inputs)
{
    if (inputs == NULL)
    {
        return;
    }

    const float baseline = fmaxf(s_estimator.torque_baseline, 0.2f);
    float torque_index = 0.0f;
    if (inputs->auger_rpm > s_config.texture_min_auger_rpm)
    {
        torque_index = (inputs->motor_current - baseline) / baseline;
    }
    torque_index = CLAMP(torque_index, 0.0f, 1.5f);

    s_estimator.torque_index_instant = torque_index;
    s_estimator.torque_index_filtered = (1.0f - s_config.torque_filter_alpha) * s_estimator.torque_index_filtered +
                                        s_config.torque_filter_alpha * torque_index;

    const float steady_fraction = CLAMP((float)s_estimator.steady_timer_ms / (float)s_config.steady_time_ms, 0.0f, 1.0f);
    const float texture = s_config.texture_torque_weight * CLAMP(s_estimator.torque_index_filtered, 0.0f, 1.0f) +
                          (1.0f - s_config.texture_torque_weight) * steady_fraction;
    s_estimator.texture_index = CLAMP(texture, 0.0f, 1.0f);

    if ((s_estimator.texture_index < 0.2f) && (inputs->auger_rpm > s_config.texture_min_auger_rpm))
    {
        const float adapt_alpha = s_config.torque_baseline_adapt_alpha;
        s_estimator.torque_baseline = (1.0f - adapt_alpha) * s_estimator.torque_baseline +
                                      adapt_alpha * inputs->motor_current;
    }
}

static void update_volume_estimate(void)
{
    float norm_delta = 0.0f;
    if (s_config.delta_max > s_config.delta_min)
    {
        norm_delta = (s_estimator.delta_norm_filtered - s_config.delta_min) /
                     (s_config.delta_max - s_config.delta_min);
    }
    norm_delta = CLAMP(norm_delta, 0.0f, 1.0f);

    const float slope_mag = fabsf(s_estimator.real_bowl_slope);
    const float norm_slope = CLAMP(slope_mag / s_config.slope_scale, 0.0f, 1.0f);

    const float torque_idx = CLAMP(s_estimator.torque_index_filtered, 0.0f, 1.0f);

    float w1 = s_config.volume_weight_delta * (1.0f - 0.5f * s_estimator.ice_index);
    float w2 = s_config.volume_weight_slope;
    float w3 = s_config.volume_weight_torque;
    float w_sum = w1 + w2 + w3;
    if (w_sum < 1e-3f)
    {
        w1 = w2 = w3 = 1.0f / 3.0f;
        w_sum = 1.0f;
    }
    w1 /= w_sum;
    w2 /= w_sum;
    w3 /= w_sum;

    const float volume_raw = w1 * norm_delta + w2 * (1.0f - norm_slope) + w3 * (1.0f - torque_idx);
    s_estimator.volume_raw = CLAMP(volume_raw, 0.0f, 1.0f);

    s_estimator.volume_filtered = (1.0f - s_config.volume_filter_alpha) * s_estimator.volume_filtered +
                                  s_config.volume_filter_alpha * s_estimator.volume_raw;

    s_estimator.volume_filtered = CLAMP(s_estimator.volume_filtered, 0.0f, 1.0f);

    uint8_t level = 0U;
    if (s_estimator.volume_filtered >= s_config.volume_high_threshold)
    {
        level = 2U;
    }
    else if (s_estimator.volume_filtered >= s_config.volume_medium_threshold)
    {
        level = 1U;
    }
    s_estimator.volume_level = level;
}

static void update_ice_index(const estimator_inputs_t *inputs, float dt)
{
    (void)dt;
    if (inputs == NULL)
    {
        return;
    }

    const size_t slot = pick_baseline_slot(inputs->compressor_rpm);
    const float baseline_delta = s_estimator.delta_slots[slot].baseline_delta;

    float delta_term = 0.0f;
    if (baseline_delta > 0.1f)
    {
        delta_term = (s_estimator.delta_norm_filtered - baseline_delta) / baseline_delta;
    }
    delta_term = CLAMP(delta_term, 0.0f, 1.0f);

    const float baseline_torque = fmaxf(s_estimator.torque_baseline, 0.2f);
    float torque_term = 0.0f;
    if (inputs->motor_current > baseline_torque)
    {
        torque_term = (inputs->motor_current - baseline_torque) / baseline_torque;
    }
    torque_term = CLAMP(torque_term, 0.0f, 1.0f);

    const float ice_index = s_config.ice_weight * delta_term + (1.0f - s_config.ice_weight) * torque_term;
    s_estimator.ice_index = CLAMP(ice_index, 0.0f, 1.0f);

    if ((s_estimator.ice_index < 0.3f) &&
        (inputs->compressor_rpm > s_config.baseline_learn_min_comp_rpm) &&
        (s_estimator.delta_norm_filtered > s_config.baseline_learn_min_delta))
    {
        adapt_baseline(slot, s_estimator.delta_norm_filtered);
    }

    if (s_estimator.ice_index > s_config.ice_threshold)
    {
        if (s_estimator.ice_timer_ms < UINT32_MAX - ESTIMATOR_UPDATE_PERIOD_MS)
        {
            s_estimator.ice_timer_ms += ESTIMATOR_UPDATE_PERIOD_MS;
        }
    }
    else
    {
        s_estimator.ice_timer_ms = 0U;
    }
}

static void detect_refill(const estimator_inputs_t *inputs)
{
    (void)inputs;

    const float temp_jump = s_estimator.real_bowl_temp - s_estimator.last_bowl_temp;
    const float delta_drop = s_estimator.last_delta_norm - s_estimator.delta_norm_filtered;
    const float torque_drop = s_estimator.last_torque_index - s_estimator.torque_index_filtered;

    bool refill_detected = false;
    if (temp_jump > s_config.refill_temp_jump)
    {
        refill_detected = true;
    }
    if ((delta_drop > s_config.refill_delta_drop) && (torque_drop > s_config.refill_torque_drop))
    {
        refill_detected = true;
    }

    if (refill_detected && (s_estimator.refill_grace_ms == 0U))
    {
        estimator_reset_volume();
        s_estimator.steady_timer_ms = 0U;
    }
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

#endif /* ENABLE_CONTROL_FSM */

