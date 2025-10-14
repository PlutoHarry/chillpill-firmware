#include "fault_handler.h"

/**
 * @file fault_handler.c
 * @brief Centralised safety monitoring for the control FSM runtime.
 *
 * The fault handler collects raw telemetry from the hardware abstraction
 * layer, detects abnormal conditions, and takes immediate mitigation steps
 * (such as disabling actuators and driving the user-facing RGB indicator).
 *
 * To use the module:
 *   1. Call fault_handler_init() once during system start-up to reset all
 *      latched state and ensure the indicator is cleared.
 *   2. Call fault_handler_run(now_ms) from a 1 Hz (or faster) scheduler. The
 *      helper self-rates and only performs work every FAULT_RUN_INTERVAL_MS.
 *   3. Inspect the latched state with fault_handler_is_active() or
 *      fault_handler_get_highest_priority() to coordinate state transitions in
 *      the finite state machine.
 *   4. Once the FSM has handled the condition, clear the fault with
 *      fault_handler_clear() or fault_handler_clear_all().
 */

#if ENABLE_CONTROL_FSM

#include "actuators.h"
#include "lights.h"
#include "sensors.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/*                                CONFIGURATION                               */
/* -------------------------------------------------------------------------- */

/** Interval in milliseconds between consecutive monitoring runs. */
#define FAULT_RUN_INTERVAL_MS              1000U

/* Motor stall heuristics --------------------------------------------------- */

/** Motor current (in amperes) considered indicative of a stalled auger. */
#define MOTOR_STALL_CURRENT_A              2.2f
/** Minimum auger RPM that counts as "running" for stall detection. */
#define AUGER_MIN_RUNNING_RPM              8.0f
/** Number of consecutive detections required before latching a stall. */
#define MOTOR_STALL_DEBOUNCE_COUNT         2U    /* >1 s */

/* Fan monitoring ----------------------------------------------------------- */

/** Minimum RPM each fan must maintain while required. */
#define FAN_MIN_RPM                        250.0f
/** Auger RPM above which the fans are deemed necessary for airflow. */
#define FAN_REQUIRED_AUGER_RPM             5.0f
/** Seconds of missing feedback before declaring a fan fault. */
#define FAN_FAULT_DEBOUNCE_COUNT           5U    /* 5 s */

/* Compressor performance --------------------------------------------------- */

/** Compressor RPM threshold that indicates it is actively running. */
#define COMPRESSOR_MIN_ACTIVE_RPM          1000U
/** Expected minimum evaporator delta-T when the compressor is healthy. */
#define COMPRESSOR_MIN_DELTA_C             5.0f
/**
 * Number of seconds to allow the compressor to stabilise after start before
 * enforcing the performance check.
 */
#define COMPRESSOR_GRACE_SECONDS           240U /* 4 minutes */
/**
 * Window of poor performance (seconds) that must elapse before latching a
 * compressor fault once outside the grace period.
 */
#define COMPRESSOR_BAD_DELTA_SECONDS       60U  /* persistent window */

/* Over-temperature --------------------------------------------------------- */

/** Maximum permissible bowl temperature in degrees Celsius. */
#define BOWL_OVERTEMP_C                    35.0f
/** Seconds the cabinet may exceed the threshold before latching. */
#define OVERTEMP_DEBOUNCE_SECONDS          30U

/* -------------------------------------------------------------------------- */
/*                                 STATE                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Bit-field definitions used internally to track actuator sub-faults.
 */
enum
{
    ACT_FAULT_MOTOR      = (1u << 0), /**< Auger motor has faulted. */
    ACT_FAULT_FAN        = (1u << 1), /**< One or more fans has faulted. */
    ACT_FAULT_COMPRESSOR = (1u << 2), /**< Compressor has faulted. */
};

/** Latched set of FAULT_HANDLER_FAULT_* bits representing reported faults. */
static uint32_t fault_latch_bits = 0U;
/** Bitmask of ACT_FAULT_* entries corresponding to disabled actuators. */
static uint8_t actuator_fault_mask = 0U;
/** Highest priority fault currently latched. */
static fault_handler_fault_t highest_fault = FAULT_HANDLER_FAULT_NONE;

/** Timestamp (ms) of the previous monitoring run. */
static uint32_t last_run_ms = 0U;
/** Debounce counter used to recognise persistent auger stalls. */
static uint8_t motor_stall_counter = 0U;
/** Debounce counter used to recognise persistent fan failures. */
static uint8_t fan_fault_counter = 0U;
/** Seconds the compressor has been active during the current monitoring run. */
static uint32_t compressor_active_seconds = 0U;
/** Seconds the compressor delta-T has been below threshold while active. */
static uint32_t compressor_bad_delta_seconds = 0U;
/** Debounce counter tracking consecutive over-temperature readings. */
static uint8_t overtemp_counter = 0U;

/* -------------------------------------------------------------------------- */
/*                                 HELPERS                                    */
/* -------------------------------------------------------------------------- */

/**
 * @brief Convert a public fault enum into its associated latch bit.
 *
 * @param fault Fault category to convert.
 * @return Bit mask with a single bit corresponding to @p fault.
 */
static inline uint32_t fault_bit(fault_handler_fault_t fault)
{
    return (fault == FAULT_HANDLER_FAULT_NONE) ? 0U : (1u << (uint32_t)fault);
}

/**
 * @brief Determine whether a particular fault has already latched.
 *
 * @param fault Fault category to query.
 * @return true if latched, false otherwise.
 */
static bool fault_is_set(fault_handler_fault_t fault)
{
    return (fault_latch_bits & fault_bit(fault)) != 0U;
}

/**
 * @brief Update the cached highest priority fault based on the latch bits.
 */
static void recompute_highest_priority(void)
{
    if (fault_is_set(FAULT_HANDLER_FAULT_SENSOR))
    {
        highest_fault = FAULT_HANDLER_FAULT_SENSOR;
    }
    else if (fault_is_set(FAULT_HANDLER_FAULT_OVERTEMP))
    {
        highest_fault = FAULT_HANDLER_FAULT_OVERTEMP;
    }
    else if (fault_is_set(FAULT_HANDLER_FAULT_ACTUATOR))
    {
        highest_fault = FAULT_HANDLER_FAULT_ACTUATOR;
    }
    else if (fault_is_set(FAULT_HANDLER_FAULT_UNKNOWN))
    {
        highest_fault = FAULT_HANDLER_FAULT_UNKNOWN;
    }
    else
    {
        highest_fault = FAULT_HANDLER_FAULT_NONE;
    }
}

/**
 * @brief Drive the front RGB indicator to reflect the highest priority fault.
 */
static void update_rgb_indicator(void)
{
    fault_type_t mode = FAULT_OFF;

    if (fault_is_set(FAULT_HANDLER_FAULT_SENSOR))
    {
        mode = FAULT_COMPRESSOR;
    }
    else if (fault_is_set(FAULT_HANDLER_FAULT_OVERTEMP))
    {
        mode = FAULT_COMPRESSOR;
    }
    else if (fault_is_set(FAULT_HANDLER_FAULT_ACTUATOR))
    {
        if (actuator_fault_mask & ACT_FAULT_COMPRESSOR)
        {
            mode = FAULT_COMPRESSOR;
        }
        else if (actuator_fault_mask & ACT_FAULT_MOTOR)
        {
            mode = FAULT_MOTOR;
        }
        else if (actuator_fault_mask & ACT_FAULT_FAN)
        {
            mode = FAULT_FAN;
        }
        else
        {
            mode = FAULT_COMPRESSOR;
        }
    }
    else if (fault_is_set(FAULT_HANDLER_FAULT_UNKNOWN))
    {
        mode = FAULT_MOTOR;
    }

    set_front_rgb_fault(mode);
}

/**
 * @brief Latch a sensor fault and pre-emptively disable the major actuators.
 */
static void latch_sensor_fault(void)
{
    if (!fault_is_set(FAULT_HANDLER_FAULT_SENSOR))
    {
        set_motor_speed(0.0f);
        motor_disable();
        set_compressor_speed(0U);
        set_fan_speed(0U);
    }
    fault_handler_report(FAULT_HANDLER_FAULT_SENSOR);
}

/**
 * @brief Latch an auger motor fault and stop the motor output.
 */
static void latch_motor_fault(void)
{
    if ((actuator_fault_mask & ACT_FAULT_MOTOR) == 0U)
    {
        actuator_fault_mask |= ACT_FAULT_MOTOR;
        set_motor_speed(0.0f);
        motor_disable();
    }
    fault_handler_report(FAULT_HANDLER_FAULT_ACTUATOR);
}

/**
 * @brief Latch a fan fault and stop the fan output.
 */
static void latch_fan_fault(void)
{
    if ((actuator_fault_mask & ACT_FAULT_FAN) == 0U)
    {
        actuator_fault_mask |= ACT_FAULT_FAN;
        set_fan_speed(0U);
    }
    fault_handler_report(FAULT_HANDLER_FAULT_ACTUATOR);
}

/**
 * @brief Latch a compressor fault and command the compressor off.
 */
static void latch_compressor_fault(void)
{
    if ((actuator_fault_mask & ACT_FAULT_COMPRESSOR) == 0U)
    {
        actuator_fault_mask |= ACT_FAULT_COMPRESSOR;
        set_compressor_speed(0U);
    }
    fault_handler_report(FAULT_HANDLER_FAULT_ACTUATOR);
}

/**
 * @brief Latch an over-temperature fault and shut down cooling and mixing.
 */
static void latch_overtemp_fault(void)
{
    if (!fault_is_set(FAULT_HANDLER_FAULT_OVERTEMP))
    {
        set_compressor_speed(0U);
        set_motor_speed(0.0f);
    }
    fault_handler_report(FAULT_HANDLER_FAULT_OVERTEMP);
}

/**
 * @brief Determine whether the monitoring pass should run at @p now_ms.
 *
 * @param now_ms Current firmware tick in milliseconds.
 * @return true when the internal period has elapsed, false otherwise.
 */
static bool should_run_now(uint32_t now_ms)
{
    if (last_run_ms == 0U)
    {
        last_run_ms = now_ms;
        return true;
    }

    uint32_t elapsed = now_ms - last_run_ms;
    if (elapsed >= FAULT_RUN_INTERVAL_MS)
    {
        last_run_ms = now_ms;
        return true;
    }

    return false;
}

/**
 * @brief Sanity-check a floating point reading for numerical validity.
 *
 * @param v Value to test.
 * @return true when @p v is invalid (NaN, infinite, or unreasonable).
 */
static bool value_invalid(float v)
{
    return (!isfinite(v) || v < -500.0f || v > 500.0f);
}

/* -------------------------------------------------------------------------- */
/*                                 PUBLIC API                                 */
/* -------------------------------------------------------------------------- */

/**
 * @brief Reset all tracking state, disable latched faults, and clear outputs.
 */
void fault_handler_init(void)
{
    fault_latch_bits = 0U;
    actuator_fault_mask = 0U;
    highest_fault = FAULT_HANDLER_FAULT_NONE;

    last_run_ms = 0U;
    motor_stall_counter = 0U;
    fan_fault_counter = 0U;
    compressor_active_seconds = 0U;
    compressor_bad_delta_seconds = 0U;
    overtemp_counter = 0U;

    set_front_rgb_fault(FAULT_OFF);
}

/**
 * @brief Perform a monitoring pass and latch faults based on telemetry.
 *
 * @param now_ms Current firmware tick in milliseconds.
 */
void fault_handler_run(uint32_t now_ms)
{
    if (!should_run_now(now_ms))
    {
        return;
    }

    /* Latest auger RPM reading reported by the motion sensors. */
    float auger_rpm = 0.0f;
    /* Latest auger motor current draw reported by telemetry. */
    float motor_current = 0.0f;
    /* Feedback RPM from fan channel one. */
    float fan1_rpm = 0.0f;
    /* Feedback RPM from fan channel two. */
    float fan2_rpm = 0.0f;
    /* Evaporator inlet temperature in degrees Celsius. */
    float evap_in_c = 0.0f;
    /* Evaporator outlet temperature in degrees Celsius. */
    float evap_out_c = 0.0f;
    /* Bowl temperature in degrees Celsius. */
    float bowl_temp_c = 0.0f;

    /* Aggregated success flag for all sensor reads in this cycle. */
    bool sensor_ok = true;
    sensor_ok &= (get_motor_current(&motor_current) == 0U);
    sensor_ok &= (get_auger_speed(&auger_rpm) == 0U);
    sensor_ok &= (get_fan_one_speed(&fan1_rpm) == 0U);
    sensor_ok &= (get_fan_two_speed(&fan2_rpm) == 0U);
    sensor_ok &= (get_evap_in_temp(&evap_in_c) == 0U);
    sensor_ok &= (get_evap_out_temp(&evap_out_c) == 0U);
    sensor_ok &= (get_bowl_temp(&bowl_temp_c) == 0U);

    if (!sensor_ok || value_invalid(motor_current) || value_invalid(auger_rpm) ||
        value_invalid(fan1_rpm) || value_invalid(fan2_rpm) ||
        value_invalid(evap_in_c) || value_invalid(evap_out_c) ||
        value_invalid(bowl_temp_c))
    {
        latch_sensor_fault();
        return;
    }

    /* Tachometer reading for the compressor in RPM. */
    uint16_t compressor_rpm = get_compressor_speed();

    /* -------- Motor stall detection -------- */
    if ((actuator_fault_mask & ACT_FAULT_MOTOR) == 0U)
    {
        bool stalled = (fabsf(auger_rpm) < AUGER_MIN_RUNNING_RPM) &&
                       (motor_current >= MOTOR_STALL_CURRENT_A);

        if (stalled)
        {
            if (motor_stall_counter < UINT8_MAX)
            {
                motor_stall_counter++;
            }
        }
        else
        {
            motor_stall_counter = 0U;
        }

        if (motor_stall_counter >= MOTOR_STALL_DEBOUNCE_COUNT)
        {
            latch_motor_fault();
        }
    }

    /* -------- Fan responsiveness -------- */
    /* True if the compressor feedback indicates an active state. */
    bool compressor_active = (compressor_rpm >= COMPRESSOR_MIN_ACTIVE_RPM);
    /* True if the auger is running fast enough to require active cooling. */
    bool motor_active = (fabsf(auger_rpm) >= FAN_REQUIRED_AUGER_RPM);
    /* True when any subsystem demands fan airflow. */
    bool fans_required = compressor_active || motor_active;

    if ((actuator_fault_mask & ACT_FAULT_FAN) == 0U)
    {
        if (fans_required &&
            ((fan1_rpm < FAN_MIN_RPM) || (fan2_rpm < FAN_MIN_RPM)))
        {
            if (fan_fault_counter < UINT8_MAX)
            {
                fan_fault_counter++;
            }
            if (fan_fault_counter >= FAN_FAULT_DEBOUNCE_COUNT)
            {
                latch_fan_fault();
            }
        }
        else
        {
            fan_fault_counter = 0U;
        }
    }

    /* -------- Compressor performance -------- */
    if ((actuator_fault_mask & ACT_FAULT_COMPRESSOR) == 0U)
    {
        if (compressor_active)
        {
            if (compressor_active_seconds < UINT32_MAX)
            {
                compressor_active_seconds++;
            }

            /* Calculated evaporator delta-T while the compressor is running. */
            float delta_c = evap_in_c - evap_out_c;
            if (delta_c < COMPRESSOR_MIN_DELTA_C)
            {
                if (compressor_bad_delta_seconds < UINT32_MAX)
                {
                    compressor_bad_delta_seconds++;
                }
            }
            else
            {
                compressor_bad_delta_seconds = 0U;
            }

            if (compressor_active_seconds >= COMPRESSOR_GRACE_SECONDS &&
                compressor_bad_delta_seconds >= COMPRESSOR_BAD_DELTA_SECONDS)
            {
                latch_compressor_fault();
            }
        }
        else
        {
            compressor_active_seconds = 0U;
            compressor_bad_delta_seconds = 0U;
        }
    }

    /* -------- Over-temperature -------- */
    if (!fault_is_set(FAULT_HANDLER_FAULT_OVERTEMP))
    {
        if (bowl_temp_c >= BOWL_OVERTEMP_C && compressor_active)
        {
            if (overtemp_counter < UINT8_MAX)
            {
                overtemp_counter++;
            }
            if (overtemp_counter >= OVERTEMP_DEBOUNCE_SECONDS)
            {
                latch_overtemp_fault();
            }
        }
        else if (overtemp_counter > 0U)
        {
            overtemp_counter--;
        }
    }
}

/**
 * @brief Manually report a fault and update derived state.
 *
 * @param fault Fault category to report.
 */
void fault_handler_report(fault_handler_fault_t fault)
{
    if (fault == FAULT_HANDLER_FAULT_NONE)
    {
        return;
    }

    fault_latch_bits |= fault_bit(fault);
    recompute_highest_priority();
    update_rgb_indicator();
}

/**
 * @brief Clear a specific latched fault and reset dependent counters.
 *
 * @param fault Fault category to clear.
 */
void fault_handler_clear(fault_handler_fault_t fault)
{
    if (fault == FAULT_HANDLER_FAULT_NONE)
    {
        return;
    }

    fault_latch_bits &= ~fault_bit(fault);

    if (fault == FAULT_HANDLER_FAULT_ACTUATOR)
    {
        actuator_fault_mask = 0U;
        motor_stall_counter = 0U;
        fan_fault_counter = 0U;
        compressor_active_seconds = 0U;
        compressor_bad_delta_seconds = 0U;
    }

    if (fault == FAULT_HANDLER_FAULT_OVERTEMP)
    {
        overtemp_counter = 0U;
    }

    recompute_highest_priority();
    update_rgb_indicator();
}

/**
 * @brief Clear all latched faults and restore the indicator to idle.
 */
void fault_handler_clear_all(void)
{
    fault_latch_bits = 0U;
    actuator_fault_mask = 0U;
    highest_fault = FAULT_HANDLER_FAULT_NONE;

    motor_stall_counter = 0U;
    fan_fault_counter = 0U;
    compressor_active_seconds = 0U;
    compressor_bad_delta_seconds = 0U;
    overtemp_counter = 0U;

    set_front_rgb_fault(FAULT_OFF);
}

/**
 * @brief Query whether the supplied fault category is currently latched.
 *
 * @param fault Fault category to inspect.
 * @return true if the fault is active, false otherwise.
 */
bool fault_handler_is_active(fault_handler_fault_t fault)
{
    if (fault == FAULT_HANDLER_FAULT_NONE)
    {
        return false;
    }
    return (fault_latch_bits & fault_bit(fault)) != 0U;
}

/**
 * @brief Return the highest priority fault currently latched.
 *
 * @return FAULT_HANDLER_FAULT_* entry, ordered by severity.
 */
fault_handler_fault_t fault_handler_get_highest_priority(void)
{
    return highest_fault;
}

#endif /* ENABLE_CONTROL_FSM */

