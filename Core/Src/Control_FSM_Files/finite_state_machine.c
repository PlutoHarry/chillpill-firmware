/*
 * finite_state_machine.c
 *
 *		Description:
 * High‑level finite state machine for the slush machine controller.
 *
 * This implementation coordinates the major operational states of the
 * machine: startup, pull‑down, steady state, de‑icing, shutdown,
 * service mode, fault handling and factory test.  It delegates
 * sensor processing to the estimator, actuation to the actuators
 * module, PID regulation to the pid_controller and user feedback to
 * the error handler, service and factory test modules.  The FSM
 * operates on a simple tick: call fsm_run_tick() periodically (e.g.
 * every 20 ms) with the current system tick in milliseconds.  Each
 * state has enter/run/exit handlers to encapsulate state‑specific
 * behaviour.
 *
 * 		Function:
 * 		-> Control state selection
 * 		-> Control the functionality of each state
 *
 * 		States:
 * 		->
 */

#include "finite_state_machine.h"
#include "estimator.h"
#include "actuators.h"
#include "buttons.h"
#include "sensors.h"
#include "pid_controller.h"
#include "control_config.h"
#include "error_handler.h"
#include "service.h"
#include "factory_test.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

/* Current state and entry timestamp */
static fsm_state_t current_state = STATE_STARTUP;
static uint32_t    state_entry_time = 0U;

/* Pull‑down control variables */
static float      current_max_freq        = 0.0f;
static uint32_t   last_icing_event_time   = 0U;
static uint32_t   last_deice_time         = 0U;
static uint32_t   shutdown_end_time       = 0U;
/* De‑icing timing */
static uint32_t   deice_start_time        = 0U;

/* Internal forward declarations */
static void state_startup_enter(uint32_t now);
static void state_startup_run(uint32_t now);
static void state_startup_exit(uint32_t now);

static void state_pull_down_enter(uint32_t now);
static void state_pull_down_run(uint32_t now);
static void state_pull_down_exit(uint32_t now);

static void state_steady_enter(uint32_t now);
static void state_steady_run(uint32_t now);
static void state_steady_exit(uint32_t now);

static void state_deicing_enter(uint32_t now);
static void state_deicing_run(uint32_t now);
static void state_deicing_exit(uint32_t now);

static void state_shutdown_enter(uint32_t now);
static void state_shutdown_run(uint32_t now);
static void state_shutdown_exit(uint32_t now);

static void state_fault_enter(uint32_t now);
static void state_fault_run(uint32_t now);
static void state_fault_exit(uint32_t now);

static void state_service_enter(uint32_t now);
static void state_service_run(uint32_t now);
static void state_service_exit(uint32_t now);

static void state_factory_test_enter(uint32_t now);
static void state_factory_test_run(uint32_t now);
static void state_factory_test_exit(uint32_t now);

static bool pull_down_cycle_complete(void);
static bool deice_cycle_complete(void);

/* Local helper to change state.  Calls exit on the old state,
 * updates the current_state and calls enter on the new state. */
static void fsm_transition(fsm_state_t new_state, uint32_t now)
{
    if (new_state == current_state) {
        return;
    }
    /* Exit old state */
    switch (current_state) {
        case STATE_STARTUP:        state_startup_exit(now);        break;
        case STATE_PULL_DOWN:      state_pull_down_exit(now);      break;
        case STATE_STEADY:         state_steady_exit(now);         break;
        case STATE_DEICING:        state_deicing_exit(now);        break;
        case STATE_SHUTDOWN:       state_shutdown_exit(now);       break;
        case STATE_FAULT:          state_fault_exit(now);          break;
        case STATE_SERVICE:        state_service_exit(now);        break;
        case STATE_FACTORY_TEST:   state_factory_test_exit(now);   break;
        default: break;
    }
    current_state = new_state;
    state_entry_time = now;
    /* Enter new state */
    switch (current_state) {
        case STATE_STARTUP:        state_startup_enter(now);        break;
        case STATE_PULL_DOWN:      state_pull_down_enter(now);      break;
        case STATE_STEADY:         state_steady_enter(now);         break;
        case STATE_DEICING:        state_deicing_enter(now);        break;
        case STATE_SHUTDOWN:       state_shutdown_enter(now);       break;
        case STATE_FAULT:          state_fault_enter(now);          break;
        case STATE_SERVICE:        state_service_enter(now);        break;
        case STATE_FACTORY_TEST:   state_factory_test_enter(now);   break;
        default: break;
    }
}

/* Public accessor */
fsm_state_t fsm_get_state(void)
{
    return current_state;
}

/* Initialise the FSM.  Call this once after hardware init. */
void fsm_init(void)
{
    uint32_t now = HAL_GetTick();
    current_state = STATE_STARTUP;
    state_entry_time = now;
    /* Initialise subsystems */
    estimator_init();
    control_init();
    pid_controller_init();
    error_handler_init();
    service_init();
    factory_test_init();
    /* Pull‑down limits */
    current_max_freq      = g_control_config.pull_down_max_freq_initial;
    last_icing_event_time = 0U;
    last_deice_time       = now;
    shutdown_end_time     = 0U;
    deice_start_time      = 0U;
    /* Enter initial state */
    state_startup_enter(now);
    /* Immediately enter factory test if enabled */
    if (g_control_config.factory_test_enable) {
        /* Start test and switch state */
        factory_test_start(now);
        current_state = STATE_FACTORY_TEST;
        state_entry_time = now;
    }
}

/* Main tick.  Call this at a fixed rate. */
void fsm_run_tick(uint32_t now)
{
    /* Update estimator regardless of state */
    estimator_update();
    /* LED pulse and ready indicator tasks */
    control_led_pulse_task();
    control_ready_indicator_task();
    /* Update motor runtime for service */
    control_update_runtime(now);

    /* Factory test overrides all other behaviour */
    if (current_state == STATE_FACTORY_TEST) {
        state_factory_test_run(now);
        return;
    }

    /* Check for service mode entry by holding light and freeze buttons */
    static uint32_t service_hold_start = 0U;
    if (current_state != STATE_SERVICE) {
        GPIO_PinState light_pressed  = HAL_GPIO_ReadPin(LIGHT_BTN_EXT11_GPIO_Port, LIGHT_BTN_EXT11_Pin);
        GPIO_PinState freeze_pressed = HAL_GPIO_ReadPin(FREEZ_BTN_EXT10_GPIO_Port, FREEZ_BTN_EXT10_Pin);
        if (light_pressed == GPIO_PIN_SET && freeze_pressed == GPIO_PIN_SET) {
            if (service_hold_start == 0U) {
                service_hold_start = now;
            } else if ((now - service_hold_start) >= g_control_config.service_entry_hold_ms) {
                fsm_transition(STATE_SERVICE, now);
                service_hold_start = 0U;
                return;
            }
        } else {
            service_hold_start = 0U;
        }
    }

    /* Check for new fault conditions.  If a new fault is detected,
     * transition to FAULT. */
    if (error_handler_check_and_set()) {
        fsm_transition(STATE_FAULT, now);
    }
    /* Update fault LED pattern */
    error_handler_update(now);

    /* Power button off triggers shutdown */
    extern bool btn_power_status;
    if (!btn_power_status && current_state != STATE_SHUTDOWN) {
        fsm_transition(STATE_SHUTDOWN, now);
    }

    /* Delegate to state run handler */
    switch (current_state) {
        case STATE_STARTUP:      state_startup_run(now);      break;
        case STATE_PULL_DOWN:    state_pull_down_run(now);    break;
        case STATE_STEADY:       state_steady_run(now);       break;
        case STATE_DEICING:      state_deicing_run(now);      break;
        case STATE_SHUTDOWN:     state_shutdown_run(now);     break;
        case STATE_FAULT:        state_fault_run(now);        break;
        case STATE_SERVICE:      state_service_run(now);      break;
        default: break;
    }
}

/* ------------------ State: STARTUP ------------------ */
static void state_startup_enter(uint32_t now)
{
    /* At startup, ensure all actuators are off and LEDs reset. */
    control_stop_compressor();
    control_stop_motor();
    control_stop_fan();
    control_set_led_pulse(false);
    control_set_led_brightness(0.0f);
    freeze_rgb_led_off();
}

static void state_startup_run(uint32_t now)
{
    /* Stay in STARTUP for a short period (e.g. 1 s) to allow
     * sensors to stabilise, then transition into pull‑down. */
    if ((now - state_entry_time) >= 1000U) {
        fsm_transition(STATE_PULL_DOWN, now);
    }
}

static void state_startup_exit(uint32_t now)
{
    /* Nothing to clean up on exit */
    (void)now;
}

/* ------------------ State: PULL_DOWN ------------------ */
static void state_pull_down_enter(uint32_t now)
{
    /* Disable steady‑state PID regulation during pull‑down */
    pid_controller_enable(false);
    /* Determine mode (cold drink vs slush) */
    freeze_temp_set_t mode = pid_controller_get_current_mode();
    if (mode == FREEZE_TEMP_PLUS4) {
        current_max_freq = g_control_config.cold_pull_down_max_freq;
    } else {
        current_max_freq = g_control_config.pull_down_max_freq_initial;
    }
    /* Command compressor toward the maximum frequency */
    control_set_compressor_frequency((uint8_t)current_max_freq);
    /* Set auger speed: cold mode uses cold_pull_down_motor_speed; slush uses deice_motor_speed */
    float motor_speed = (mode == FREEZE_TEMP_PLUS4) ? g_control_config.cold_pull_down_motor_speed : g_control_config.deice_motor_speed;
    control_set_motor_speed(motor_speed);
    /* Run fans at full speed during pull‑down */
    control_set_fan_speed(100);
    /* Reset icing timer */
    last_icing_event_time = 0U;
}

static void state_pull_down_run(uint32_t now)
{
    /* Limit compressor frequency changes gradually via inverter stepper */
    control_set_compressor_frequency((uint8_t)current_max_freq);
    /* Check for icing.  If icing condition is met, reduce max frequency
     * and transition into de‑icing. */
    if (estimator_needs_deicing()) {
        /* Reduce max frequency by step */
        if (current_max_freq > g_control_config.pull_down_min_freq) {
            current_max_freq -= g_control_config.pull_down_freq_step;
            if (current_max_freq < g_control_config.pull_down_min_freq) {
                current_max_freq = g_control_config.pull_down_min_freq;
            }
        }
        last_icing_event_time = now;
        fsm_transition(STATE_DEICING, now);
        return;
    }
    /* If pull‑down is complete, enter steady state */
    if (pull_down_cycle_complete()) {
        fsm_transition(STATE_STEADY, now);
    }
}

static void state_pull_down_exit(uint32_t now)
{
    /* Nothing to do */
    (void)now;
}

/* ------------------ State: STEADY ------------------ */
static void state_steady_enter(uint32_t now)
{
    /* Enable PID regulation */
    pid_controller_enable(true);
    /* Start the ready indicator */
    control_start_ready_indicator();
    /* Update last de‑ice time */
    last_deice_time = now;
    /* Set auger speed depending on mode */
    freeze_temp_set_t mode = pid_controller_get_current_mode();
    float motor_speed = (mode == FREEZE_TEMP_PLUS4) ? g_control_config.cold_pull_down_motor_speed : g_control_config.deice_motor_speed;
    control_set_motor_speed(motor_speed);
}

static void state_steady_run(uint32_t now)
{
    /* Perform PID update */
    float bowl_temp    = estimator_get_real_bowl_temp();
    float texture_idx  = estimator_get_texture_index();
    pid_controller_update(bowl_temp, texture_idx);
    /* Apply target frequency */
    uint8_t target_freq = pid_controller_get_target_frequency();
    control_set_compressor_frequency(target_freq);
    /* Adjust motor speed for current mode */
    freeze_temp_set_t mode = pid_controller_get_current_mode();
    float motor_speed = (mode == FREEZE_TEMP_PLUS4) ? g_control_config.cold_pull_down_motor_speed : g_control_config.deice_motor_speed;
    control_set_motor_speed(motor_speed);
    /* Check if periodic de‑icing is due */
    if ((now - last_deice_time) >= g_control_config.periodic_deice_interval_ms) {
        fsm_transition(STATE_DEICING, now);
        return;
    }
    /* Check for icing in steady state */
    if (estimator_needs_deicing()) {
        fsm_transition(STATE_DEICING, now);
        return;
    }
}

static void state_steady_exit(uint32_t now)
{
    /* Disable PID temporarily on exit */
    pid_controller_enable(false);
    (void)now;
}

/* ------------------ State: DEICING ------------------ */
static void state_deicing_enter(uint32_t now)
{
    /* Disable PID */
    pid_controller_enable(false);
    /* Stop compressor */
    control_stop_compressor();
    /* Run auger slowly to clear ice */
    control_set_motor_speed(g_control_config.deice_motor_speed);
    /* Run fans at full speed */
    control_set_fan_speed(100);
    /* Mark de‑ice start */
    deice_start_time = now;
}

static void state_deicing_run(uint32_t now)
{
    /* Allow de‑icing to continue until the estimator signals clear or
     * a maximum time has elapsed.  A minimum dwell of 10 s is
     * enforced to ensure ice melts sufficiently. */
    const uint32_t MIN_DEICE_MS  = 10000U;
    const uint32_t MAX_DEICE_MS  = 180000U; /* 3 minutes max */
    uint32_t elapsed = now - deice_start_time;
    if (elapsed >= MIN_DEICE_MS) {
        if (deice_cycle_complete() || elapsed >= MAX_DEICE_MS) {
            /* Recalibrate torque baseline */
            update_slush_torque_reference();
            /* Update last de‑ice time */
            last_deice_time = now;
            /* Resume PID */
            pid_controller_enable(true);
            /* Transition back to steady state */
            fsm_transition(STATE_STEADY, now);
            return;
        }
    }
}

static void state_deicing_exit(uint32_t now)
{
    (void)now;
    /* Nothing else to do.  PID is re‑enabled in run handler. */
}

/* ------------------ State: SHUTDOWN ------------------ */
static void state_shutdown_enter(uint32_t now)
{
    /* Stop compressor and motor */
    control_stop_compressor();
    control_stop_motor();
    /* Determine if condenser is hot: use estimated load or bowl temperature */
    float condenser_load = estimator_get_condenser_load();
    float bowl  = estimator_get_real_bowl_temp();
    bool hot = (condenser_load >= 0.95f) ||
               (bowl  > g_control_config.condenser_hot_temp_threshold);
    if (hot) {
        shutdown_end_time = now + g_control_config.shutdown_fan_runtime_ms;
    } else {
        shutdown_end_time = now;
    }
    /* Keep fans running */
    control_set_fan_speed(100);
}

static void state_shutdown_run(uint32_t now)
{
    if (now >= shutdown_end_time) {
        control_stop_fan();
    }
}

static void state_shutdown_exit(uint32_t now)
{
    (void)now;
    /* Ensure fans are off */
    control_stop_fan();
}

/* ------------------ State: FAULT ------------------ */
static void state_fault_enter(uint32_t now)
{
    (void)now;
    /* Stop all actuators */
    control_stop_compressor();
    control_stop_motor();
    control_stop_fan();
    /* Ensure breathing and ready indicator are off */
    control_set_led_pulse(false);
}

static void state_fault_run(uint32_t now)
{
    (void)now;
    /* The error handler updates the RGB LED pattern */
    /* Remain in fault until reset or power cycle */
}

static void state_fault_exit(uint32_t now)
{
    (void)now;
    /* Clear fault and stop LED flashing */
    error_handler_clear_error();
}

/* ------------------ State: SERVICE ------------------ */
static void state_service_enter(uint32_t now)
{
    (void)now;
    /* Enter service mode: service module stops actuators and sets LED */
    service_enter(now);
}

static void state_service_run(uint32_t now)
{
    service_update(now);
}

static void state_service_exit(uint32_t now)
{
    (void)now;
    service_exit();
}

/* ------------------ State: FACTORY_TEST ------------------ */
static void state_factory_test_enter(uint32_t now)
{
    (void)now;
    /* Start the factory test if not already running */
    factory_test_start(now);
    /* Ensure breathing and ring LED are off */
    control_set_led_pulse(false);
    control_set_led_brightness(0.0f);
}

static void state_factory_test_run(uint32_t now)
{
    /* Delegate to factory test module */
    factory_test_run(now);
    /* If the test is finished, automatically shut down */
    if (factory_test_is_finished()) {
        /* Indicate pass/fail via LED: factory_test module handles LED colour */
        /* Prevent further transitions */
    }
}

static void state_factory_test_exit(uint32_t now)
{
    (void)now;
    /* Nothing to clean up explicitly; test module resets LED */
}

static bool pull_down_cycle_complete(void)
{
    freeze_temp_set_t mode = pid_controller_get_current_mode();
    float bowl_temp  = estimator_get_real_bowl_temp();
    float texture    = estimator_get_texture_index();

    float temp_target = g_control_config.freeze_temp_medium_setpoint;
    float texture_target = g_control_config.texture_target_medium;

    if (mode == FREEZE_TEMP_PLUS4) {
        temp_target = g_control_config.cold_drink_temp_setpoint;
        texture_target = 0.0f;
    }
#ifdef FREEZE_TEMP_LIGHT
    else if (mode == FREEZE_TEMP_LIGHT) {
        temp_target = g_control_config.freeze_temp_light_setpoint;
        texture_target = g_control_config.texture_target_light;
    }
#endif
#ifdef FREEZE_TEMP_HEAVY
    else if (mode == FREEZE_TEMP_HEAVY) {
        temp_target = g_control_config.freeze_temp_heavy_setpoint;
        texture_target = g_control_config.texture_target_heavy;
    }
#endif
#ifdef FREEZE_TEMP_MEDIUM
    else if (mode == FREEZE_TEMP_MEDIUM) {
        temp_target = g_control_config.freeze_temp_medium_setpoint;
        texture_target = g_control_config.texture_target_medium;
    }
#endif

    const float TEMP_MARGIN = 0.4f;
    const float TEXTURE_MARGIN = 0.08f;

    bool temp_ready = (bowl_temp <= temp_target + TEMP_MARGIN);
    bool texture_ready = (mode == FREEZE_TEMP_PLUS4) ? true
                        : (texture >= fmaxf(0.0f, texture_target - TEXTURE_MARGIN));

    return temp_ready && texture_ready;
}

static bool deice_cycle_complete(void)
{
    if (estimator_needs_deicing()) {
        return false;
    }
    return estimator_get_ice_index() < 0.35f;
}
