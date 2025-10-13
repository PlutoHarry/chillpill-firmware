/**
 * @file finite_state_machine.c
 *
 * @section Purpose
 * This translation unit implements the high-level finite state machine (FSM)
 * responsible for orchestrating the ChillPill refrigeration controller.  The
 * FSM owns all sequencing logic required to safely progress through
 * power-on, pull-down, steady-state cooling, de-icing, shutdown, fault
 * handling, and service/factory diagnostics modes.
 *
 * @section Public API Overview
 * - ::control_fsm_init(): Initialise the FSM, dependent subsystems and enter
 *   the initial state.  Call once at boot before using any other API.
 * - ::control_fsm_reset(): Reinitialise the FSM to its defaults.  Useful when
 *   leaving debug mode or after critical configuration changes.
 * - ::control_fsm_run_tick(): Execute a single scheduler tick.  Invoke at a
 *   fixed cadence (e.g. from the main loop) with the current timestamp in
 *   milliseconds to advance state transitions and control loops.
 * - ::control_fsm_set_target_temperature(): Update the user-requested freeze
 *   temperature that informs steady-state control decisions.
 * - ::control_fsm_request_state(): Queue a state change to be applied in the
 *   next tick; the transition is deferred to maintain clean entry/exit hooks.
 * - ::control_fsm_get_state(): Read the currently active FSM state.
 * - ::control_fsm_is_transition_pending(): Determine whether a queued state
 *   change is waiting to be processed.
 * - ::control_fsm_set_debug_mode() / ::control_fsm_is_debug_mode(): Enable or
 *   query the special manual-debug loop used during servicing.
 * - ::fsm_request_service_mode() / ::fsm_request_factory_mode(): Lightweight
 *   hooks triggered by gesture handlers to request special diagnostic states.
 *
 * @section Usage Notes
 * - All public functions are thread-context agnostic but assume cooperative
 *   scheduling: invoke ::control_fsm_run_tick() frequently to keep state
 *   transitions responsive.
 * - The FSM interacts with PID, estimator, actuator, and fault subsystems.
 *   Those modules must remain initialised for the FSM to operate correctly.
 * - Debug mode intentionally bypasses closed-loop PID control so the
 *   ::control_fsm_run_tick() helper detects the mode and short-circuits into a
 *   manual control loop.
 *
 * @section Additional Details
 * - Timekeeping uses the HAL tick counter (milliseconds) and stores per-state
 *   entry timestamps to evaluate duration-based transitions.
 * - Each state has explicit entry and exit helpers to guarantee actuators are
 *   configured deterministically when entering or leaving the state.
 * - Console logging is relied upon heavily during development to trace state
 *   transitions and debug behaviour; comment coverage emphasises the key
 *   decision points to aid future modifications.
 */

#include "finite_state_machine.h"

#include "actuators.h"
#include "control_config.h"
#include "estimator.h"
#include "factory_test.h"
#include "fault_handler.h"
#include "lights.h"
#include "pid_controller.h"
#include "user_settings.h"

#include "stm32f1xx_hal.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#if ENABLE_CONTROL_FSM

/** Delay after start-up before leaving CONTROL_FSM_STATE_STARTUP (ms). */
#define STARTUP_SETTLE_MS          1000U
/** Maximum time allowed in pull-down before forcing steady state (ms). */
#define PULL_DOWN_MAX_MS         300000U
/** Minimum time to remain in the dedicated de-icing state (ms). */
#define DEICE_MIN_DWELL_MS        20000U
/** Period between debug summary prints (ms). */
#define DEBUG_PRINT_PERIOD_MS      1000U

/** Helper converting compressor electrical frequency (Hz) to RPM. */
static inline uint16_t freq_hz_to_rpm(float freq_hz)
{
    if (!isfinite(freq_hz) || freq_hz <= 0.0f) {
        return 0U;
    }
    float rpm = freq_hz * 60.0f;
    if (rpm < 0.0f) {
        rpm = 0.0f;
    }
    if (rpm > 6000.0f) {
        rpm = 6000.0f;
    }
    return (uint16_t)lrintf(rpm);
}

/** Map freeze-mode selection to PID controller mode. */
static pid_controller_mode_t freeze_mode_to_pid_mode(uint8_t freeze_mode)
{
    switch (freeze_mode) {
        case 0: return PID_CONTROLLER_MODE_COLD_DRINK;
        case 1: return PID_CONTROLLER_MODE_COLD_DRINK;
        case 2: return PID_CONTROLLER_MODE_LIGHT_SLUSH;
        case 3: return PID_CONTROLLER_MODE_MEDIUM_SLUSH;
        case 4: return PID_CONTROLLER_MODE_HEAVY_SLUSH;
        default: break;
    }
    return PID_CONTROLLER_MODE_MEDIUM_SLUSH;
}

/**
 * @brief Aggregate of all mutable FSM state.
 *
 * The struct stores both the externally-visible state as well as bookkeeping
 * data that simplifies transition decisions (timestamps, cached modes, etc.).
 */
typedef struct
{
    control_fsm_state_t state;                /**< Currently active FSM state. */
    control_fsm_state_t requested_state;      /**< Queued target state. */
    uint32_t            entry_ms;             /**< Timestamp when the current
                                               *   state was entered. */
    uint32_t            last_debug_print_ms;  /**< Last debug log emission time. */
    uint32_t            pull_down_start_ms;   /**< Start time for pull-down
                                               *   monitoring. */
    uint32_t            deice_start_ms;       /**< Start time for de-icing
                                               *   dwell tracking. */
    int32_t             target_temp_c;        /**< Requested bowl temperature in
                                               *   degrees Celsius. */
    bool                transition_pending;   /**< True when a state change is
                                               *   scheduled for the next tick. */
    bool                debug_mode;           /**< Enables the manual debug loop. */
    bool                ready_indicator_active;/**< Cached front-ring indicator. */
    uint8_t             last_debug_freeze_mode;/**< Remembered debug palette step
                                                *   used to suppress log spam. */
    pid_controller_mode_t current_pid_mode;   /**< Active PID tuning profile. */
} control_fsm_context_t;

static control_fsm_context_t s_fsm = {
    .state                     = CONTROL_FSM_STATE_STARTUP,
    .requested_state           = CONTROL_FSM_STATE_STARTUP,
    .entry_ms                  = 0U,
    .last_debug_print_ms       = 0U,
    .pull_down_start_ms        = 0U,
    .deice_start_ms            = 0U,
    .target_temp_c             = 0,
    .transition_pending        = false,
    .debug_mode                = (DEFAULT_DEBUG_MODE != 0),
    .ready_indicator_active    = false,
    .last_debug_freeze_mode    = 0xFFU,
    .current_pid_mode          = PID_CONTROLLER_MODE_MEDIUM_SLUSH,
};

/* Forward declarations for state handlers */
static void fsm_change_state(control_fsm_state_t new_state, uint32_t now_ms);
static void fsm_enter_state(control_fsm_state_t state, uint32_t now_ms);
static void fsm_exit_state(control_fsm_state_t state, uint32_t now_ms);
static void fsm_run_debug_loop(uint32_t now_ms);
static bool pull_down_complete(void);
static bool deice_complete(uint32_t now_ms);
static void update_ready_indicator(bool enable);
static void log_state_transition(control_fsm_state_t from,
                                 control_fsm_state_t to,
                                 uint32_t            now_ms);

/* ------------------------------------------------------------------------- */
/*                           Public API implementation                       */
/* ------------------------------------------------------------------------- */

void control_fsm_init(void)
{
    /* Reset every field so we start from a predictable baseline. */
    memset(&s_fsm, 0, sizeof(s_fsm));
    s_fsm.state            = CONTROL_FSM_STATE_STARTUP;   /* Initial public state. */
    s_fsm.requested_state  = CONTROL_FSM_STATE_STARTUP;   /* No queued change yet. */
    s_fsm.entry_ms         = HAL_GetTick();               /* Capture boot timestamp. */
    s_fsm.debug_mode       = (DEFAULT_DEBUG_MODE != 0);   /* Honour build-time flag. */
    s_fsm.current_pid_mode = PID_CONTROLLER_MODE_MEDIUM_SLUSH; /* Default PID tuning. */

    /* Initialise all dependent subsystems before we drive any actuators. */
    estimator_init();
    pid_controller_init();
    pid_controller_set_mode(s_fsm.current_pid_mode);
    fault_handler_init();
    factory_test_init();

    printf("FSM: init complete (debug mode %s)\n",
           s_fsm.debug_mode ? "enabled" : "disabled");

    /* Execute the start-up entry hook to configure outputs immediately. */
    fsm_enter_state(s_fsm.state, s_fsm.entry_ms);
}

void control_fsm_reset(void)
{
    /* Re-use the init logic so the entire subsystem returns to a pristine state. */
    printf("FSM: reset requested\n");
    control_fsm_init();
}

void control_fsm_run_tick(uint32_t now_ms)
{
    /* Refresh estimator inputs so subsequent control logic operates on new data. */
    estimator_update();

    /* When debug mode is active the specialised loop owns control entirely. */
    if (s_fsm.debug_mode) {
        fsm_run_debug_loop(now_ms);
        return;
    }

    /* Apply any queued transition before evaluating current-state behaviour. */
    if (s_fsm.transition_pending) {
        fsm_change_state(s_fsm.requested_state, now_ms);
        s_fsm.transition_pending = false;
    }

    /* Allow the fault handler to update its state and immediately honour faults. */
    fault_handler_run(now_ms);
    if (fault_handler_get_highest_priority() != FAULT_HANDLER_FAULT_NONE &&
        s_fsm.state != CONTROL_FSM_STATE_FAULT) {
        printf("FSM: fault detected -> FAULT state\n");
        fsm_change_state(CONTROL_FSM_STATE_FAULT, now_ms);
    }

    /* Execute behaviour specific to the currently active FSM state. */
    switch (s_fsm.state) {
        case CONTROL_FSM_STATE_STARTUP:
            /* After the hardware has had time to stabilise, proceed to pull-down. */
            if ((now_ms - s_fsm.entry_ms) >= STARTUP_SETTLE_MS) {
                fsm_change_state(CONTROL_FSM_STATE_PULL_DOWN, now_ms);
            }
            break;
        case CONTROL_FSM_STATE_PULL_DOWN:
            /* Evaluate whether the bowl has reached temperature/texture targets. */
            if (pull_down_complete()) {
                printf("FSM: pull-down complete -> STEADY\n");
                fsm_change_state(CONTROL_FSM_STATE_STEADY, now_ms);
            } else if ((now_ms - s_fsm.pull_down_start_ms) >= PULL_DOWN_MAX_MS) {
                /* Timeout guard to avoid running indefinitely if sensors misbehave. */
                printf("FSM: pull-down timeout -> STEADY\n");
                fsm_change_state(CONTROL_FSM_STATE_STEADY, now_ms);
            } else if (estimator_needs_deicing()) {
                /* Ice load was detected before reaching steady state. */
                printf("FSM: icing detected during pull-down\n");
                fsm_change_state(CONTROL_FSM_STATE_DEICING, now_ms);
            }
            break;
        case CONTROL_FSM_STATE_STEADY: {
            /* Continuously tune the PID based on the user's freeze-mode selection. */
            uint8_t freeze_mode = user_settings_get_effective_freeze_mode();
            pid_controller_mode_t desired_mode = freeze_mode_to_pid_mode(freeze_mode);
            if (desired_mode != s_fsm.current_pid_mode) {
                pid_controller_set_mode(desired_mode);
                s_fsm.current_pid_mode = desired_mode;
                printf("FSM: PID mode -> %u\n", (unsigned)desired_mode);
            }

            /* Feed the latest process variables into the PID controller. */
            pid_controller_update_state(estimator_get_real_bowl_temp(),
                                        estimator_get_texture_index(),
                                        estimator_get_volume());
            pid_controller_run(now_ms);  /* Compute new actuator set-points. */
            pid_controller_output_t out = pid_controller_get_output_struct();

            set_compressor_speed(freq_hz_to_rpm(out.compressor_frequency_hz));
            set_motor_speed(out.motor_rpm);
            if (estimator_needs_deicing()) {
                printf("FSM: icing detected -> DEICING\n");
                fsm_change_state(CONTROL_FSM_STATE_DEICING, now_ms);
            }
            break;
        }
        case CONTROL_FSM_STATE_DEICING:
            /* Wait until both the minimum dwell time and estimator agree ice is gone. */
            if (deice_complete(now_ms)) {
                printf("FSM: de-icing complete\n");
                update_slush_torque_reference();
                fsm_change_state(CONTROL_FSM_STATE_STEADY, now_ms);
            }
            break;
        case CONTROL_FSM_STATE_SHUTDOWN:
            /* Keep the fan running briefly to evacuate residual cold air. */
            if (now_ms - s_fsm.entry_ms >= g_control_config.shutdown_fan_runtime_ms) {
                set_fan_speed(0);
            }
            break;
        case CONTROL_FSM_STATE_SERVICE:
            /* Service visuals handled in entry helper; nothing to do per tick. */
            break;
        case CONTROL_FSM_STATE_FACTORY_TEST:
            factory_test_run(now_ms);
            if (factory_test_is_complete()) {
                printf("FSM: factory test completed\n");
            }
            break;
        case CONTROL_FSM_STATE_FAULT:
        case CONTROL_FSM_STATE_POWER_ON:
        case CONTROL_FSM_STATE_DEBUG:
        default:
            break;
    }
}

void control_fsm_set_target_temperature(int32_t target_c)
{
    /* Cache the new target and log it for operator visibility. */
    s_fsm.target_temp_c = target_c;
    printf("FSM: new target temperature %ld C\n", (long)target_c);
}

void control_fsm_request_state(control_fsm_state_t state)
{
    /* Schedule the state change for the next tick to respect exit/enter hooks. */
    s_fsm.requested_state    = state;
    s_fsm.transition_pending = true;
    printf("FSM: transition requested -> %d\n", (int)state);
}

control_fsm_state_t control_fsm_get_state(void)
{
    /* Expose the public view of the FSM for diagnostics or UI updates. */
    return s_fsm.state;
}

bool control_fsm_is_transition_pending(void)
{
    /* Useful for tests to confirm whether a queued request is still outstanding. */
    return s_fsm.transition_pending;
}

void control_fsm_set_debug_mode(bool enabled)
{
    /* Ignore redundant requests so we do not reset diagnostics unnecessarily. */
    if (s_fsm.debug_mode == enabled) {
        return;
    }
    s_fsm.debug_mode = enabled;
    printf("FSM: debug mode %s\n", enabled ? "enabled" : "disabled");
    if (enabled) {
        /* Entering debug mode clears faults and transitions to manual control. */
        fault_handler_clear_all();
        fsm_change_state(CONTROL_FSM_STATE_DEBUG, HAL_GetTick());
    } else {
        /* Leaving debug mode should restore a clean, freshly initialised FSM. */
        control_fsm_reset();
    }
}

bool control_fsm_is_debug_mode(void)
{
    /* Allows UI layers to reflect when the machine is in technician control. */
    return s_fsm.debug_mode;
}

/* ------------------------------------------------------------------------- */
/*                            Debug helper implementation                     */
/* ------------------------------------------------------------------------- */

static const struct {
    freeze_btn_color button;
    rgb_color_t      rgb;
    const char      *label;
} g_freeze_palette[5] = {
    { FREEZE_BTN_OFF,       RGB_OFF,       "OFF" },
    { FREEZE_BTN_GREEN,     RGB_GREEN,     "GREEN" },
    { FREEZE_BTN_WHITE,     RGB_WHITE,     "WHITE" },
    { FREEZE_BTN_LIGHTBLUE, RGB_LIGHTBLUE, "LIGHT BLUE" },
    { FREEZE_BTN_BLUE,      RGB_BLUE,      "BLUE" },
};

/**
 * @brief Convert the user-facing freeze mode into a compressor frequency.
 *
 * The mapping implements the stepped profile requested for debug mode using
 * the min/max limits from the medium slush control profile.  Returning Hz here
 * keeps the helper reusable for other call sites that may prefer raw
 * frequency rather than RPM.
 */
static float debug_mode_frequency(uint8_t freeze_mode)
{
    const control_mode_profile_t *profile =
        control_config_get_profile(CONTROL_MODE_MEDIUM_SLUSH);
    float min_freq = profile->compressor_min_freq_hz;
    float max_freq = profile->compressor_max_freq_hz;
    float span = max_freq - min_freq;
    if (span < 0.0f) {
        span = 0.0f;
    }

    switch (freeze_mode) {
        case 0: return 0.0f;                          /* OFF   -> stopped compressor */
        case 1: return min_freq;                      /* GREEN -> minimum frequency  */
        case 2: return min_freq + (span / 3.0f);      /* WHITE -> 1/3 span step      */
        case 3: return min_freq + (2.0f * span / 3.0f);/* LBL  -> 2/3 span step      */
        case 4: return max_freq;                      /* DBL   -> maximum frequency  */
        default: return 0.0f;
    }
}

/**
 * @brief Execute the manual-debug operating loop.
 *
 * The loop mirrors the runtime behaviour of the production FSM but swaps out
 * closed-loop control for deterministic set-points so that technicians can
 * exercise the hardware safely.  Each iteration performs the following steps:
 * 1. Ensure the reported FSM state reflects the debug operating mode.
 * 2. Read the freeze mode selection and derive compressor/auger targets.
 * 3. Apply the targets to the compressor, auger motor and condenser fan.
 * 4. Update the user feedback (button colour and ring LED preview).
 * 5. Log mode transitions and emit the once-per-second estimator snapshot.
 */
static void fsm_run_debug_loop(uint32_t now_ms)
{
    /* Make sure the public state machine reflects that we are in debug mode. */
    if (s_fsm.state != CONTROL_FSM_STATE_DEBUG) {
        fsm_change_state(CONTROL_FSM_STATE_DEBUG, now_ms);
    }

    /* Clamp the freeze mode coming from the user interface to the valid range. */
    uint8_t freeze_mode = user_settings_get_effective_freeze_mode();
    if (freeze_mode > 4U) {
        freeze_mode = 0U;
    }

    /* Translate the mode into compressor speed targets and derived RPM. */
    float compressor_freq = debug_mode_frequency(freeze_mode);
    uint16_t compressor_rpm = freq_hz_to_rpm(compressor_freq);

    /* Apply the chosen debug speed targets.  Mode 0 keeps everything stopped. */
    if (compressor_rpm == 0U) {
        set_compressor_speed(0U);
        set_motor_speed(0.0f);
        set_fan_speed(0U);
    } else {
        set_compressor_speed(compressor_rpm);
        set_motor_speed(25.0f);
        set_fan_speed(100U);
    }

    /* Keep the button ring synchronised with the colour associated to the step. */
    freeze_btn_color btn = g_freeze_palette[freeze_mode].button;
    rgb_color_t rgb = g_freeze_palette[freeze_mode].rgb;
    set_freeze_btn_color(btn);
    set_front_rgb_preview_selection_10s(rgb);

    /* Only log when the selection actually changes to avoid flooding output. */
    if (freeze_mode != s_fsm.last_debug_freeze_mode) {
        printf("FSM: debug freeze mode -> %s\n", g_freeze_palette[freeze_mode].label);
        s_fsm.last_debug_freeze_mode = freeze_mode;
    }

    /* Emit the estimator's condensed snapshot roughly once per second. */
    if ((now_ms - s_fsm.last_debug_print_ms) >= DEBUG_PRINT_PERIOD_MS) {
        estimator_print_debug_data(now_ms);
        s_fsm.last_debug_print_ms = now_ms;
    }
}

/* ------------------------------------------------------------------------- */
/*                               State helpers                                */
/* ------------------------------------------------------------------------- */

static void fsm_change_state(control_fsm_state_t new_state, uint32_t now_ms)
{
    if (new_state == s_fsm.state) {
        return;
    }
    /* Print the transition, call the exit hook, swap state and then enter. */
    log_state_transition(s_fsm.state, new_state, now_ms);
    fsm_exit_state(s_fsm.state, now_ms);
    s_fsm.state    = new_state;
    s_fsm.entry_ms = now_ms;
    fsm_enter_state(new_state, now_ms);
}

static void fsm_enter_state(control_fsm_state_t state, uint32_t now_ms)
{
    switch (state) {
        case CONTROL_FSM_STATE_STARTUP:
            /* Ensure every actuator starts idle during the power-on settling period. */
            set_compressor_speed(0U);
            set_motor_speed(0.0f);
            set_fan_speed(0U);
            update_ready_indicator(false);
            break;
        case CONTROL_FSM_STATE_PULL_DOWN: {
            const control_mode_profile_t *profile =
                control_config_get_profile(CONTROL_MODE_MEDIUM_SLUSH);
            s_fsm.pull_down_start_ms = now_ms; /* Remember when pull-down began. */
            set_compressor_speed(freq_hz_to_rpm(g_control_config.pull_down_max_freq_initial));
            set_motor_speed(profile->auger_rpm); /* Aggressive auger speed for quick freeze. */
            set_fan_speed(100U);                  /* Full fan to remove condenser heat. */
            update_ready_indicator(false);        /* Not ready for dispensing yet. */
            break;
        }
        case CONTROL_FSM_STATE_STEADY:
            /* Ready indicator tells users product can be dispensed. */
            update_ready_indicator(true);
            break;
        case CONTROL_FSM_STATE_DEICING:
            s_fsm.deice_start_ms = now_ms;          /* Start dwell timer. */
            set_compressor_speed(0U);               /* Stop cooling to allow thawing. */
            set_motor_speed(g_control_config.deice_motor_speed); /* Slow churn to break ice. */
            set_fan_speed(100U);                    /* Keep air moving through condenser. */
            update_ready_indicator(false);
            break;
        case CONTROL_FSM_STATE_SHUTDOWN:
            set_compressor_speed(0U);
            set_motor_speed(0.0f);
            set_fan_speed(100U);
            update_ready_indicator(false);
            break;
        case CONTROL_FSM_STATE_FAULT:
            set_compressor_speed(0U);    /* Remove power from all actuators. */
            set_motor_speed(0.0f);
            set_fan_speed(0U);
            update_ready_indicator(false);
            set_front_rgb_fault(FAULT_COMPRESSOR); /* Solid red ring communicates failure. */
            break;
        case CONTROL_FSM_STATE_FACTORY_TEST:
            factory_test_request_start(); /* Kick off scripted diagnostics. */
            break;
        case CONTROL_FSM_STATE_SERVICE:
            lights_display_motor_hours_begin(); /* Show cumulative runtime to user. */
            break;
        case CONTROL_FSM_STATE_DEBUG:
            fault_handler_clear_all();   /* Clear latched faults before manual control. */
            update_ready_indicator(false);
            break;
        case CONTROL_FSM_STATE_POWER_ON:
        default:
            break;
    }
}

static void fsm_exit_state(control_fsm_state_t state, uint32_t now_ms)
{
    (void)now_ms;
    switch (state) {
        case CONTROL_FSM_STATE_FACTORY_TEST:
            factory_test_request_stop();
            break;
        case CONTROL_FSM_STATE_SERVICE:
            lights_display_motor_hours_end();
            break;
        default:
            break;
    }
}

static bool pull_down_complete(void)
{
    const control_mode_profile_t *profile =
        control_config_get_profile(CONTROL_MODE_MEDIUM_SLUSH);
    float bowl_temp = estimator_get_real_bowl_temp();
    float texture   = estimator_get_texture_index();
    float temp_margin = g_control_config.pull_down_temp_margin_c;
    float texture_margin = g_control_config.pull_down_texture_margin;

    /* Consider pull-down complete once both temperature and texture are close to target. */
    bool temp_ok = bowl_temp <= (profile->freeze_temp_c + temp_margin);
    bool texture_ok = texture >= fmaxf(0.0f, profile->texture_target - texture_margin);
    return temp_ok && texture_ok;
}

static bool deice_complete(uint32_t now_ms)
{
    if ((now_ms - s_fsm.deice_start_ms) < DEICE_MIN_DWELL_MS) {
        return false;
    }
    /* Once we have dwelled long enough, rely on the estimator to confirm thawing. */
    return !estimator_needs_deicing();
}

static void update_ready_indicator(bool enable)
{
    if (enable == s_fsm.ready_indicator_active) {
        return;
    }
    s_fsm.ready_indicator_active = enable;
    if (enable) {
        set_front_rgb_default(true);  /* Illuminate ring in "ready" colour. */
    } else {
        set_front_rgb_default(false); /* Dim the ring while product is not ready. */
    }
}

static void log_state_transition(control_fsm_state_t from,
                                 control_fsm_state_t to,
                                 uint32_t            now_ms)
{
    /* Centralised logging ensures every transition is accounted for in telemetry. */
    printf("FSM: %lu ms state %d -> %d\n", (unsigned long)now_ms, (int)from, (int)to);
}

/* ------------------------------------------------------------------------- */
/*                     External gesture entry points (weak hooks)            */
/* ------------------------------------------------------------------------- */

void fsm_request_service_mode(void)
{
    control_fsm_request_state(CONTROL_FSM_STATE_SERVICE);
}

void fsm_request_factory_mode(void)
{
    control_fsm_request_state(CONTROL_FSM_STATE_FACTORY_TEST);
}

#else /* ENABLE_CONTROL_FSM */

void control_fsm_init(void) {}
void control_fsm_reset(void) {}
void control_fsm_run_tick(uint32_t now_ms) {(void)now_ms;}
void control_fsm_set_target_temperature(int32_t target_c) {(void)target_c;}
void control_fsm_request_state(control_fsm_state_t state) {(void)state;}
control_fsm_state_t control_fsm_get_state(void)
{
    return CONTROL_FSM_STATE_POWER_ON;
}
bool control_fsm_is_transition_pending(void)
{
    return false;
}
void control_fsm_set_debug_mode(bool enabled) {(void)enabled;}
bool control_fsm_is_debug_mode(void)
{
    return false;
}

#endif /* ENABLE_CONTROL_FSM */
