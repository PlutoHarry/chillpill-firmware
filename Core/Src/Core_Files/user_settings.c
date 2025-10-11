/*
 *  user_settings.c
 *
 *  Created on: Oct 5th 2025
 *  Author: Harry Lawton
 *
 *  PURPOSE
 *  -------
 *  - Restore saved user settings and apply initial light states (no peripheral bring-up here)
 *  - Interpret button gesture durations into settings/state changes
 *  - Handle temporary preview (freeze mode) with 10 s confirm window
 *  - Defer flash writes for ring brightness & RGB default by 10 s of inactivity
 *  - Persist confirmed settings to flash, including rounded motor hours and live fault code
 *  - Provide exclusive entry triggers for Service and Factory modes (FSM handles states)
 *
 *  DEPENDENCIES
 *  ------------
 *  - flash_parms.{h,c} for persistence
 *  - lights.h (visual feedback), actuators.h (power control hooks), sensors.h (hours)
 *  - buttons.c emits change_user_settings()
 *
 *  PUBLIC API
 *  ----------
 *  void user_init(void);
 *  void change_user_settings(uint8_t power_sec, uint8_t freeze_sec, uint8_t light_sec);
 *  void user_settings_task_20ms(uint32_t now_ms);
 */

#include "main.h"
#include "lights.h"
#include "buttons.h"
#include "actuators.h"
#include "sensors.h"
#include "flash_parms.h"
#include "stm32f1xx_hal.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

/* ============================ External hooks / getters ============================ */

/* FSM requests (provide real implementations in your FSM module) */
__attribute__((weak)) void fsm_request_service_mode(void) {}
__attribute__((weak)) void fsm_request_factory_mode(void) {}

/* Control-FSM-only helpers used by legacy builds.  Provide harmless defaults
 * so the Core/Foundation firmware links when ENABLE_CONTROL_FSM == 0. */
__attribute__((weak)) void motor_fail_stop_timer(void) {}
__attribute__((weak)) void compressor_fail_clear(void) {}
__attribute__((weak)) void pwm_inverter_started_reset(void) {}

/* Fault snapshot getter – implement this where you track current live fault.
   Return 0 when no fault; non-zero for your chosen code scheme. */
__attribute__((weak)) uint8_t get_live_fault_condition(void) { return 0; }

/* Motor hours (exact fractional hours). Implemented in sensors.c per your plan. */
extern float get_motor_hours(void);

/* ============================ Persistent settings container ============================ */
/* Make sure these fields exist in params_settings_t (flash_parms.h/.c):
     uint8_t  saved_freeze_mode;
     uint8_t  saved_led_ring_brightness;   // 0..100
     uint8_t  saved_rgb_led_default;       // 0=OFF default, 1=WHITE default
     uint32_t saved_motor_hours;           // rounded hours
     uint8_t  saved_live_fault_condition;  // last saved live fault snapshot
*/
params_settings_t usr_params_settings = {0};

/* ============================ Runtime (volatile) state ============================ */

static bool     system_power_on         = false;  /* power rail latch (from user_power_on/off) */
static bool     pulse_mode_active       = false;

static uint8_t  current_freeze_mode     = 0;      /* 0..4 */
static uint8_t  current_ring_brightness = 50;     /* % */
static bool     rgb_default_white       = true;   /* idle front RGB = white or off */

/* Temporary freeze selection waiting for 10 s confirm */
static uint8_t  pending_freeze_mode     = 0;
static uint32_t last_selection_time_ms  = 0;

/* Deferred-save for ring brightness & RGB default */
static bool     pending_save_brightness    = false;
static bool     pending_save_rgb_default   = false;
static uint32_t pending_save_epoch_ms      = 0;   /* last change time; 10 s quiet → save */

/* ============================ Persistence helpers ============================ */

static void user_settings_save_all(void)
{
    /* Copy current runtime state to persistent struct */
    usr_params_settings.saved_freeze_mode         = current_freeze_mode;
    usr_params_settings.saved_led_ring_brightness = current_ring_brightness;
    usr_params_settings.saved_rgb_led_default     = rgb_default_white ? 1U : 0U;

    /* Also store rounded motor-hours and the latest live-fault snapshot */
    float    hrs_f   = get_motor_hours();
    uint32_t hrs_rnd = (uint32_t)(hrs_f + 0.5f);
    usr_params_settings.saved_motor_hours          = hrs_rnd;
    usr_params_settings.saved_live_fault_condition = get_live_fault_condition();

    if (!flash_params_save(&usr_params_settings)) {
        printf("WARN: flash_params_save() failed\n");
    }
}

/* Better name: push saved settings (already loaded) to the actual outputs */
static void apply_saved_settings_to_outputs(void)
{
    /* Lights base state */
    set_ring_led_level_percent(current_ring_brightness);
    set_front_rgb_default(rgb_default_white);
    set_freeze_btn_color((freeze_btn_color)current_freeze_mode);

    /* Clear any pulse/flash by default */
    set_ring_led_pulse_enable(false);
    set_ring_led_flash_enable(false);

    pulse_mode_active = false;
}

/* ============================ Public API ============================ */

/* Load from flash and apply initial visual state (no peripheral start here). */
void user_init(void)
{
    /* Load from flash; if fails, seed defaults and save once */
    if (!flash_params_load(&usr_params_settings)) {
        usr_params_settings.saved_freeze_mode          = 0;
        usr_params_settings.saved_led_ring_brightness  = 50;
        usr_params_settings.saved_rgb_led_default      = 1U;
        usr_params_settings.saved_motor_hours          = 0U;
        usr_params_settings.saved_live_fault_condition = 0U;
        (void)flash_params_save(&usr_params_settings);
    }

    /* Populate runtime state from saved values */
    current_freeze_mode     = (uint8_t)usr_params_settings.saved_freeze_mode;
    current_ring_brightness = (uint8_t)usr_params_settings.saved_led_ring_brightness;
    rgb_default_white       = (usr_params_settings.saved_rgb_led_default != 0);

    /* Seed the motor hours accumulator with the rounded value from Flash
     * before any session accumulation begins.  Without this call the
     * reported lifetime hours would always start from zero. */
    sensors_set_lifetime_hours_base(usr_params_settings.saved_motor_hours);

    /* No preview or pending saves on boot */
    pending_freeze_mode     = current_freeze_mode;
    last_selection_time_ms  = 0;
    pending_save_brightness = false;
    pending_save_rgb_default= false;
    pending_save_epoch_ms   = 0;

    /* Apply base visuals */
    apply_saved_settings_to_outputs();
}

/* Explicit power control from gestures; do not defer here */
void user_power_on(void)
{
    HAL_GPIO_WritePin(PWOER_CTRL_PC1_GPIO_Port, PWOER_CTRL_PC1_Pin, GPIO_PIN_SET);
    system_power_on   = true;
    pulse_mode_active = false;
    set_ring_led_pulse_enable(false);
}

void user_power_off(void)
{
    /* Actuator and fault modules reinitialise on the next boot, so the
     * legacy manual reset hooks are no longer required here. */
    HAL_GPIO_WritePin(PWOER_CTRL_PC1_GPIO_Port, PWOER_CTRL_PC1_Pin, GPIO_PIN_RESET);
    system_power_on   = false;

    /* Exit pulse promptly */
    if (pulse_mode_active) {
        set_ring_led_pulse_enable(false);
        pulse_mode_active = false;
    }

    /* One final save to avoid losing latest choices */
    user_settings_save_all();

    /* Clear any pending deferred save intents (they are now committed) */
    pending_save_brightness  = false;
    pending_save_rgb_default = false;
    pending_save_epoch_ms    = 0;
}

/* Commit the 10 s preview to active + flash */
void apply_user_settings(void)
{
    current_freeze_mode = pending_freeze_mode;

    set_freeze_btn_color((freeze_btn_color)current_freeze_mode);
    set_front_rgb_default(rgb_default_white);
    set_ring_led_level_percent(current_ring_brightness);

    /* Save all persisted fields, including hours and fault snapshot */
    user_settings_save_all();

    /* Freeze confirmation is its own save event; doesn't touch brightness/RGB deferral flags */
    printf("Freeze mode confirmed & saved: %u\n", current_freeze_mode);
}

/* Interpret one combined gesture (button durations rounded by buttons.c) */
void change_user_settings(uint8_t power_sec, uint8_t freeze_sec, uint8_t light_sec)
{
    uint32_t now = HAL_GetTick();

    /* ---------------- Exclusive special combos ----------------
       Service mode: FREEZE >= 5s AND LIGHT >= 5s AND POWER == 0
       Factory mode: POWER  >=10s AND FREEZE>=10s AND LIGHT == 0
       (No third button allowed)
    ------------------------------------------------------------*/
    if (power_sec == 0 && freeze_sec >= 5 && light_sec >= 5) {
        fsm_request_service_mode();
        printf("Service mode requested\n");
        return;
    }

    if (power_sec >= 10 && freeze_sec >= 10 && light_sec == 0) {
        fsm_request_factory_mode();
        printf("Factory mode requested\n");
        return;
    }

    /* ---------------- POWER alone ---------------- */
    if (power_sec > 0 && freeze_sec == 0 && light_sec == 0) {
        if (system_power_on) user_power_off();
        else                 user_power_on();
        printf("Power toggled: %s\n", system_power_on ? "ON" : "OFF");
        return;
    }

    /* ---------------- FREEZE alone ----------------
       Rotate freeze mode 0..4 with 10 s preview window (saved on confirm). */
    if (freeze_sec > 0 && power_sec == 0 && light_sec == 0) {
        pending_freeze_mode    = (current_freeze_mode + 1U) % 5U;
        last_selection_time_ms = now;

        /* Preview: button + front RGB for 10 s (lights.c reverts after timeout) */
        set_freeze_btn_color((freeze_btn_color)pending_freeze_mode);
        set_front_rgb_preview_selection_10s((rgb_color_t)pending_freeze_mode);

        printf("Freeze mode preview: %u (waiting 10 s)\n", pending_freeze_mode);
        return;
    }

    /* ---------------- LIGHT alone ---------------- */
    if (light_sec > 0 && power_sec == 0 && freeze_sec == 0) {

        if (light_sec > 5) {
            /* Enter/exit ring pulse mode (transient; not saved) */
            pulse_mode_active = !pulse_mode_active;
            set_ring_led_pulse_enable(pulse_mode_active);
            set_ring_led_flash_enable(false);
            printf("Pulse mode %s\n", pulse_mode_active ? "enabled" : "disabled");
            return;
        }

        if (light_sec >= 2) {
            /* Toggle front RGB default white – apply now, defer save for 10 s of quiet */
            rgb_default_white = !rgb_default_white;
            set_front_rgb_default(rgb_default_white);

            pending_save_rgb_default = true;
            pending_save_epoch_ms    = now;  /* coalesce further changes */

            printf("RGB default: %s (deferred save)\n", rgb_default_white ? "WHITE" : "OFF");
            return;
        }

        /* Short tap (<2 s): cycle ring brightness – apply now, defer save 10 s */
        uint8_t lvl = current_ring_brightness;
        if      (lvl < 25)  lvl = 25;
        else if (lvl < 50)  lvl = 50;
        else if (lvl < 75)  lvl = 75;
        else if (lvl < 100) lvl = 100;
        else                lvl = 0;

        current_ring_brightness = lvl;
        set_ring_led_level_percent(lvl);

        pending_save_brightness = true;
        pending_save_epoch_ms   = now;  /* coalesce further changes */

        printf("Ring brightness set to %u%% (deferred save)\n", lvl);
        return;
    }

    /* ---------------- Unhandled combos ----------------
       Add future multi-button gestures here if needed.
     */
}

/* Call every ~20 ms */
void user_settings_task_20ms(uint32_t now_ms)
{
    /* 10 s confirmation for freeze preview */
    if (pending_freeze_mode != current_freeze_mode &&
        last_selection_time_ms != 0 &&
        (now_ms - last_selection_time_ms) >= 10000U)
    {
        apply_user_settings();       /* commits + saves immediately */
        last_selection_time_ms = 0;
    }

    /* Deferred save for brightness / RGB default:
       If there were changes and we’ve been quiet for 10 s, persist once. */
    if ((pending_save_brightness || pending_save_rgb_default) &&
        (now_ms - pending_save_epoch_ms) >= 10000U)
    {
        user_settings_save_all();
        pending_save_brightness  = false;
        pending_save_rgb_default = false;
        pending_save_epoch_ms    = 0;
        printf("Deferred settings saved\n");
    }

    /* -------- Periodic checkpoint to survive hard power pulls --------
       - Save when rounded motor-hours changes (no-op if unchanged).
       - Save when live fault code changes (no-op if unchanged).
       This adds minimal extra wear because both conditions change infrequently. */
    {
        uint32_t hrs_rnd = (uint32_t)(get_motor_hours() + 0.5f);
        uint8_t  fault   = get_live_fault_condition();

        bool need_save = false;

        if (hrs_rnd != usr_params_settings.saved_motor_hours) {
            usr_params_settings.saved_motor_hours = hrs_rnd;
            need_save = true;
        }
        if (fault != usr_params_settings.saved_live_fault_condition) {
            usr_params_settings.saved_live_fault_condition = fault;
            need_save = true;
        }

        if (need_save) {
            /* Keep other persisted fields in sync as well */
            usr_params_settings.saved_freeze_mode         = current_freeze_mode;
            usr_params_settings.saved_led_ring_brightness = current_ring_brightness;
            usr_params_settings.saved_rgb_led_default     = rgb_default_white ? 1U : 0U;

            (void)flash_params_save(&usr_params_settings);
            // Optional debug:
            // printf("Checkpoint save: hrs=%lu, fault=%u\n", (unsigned long)hrs_rnd, fault);
        }
    }

    /* Auto-exit pulse mode on power-off */
    if (pulse_mode_active && !system_power_on) {
        set_ring_led_pulse_enable(false);
        pulse_mode_active = false;
        printf("Pulse mode auto-disabled on power-off\n");
    }
}
