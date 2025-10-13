#ifndef USER_SETTINGS_H
#define USER_SETTINGS_H

/*
 *  user_settings.h
 *
 *  PURPOSE
 *  -------
 *  Public API for restoring, changing, and applying user settings, and
 *  for driving time-based confirmation / deferred-save logic.
 *
 *  HOW TO USE
 *  ----------
 *  1) Call user_init() once after HAL/Cube peripheral init to:
 *       - Load saved settings from flash (freeze mode, ring brightness,
 *         RGB default, rounded motor hours, live fault code)
 *       - Apply initial light states (no peripheral bring-up here)
 *
 *  2) When buttons.c emits a combined gesture, call:
 *       change_user_settings(power_sec, freeze_sec, light_sec);
 *     Durations are whole seconds (0–10), already rounded/clamped by buttons.c.
 *     Special exclusive combos:
 *       - Service mode   : FREEZE >= 5s && LIGHT >= 5s && POWER == 0
 *       - Factory mode   : POWER  >=10s && FREEZE>=10s && LIGHT == 0
 *     Singles:
 *       - POWER  (>0s)   : toggle power
 *       - FREEZE (>0s)   : rotate freeze mode with 10s preview (saved on confirm)
 *       - LIGHT  <2s     : cycle ring brightness (0,25,50,75,100) — deferred save 10s
 *       - LIGHT  >=2s    : toggle RGB default white — deferred save 10s
 *       - LIGHT  >5s     : toggle ring pulse mode (not saved)
 *
 *  3) Call user_settings_task_20ms(now_ms) every ~20 ms to:
 *       - Finalize freeze-mode after 10 s preview (apply_user_settings())
 *       - Persist deferred brightness/RGB-default after 10 s inactivity
 *       - Periodically checkpoint rounded motor hours & live fault code
 *       - Auto-exit pulse mode upon power-off
 *
 *  NOTES
 *  -----
 *  - Flash I/O is handled internally via flash_parms.{h,c}.
 *  - Motor hours are read via get_motor_hours() (from sensors.c).
 *  - Live fault code is read via get_live_fault_condition() (weak stub provided).
 *  - Direction changes are NOT handled here.
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One-time restore + apply base visuals (no peripheral bring-up here) */
void user_init(void);

/* Explicit power control used by gesture handling */
void user_power_on(void);
void user_power_off(void);

/* Commit the 10 s freeze preview to active state and save immediately */
void apply_user_settings(void);

/* Interpret a combined button gesture (call from buttons.c) */
void change_user_settings(uint8_t power_sec,
                          uint8_t freeze_sec,
                          uint8_t light_sec);

/* Periodic driver (~20 ms) for confirmations, deferred saves, and checkpoints */
void user_settings_task_20ms(uint32_t now_ms);

/* Inspectors used by the control FSM / debug mode */
uint8_t user_settings_get_ring_brightness(void);
uint8_t user_settings_get_effective_freeze_mode(void);

#ifdef __cplusplus
}
#endif

#endif /* USER_SETTINGS_H */
