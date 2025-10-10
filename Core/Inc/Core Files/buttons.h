#ifndef BUTTONS_H
#define BUTTONS_H

/*
 * buttons.h
 *
 * Public API for non-blocking, debounced button gesture detection.
 * Implements combined gestures with per-button duration reporting.
 *
 * Usage:
 *   1) Call buttons_init() once after GPIOs are initialized.
 *   2) Call buttons_task_20ms(HAL_GetTick()) every ~20 ms.
 *
 * The implementation will invoke:
 *   void change_user_settings(uint8_t power_sec,
 *                             uint8_t freeze_sec,
 *                             uint8_t light_sec);
 * (provided by user_settings.c)
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One-time setup after GPIO init */
void buttons_init(void);

/* Periodic task (~20 ms cadence) */
void buttons_task_20ms(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* BUTTONS_H */
