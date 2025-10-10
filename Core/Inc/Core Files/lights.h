/*
 * lights.h
 * Updated Oct 2025
 * Author: Harry Lawton
 *
 * Public API for non-blocking light control.
 * Call set_lights_task_20ms(HAL_GetTick()) every ~20 ms.
 */

#ifndef INC_LIGHTS_H_
#define INC_LIGHTS_H_

#include <stdint.h>
#include <stdbool.h>

/* ---------------------------- ENUM DEFINITIONS ---------------------------- */

/* Front RGB(W) colours */
typedef enum {
    RGB_OFF = 0,
    RGB_GREEN,
    RGB_WHITE,
    RGB_LIGHTBLUE,
    RGB_BLUE,
    RGB_RED
} rgb_color_t;

/* Freeze button colours */
typedef enum {
    FREEZE_BTN_OFF = 0,
    FREEZE_BTN_GREEN,
    FREEZE_BTN_WHITE,
    FREEZE_BTN_LIGHTBLUE,
    FREEZE_BTN_BLUE
} freeze_btn_color;

/* Fault indicator types */
typedef enum {
    FAULT_OFF = 0,
    FAULT_MOTOR,
    FAULT_FAN,
    FAULT_COMPRESSOR
} fault_type_t;

/* -------------------------------------------------------------------------- */
/*                              FUNCTION PROTOTYPES                           */
/* -------------------------------------------------------------------------- */

/* --- Initialization --- */
void lights_init(void);

/* --- Ring LED --- */
void set_ring_led_level_percent(uint8_t percent_0_100);
void set_ring_led_pulse_enable(bool on);
void set_ring_led_flash_enable(bool on);

/* --- Front RGB(W) --- */
void set_front_rgb_default(bool on_white);
void set_front_rgb_preview_selection_10s(rgb_color_t color);
void set_front_rgb_fault(fault_type_t mode);

/* --- Button LEDs --- */
void set_freeze_btn_color(freeze_btn_color color);
void set_light_btn_led(bool on);
void set_on_btn_led(bool on);

/* --- Scheduler --- */
void set_lights_task_20ms(uint32_t now_ms);

#endif /* INC_LIGHTS_H_ */
