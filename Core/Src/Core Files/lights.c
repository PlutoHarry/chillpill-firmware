/*
 * lights.c
 * Updated: Oct 2025
 * Author: Harry Lawton
 *
 * PURPOSE:
 *   Non-blocking control of all user-visible lighting:
 *     - LED ring (PWM brightness, pulse animation, flash sequence)
 *     - Front RGB(W) indicator (default colour, fault blink, preview)
 *     - Freeze button LED (tri-colour WGB)
 *     - Light button LED
 *     - On/Off button LED (if present)
 *
 * CALL RATE:
 *   Call set_lights_task_20ms(now_ms) every ~20 ms from main loop or timer tick.
 *
 * DESIGN RULES:
 *   - No HAL_Delay(); non-blocking time-based animations only.
 *   - Each group (ring / RGB / buttons) is independent.
 *   - When routines end, previous base states are restored automatically.
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "main.h"
#include "lights.h"

/* External timer handle (PWM ring LED) */
extern TIM_HandleTypeDef RING_LED_PWM_TIM;

/* Optional hardware guard */
#if !defined(ONOFF_LED_GPIO_Port) || !defined(ONOFF_LED_Pin)
#define NO_ONOFF_LED 1
#endif

/* -------------------------------------------------------------------------- */
/*                                CONFIGURATION                               */
/* -------------------------------------------------------------------------- */

#define PULSE_CYCLE_MS        2000U
#define PULSE_MIN_BR          0.25f
#define PULSE_MAX_BR          1.00f

#define RING_FLASHES_ON_STEADY  10U
#define RING_FLASH_ON_MS        120U
#define RING_FLASH_OFF_MS       180U

#define FAULT_BLINK_ON_MS       180U
#define FAULT_BLINK_OFF_MS      220U
#define FAULT_SEQUENCE_GAP_MS   800U

#define SELECTION_PREVIEW_MS    10000U /* 10 seconds */

/* -------------------------------------------------------------------------- */
/*                              MODULE STATE                                  */
/* -------------------------------------------------------------------------- */

/* RING LED */
static float    ring_level = 1.0f;
static bool     ring_pulse_en = false;
static uint32_t ring_pulse_epoch = 0;

static bool     ring_flash_en = false;
static uint8_t  ring_flash_done = 0;
static bool     ring_flash_high = false;
static uint32_t ring_flash_epoch = 0;

/* FRONT RGB */
static bool     front_default_on_white = false;
static bool     selection_preview_active = false;
static uint32_t selection_preview_epoch  = 0;

static fault_type_t fault_mode = FAULT_OFF;
static uint8_t      fault_blinks = 0;
static uint8_t      fault_index  = 0;
static bool         fault_phase_on = false;
static uint32_t     fault_epoch = 0;

/* -------------------------------------------------------------------------- */
/*                             INITIALIZATION                                 */
/* -------------------------------------------------------------------------- */

void lights_init(void)
{
    /* Initialize and start PWM for ring LED */
    HAL_TIM_Base_Start(&RING_LED_PWM_TIM);
    HAL_TIM_PWM_Init(&RING_LED_PWM_TIM);
    HAL_TIM_PWM_Start(&RING_LED_PWM_TIM, RING_LED_PWM_CHANNEL);

    set_ring_led_level_percent(100U);
}

/* -------------------------------------------------------------------------- */
/*                              GPIO UTILITIES                                */
/* -------------------------------------------------------------------------- */

static inline void gpio_set(GPIO_TypeDef *port, uint16_t pin, bool on)
{
    HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* FRONT RGB(W) channels */
static void front_rgb_channels(bool r, bool g, bool b, bool w)
{
    gpio_set(LED_R_GPIO_Port, LED_R_Pin, r);
    gpio_set(LED_G_GPIO_Port, LED_G_Pin, g);
    gpio_set(LED_B_GPIO_Port, LED_B_Pin, b);
    gpio_set(LED_W_GPIO_Port, LED_W_Pin, w);
}

/* Map colour enum to channel pins */
static void apply_front_rgb_color(rgb_color_t color)
{
    switch (color)
    {
        case RGB_OFF:       front_rgb_channels(false,false,false,false); break;
        case RGB_GREEN:     front_rgb_channels(false,true, false,false); break;
        case RGB_WHITE:     front_rgb_channels(false,false,false,true ); break;
        case RGB_LIGHTBLUE: front_rgb_channels(false,true, true, false); break;
        case RGB_BLUE:      front_rgb_channels(false,false,true, false); break;
        case RGB_RED:       front_rgb_channels(true, false,false,false); break;
        default:            front_rgb_channels(false,false,false,false); break;
    }
}

/* Freeze button WGB (GPIOB pins 12–14) */
static void apply_freeze_btn_color(freeze_btn_color color)
{
    gpio_set(GPIOB, GPIO_PIN_12, false);
    gpio_set(GPIOB, GPIO_PIN_13, false);
    gpio_set(GPIOB, GPIO_PIN_14, false);

    switch (color)
    {
        case FREEZE_BTN_GREEN:     gpio_set(GPIOB, GPIO_PIN_13, true); break;
        case FREEZE_BTN_WHITE:     gpio_set(GPIOB, GPIO_PIN_14, true); break;
        case FREEZE_BTN_LIGHTBLUE: gpio_set(GPIOB, GPIO_PIN_12, true);
                                   gpio_set(GPIOB, GPIO_PIN_13, true); break;
        case FREEZE_BTN_BLUE:      gpio_set(GPIOB, GPIO_PIN_12, true); break;
        default: break;
    }
}

/* -------------------------------------------------------------------------- */
/*                                 RING LED                                   */
/* -------------------------------------------------------------------------- */

void set_ring_led_level_percent(uint8_t percent_0_100)
{
    if (percent_0_100 > 100U) percent_0_100 = 100U;
    ring_level = (float)percent_0_100 / 100.0f;

    uint32_t period = (uint32_t)(__HAL_TIM_GET_AUTORELOAD(&RING_LED_PWM_TIM) + 1U);
    uint32_t pulse  = (uint32_t)(ring_level * (float)period + 0.5f);
    __HAL_TIM_SET_COMPARE(&RING_LED_PWM_TIM, RING_LED_PWM_CHANNEL, pulse);
}

void set_ring_led_pulse_enable(bool on)
{
    ring_pulse_en = on;
    if (on)
        ring_pulse_epoch = HAL_GetTick();
    else
        set_ring_led_level_percent((uint8_t)(ring_level * 100.0f + 0.5f));
}

void set_ring_led_flash_enable(bool on)
{
    if (on)
    {
        ring_flash_en   = true;
        ring_flash_done = 0;
        ring_flash_high = true;
        ring_flash_epoch= HAL_GetTick();
    }
    else
    {
        ring_flash_en = false;
        set_ring_led_level_percent((uint8_t)(ring_level * 100.0f + 0.5f));
    }
}

/* -------------------------------------------------------------------------- */
/*                                FRONT RGB                                   */
/* -------------------------------------------------------------------------- */

void set_front_rgb_default(bool on_white)
{
    front_default_on_white = on_white;
    if (!selection_preview_active && fault_mode == FAULT_OFF)
        apply_front_rgb_color(on_white ? RGB_WHITE : RGB_OFF);
}

void set_front_rgb_preview_selection_10s(rgb_color_t color)
{
    selection_preview_active = true;
    selection_preview_epoch  = HAL_GetTick();
    apply_front_rgb_color(color);
}

void set_front_rgb_fault(fault_type_t mode)
{
    fault_mode   = mode;
    fault_index  = 0;
    fault_phase_on = (mode != FAULT_OFF);
    fault_epoch  = HAL_GetTick();

    switch (mode)
    {
        case FAULT_OFF:
            fault_blinks = 0;
            apply_front_rgb_color(front_default_on_white ? RGB_WHITE : RGB_OFF);
            break;
        case FAULT_MOTOR:      fault_blinks = 1; apply_front_rgb_color(RGB_RED); break;
        case FAULT_COMPRESSOR: fault_blinks = 2; apply_front_rgb_color(RGB_RED); break;
        case FAULT_FAN:        fault_blinks = 3; apply_front_rgb_color(RGB_RED); break;
        default:
            fault_blinks = 0;
            apply_front_rgb_color(front_default_on_white ? RGB_WHITE : RGB_OFF);
            break;
    }
}

/* -------------------------------------------------------------------------- */
/*                               BUTTON LEDS                                  */
/* -------------------------------------------------------------------------- */

void set_freeze_btn_color(freeze_btn_color color)
{
    apply_freeze_btn_color(color);
}

void set_light_btn_led(bool on)
{
    gpio_set(GPIOB, GPIO_PIN_15, on);
}

void set_on_btn_led(bool on)
{
#ifndef NO_ONOFF_LED
    gpio_set(ONOFF_LED_GPIO_Port, ONOFF_LED_Pin, on);
#else
    (void)on;
#endif
}

/* -------------------------------------------------------------------------- */
/*                                PERIODIC TASK                               */
/* -------------------------------------------------------------------------- */

void set_lights_task_20ms(uint32_t now_ms)
{
    /* --- Ring Pulse --- */
    if (ring_pulse_en && !ring_flash_en)
    {
        if (ring_pulse_epoch == 0U || (now_ms - ring_pulse_epoch) > PULSE_CYCLE_MS)
            ring_pulse_epoch = now_ms;

        float phase = (float)(now_ms - ring_pulse_epoch) / (float)PULSE_CYCLE_MS;
        float br = PULSE_MIN_BR + (PULSE_MAX_BR - PULSE_MIN_BR) * 0.5f *
                   (1.0f - cosf(2.0f * (float)M_PI * phase));

        uint32_t period = (uint32_t)(__HAL_TIM_GET_AUTORELOAD(&RING_LED_PWM_TIM) + 1U);
        uint32_t pulse  = (uint32_t)(br * (float)period + 0.5f);
        __HAL_TIM_SET_COMPARE(&RING_LED_PWM_TIM, RING_LED_PWM_CHANNEL, pulse);
    }

    /* --- Ring Flash --- */
    if (ring_flash_en)
    {
        uint32_t elapsed = now_ms - ring_flash_epoch;
        if (ring_flash_high)
        {
            if (elapsed >= RING_FLASH_ON_MS)
            {
                ring_flash_high = false;
                ring_flash_epoch = now_ms;
                set_ring_led_level_percent(0U);
            }
            else
                set_ring_led_level_percent(100U);
        }
        else
        {
            if (elapsed >= RING_FLASH_OFF_MS)
            {
                ring_flash_done++;
                if (ring_flash_done >= RING_FLASHES_ON_STEADY)
                {
                    ring_flash_en = false;
                    set_ring_led_level_percent((uint8_t)(ring_level * 100.0f + 0.5f));
                }
                else
                {
                    ring_flash_high = true;
                    ring_flash_epoch = now_ms;
                }
            }
        }
    }

    /* --- Fault pattern (only when no preview active) --- */
    if (!selection_preview_active && fault_blinks > 0U)
    {
        uint32_t elapsed = now_ms - fault_epoch;

        if (fault_phase_on)
        {
            if (elapsed >= FAULT_BLINK_ON_MS)
            {
                fault_phase_on = false;
                fault_epoch = now_ms;
                apply_front_rgb_color(RGB_OFF);
            }
        }
        else
        {
            if (fault_index < fault_blinks)
            {
                if (elapsed >= FAULT_BLINK_OFF_MS)
                {
                    fault_phase_on = true;
                    fault_epoch = now_ms;
                    fault_index++;
                    apply_front_rgb_color(RGB_RED);
                }
            }
            else if (elapsed >= FAULT_SEQUENCE_GAP_MS)
            {
                fault_index = 0;
                fault_phase_on = true;
                fault_epoch = now_ms;
                apply_front_rgb_color(RGB_RED);
            }
        }
    }

    /* --- Preview timeout --- */
    if (selection_preview_active &&
        (now_ms - selection_preview_epoch) >= SELECTION_PREVIEW_MS)
    {
        selection_preview_active = false;
        if (fault_mode == FAULT_OFF)
            apply_front_rgb_color(front_default_on_white ? RGB_WHITE : RGB_OFF);
    }
}
