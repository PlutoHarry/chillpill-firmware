/*
 * irq_handler_callback.c
 *
 * PURPOSE
 *   Minimal timing + capture layer:
 *     - SysTick (1 ms): raises elapsed_1ms and aggregates into 20ms / 500ms / 1s / 30s tick flags
 *     - TIM3 CH1 (motor encoder): edge-to-edge dt -> sensors_encoder_capture(dt_s)
 *     - TIM4 CH2/CH4 (fan FG): forwards pulses -> sensors_on_fan_one_pulse/two_pulse
 *
 * DEPENDENCIES
 *   - MOTOR_IC_TIMER_HZ must match TIM3 tick frequency used for encoder capture
 *   - sensors.c provides sensors_encoder_capture() and fan pulse hooks
 */

#include "stm32f1xx_hal.h"
#include "sensors.h"
#include "irq_handler_callback.h"

/* ---- Public tick flags polled by main loop ---- */
volatile uint8_t elapsed_1ms  = 0;  /* raised every SysTick */
volatile uint8_t g_ticks_20ms = 0;
volatile uint8_t g_ticks_500ms = 0;
volatile uint8_t g_ticks_1s   = 0;
volatile uint8_t g_ticks_30s  = 0;

/* ---- Local ms accumulators ---- */
static uint16_t ms_accum_20   = 0;
static uint16_t ms_accum_500  = 0;
static uint16_t ms_accum_1000 = 0;
static uint32_t ms_accum_30000 = 0;

/* ---- TIM3 overflow tracking for motor encoder capture ---- */
static volatile uint32_t tim3_overflows = 0;
static uint32_t last_tim3_total_ticks = 0;
static uint8_t  have_tim3_prev = 0;

/* ============================== SysTick ============================== */
void HAL_SYSTICK_Callback(void)
{
    /* 1 ms tick */
    elapsed_1ms = 1;

    /* Aggregate into coarser ticks */
    if (++ms_accum_20 >= 20U)     { ms_accum_20 = 0; g_ticks_20ms = 1; }
    if (++ms_accum_500 >= 500U)   { ms_accum_500 = 0; g_ticks_500ms = 1; }
    if (++ms_accum_1000 >= 1000U) { ms_accum_1000 = 0; g_ticks_1s = 1; }

    if (++ms_accum_30000 >= 30000UL) {
        ms_accum_30000 = 0;
        g_ticks_30s = 1;
    }
}

/* ==================== Period elapsed (timer overflow) ==================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* Track TIM3 overflows for motor encoder absolute tick reconstruction */
    if (htim->Instance == TIM3) {
        tim3_overflows++;
    }
}

/* =========================== Input capture events ========================== */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* ---------------- Motor encoder on TIM3, CH1 ---------------- */
    if (htim->Instance == TIM3 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        uint32_t ccr = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);

        /* absolute tick count since start: overflows * (ARR+1) + CCR1 */
        uint32_t total_ticks = (tim3_overflows * (arr + 1U)) + ccr;

        if (have_tim3_prev) {
            uint32_t dt_ticks = total_ticks - last_tim3_total_ticks;
            if (dt_ticks > 0U) {
                float dt_seconds = (float)dt_ticks / (float)MOTOR_IC_TIMER_HZ;
                sensors_encoder_capture(dt_seconds);
            }
        }
        last_tim3_total_ticks = total_ticks;
        have_tim3_prev = 1;
        return;
    }

    /* ---------------- Fan FG pulses on TIM4 ---------------- */
    if (htim->Instance == TIM4) {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
            sensors_on_fan_one_pulse();
            return;
        }
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
            sensors_on_fan_two_pulse();
            return;
        }
    }
}
