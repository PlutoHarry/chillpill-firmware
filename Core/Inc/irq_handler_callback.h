#ifndef IRQ_HANDLER_CALLBACK_H
#define IRQ_HANDLER_CALLBACK_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/*
 * irq_handler_callback.h
 * Updated: Oct 2025
 *
 * PURPOSE
 * -------
 *   Shared definitions for the IRQ shim implemented in irq_handler_callback.c.
 *   The module bridges time-critical Cube/HAL interrupt callbacks (SysTick and
 *   TIM capture/overflow events) to the cooperative scheduler in main.c and the
 *   sensors module.  It provides flag variables that the main loop can poll
 *   without needing to access HAL handles directly.
 *
 * HOW TO USE
 * ----------
 *   - Ensure this header is included anywhere that needs access to the elapsed
 *     tick flags or the MOTOR_IC_TIMER_HZ definition (typically main.c and the
 *     sensors module).
 *   - Configure TIM3 in CubeMX so that its counter frequency matches
 *     MOTOR_IC_TIMER_HZ.  The default 1 MHz gives 1 µs resolution for the auger
 *     encoder period measurement.
 *   - In your main loop, poll the volatile tick flags (elapsed_1ms,
 *     g_ticks_20ms, etc.), act on them, then clear them back to zero.  Each flag
 *     is raised from within the HAL callbacks and remains set until cleared.
 *
 * ADDITIONAL DETAILS
 * ------------------
 *   - The HAL_TIM_IC_CaptureCallback implementation reconstructs absolute
 *     encoder tick counts using overflow tracking.  Only the resulting period
 *     is exposed to higher layers via sensors_encoder_capture().
 *   - The 30-second flag (g_ticks_30s) is useful for low-priority maintenance
 *     tasks such as periodic flash checkpoints.
 */

/* Encoder input-capture timer tick rate (Hz). */
#ifndef MOTOR_IC_TIMER_HZ
#define MOTOR_IC_TIMER_HZ 1000000UL
#endif

/* Public tick flags (set in IRQs, read/cleared in main loop). */
extern volatile uint8_t elapsed_1ms;
extern volatile uint8_t g_ticks_20ms;
extern volatile uint8_t g_ticks_500ms;
extern volatile uint8_t g_ticks_1s;
extern volatile uint8_t g_ticks_30s;

#endif /* IRQ_HANDLER_CALLBACK_H */
