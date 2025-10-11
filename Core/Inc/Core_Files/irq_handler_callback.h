#ifndef IRQ_HANDLER_CALLBACK_H
#define IRQ_HANDLER_CALLBACK_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/* The encoder input-capture timer tick rate (Hz).
 * Configure TIM3 so its counter runs at this frequency (1 MHz recommended). */
#ifndef MOTOR_IC_TIMER_HZ
#define MOTOR_IC_TIMER_HZ 1000000UL
#endif

/* Public tick flags (set in IRQs, read/cleared in main loop) */
extern volatile uint8_t elapsed_1ms;
extern volatile uint8_t g_ticks_20ms;
extern volatile uint8_t g_ticks_500ms;
extern volatile uint8_t g_ticks_1s;
extern volatile uint8_t g_ticks_30s;

#endif /* IRQ_HANDLER_CALLBACK_H */
