/*
 * actuators.h
 * Updated Oct 2025
 * Author: Harry Lawton
 */

#ifndef INC_ACTUATORS_H_
#define INC_ACTUATORS_H_

#include "stdbool.h"
#include "stm32f1xx_hal.h"

/* -------------------------------------------------------------------------- */
/*                                 ENUM TYPES                                 */
/* -------------------------------------------------------------------------- */

typedef enum {
    FAN_FREQ_166,
    FAN_FREQ_1K,
    FAN_FREQ_2K,
    FAN_FREQ_4K,
    FAN_FREQ_5K,
    FAN_FREQ_20K,
    FAN_FREQ_NUM
} fan_frequency_t;

/* -------------------------------------------------------------------------- */
/*                              FUNCTION PROTOTYPES                           */
/* -------------------------------------------------------------------------- */

/* --- Initialization --- */
void actuators_init(void);

/* --- Compressor --- */
void set_compressor_speed(uint16_t rpm);
uint16_t get_compressor_speed(void);

/* --- Motor --- */
void set_motor_speed(float target_rpm);
void motor_enable(void);
void motor_disable(void);
void motor_clockwise(void);
void motor_anticlockwise(void);

/* --- Fans --- */
void set_fan_speed(uint8_t percent);
void pwm_fan_freq_set(fan_frequency_t freq);

#endif /* INC_ACTUATORS_H_ */
