/*
 * actuators.h
 * Updated: Oct 2025
 * Author: Harry Lawton
 *
 * PURPOSE
 * -------
 *   Declaration of the low-level actuator control layer for the ChillPill
 *   controller.  The implementation (actuators.c) owns all PWM timers and GPIO
 *   drive signals that energise the compressor inverter, auger motor and case
 *   fans.  It exposes a compact set of functions that higher level modules can
 *   call without needing to understand timer setup or safe operating limits.
 *
 * HOW TO USE
 * ----------
 *   1) Call actuators_init() once after the CubeMX-generated timer and GPIO
 *      initialisation has completed (typically near the start of main()).
 *   2) Request compressor set-points via set_compressor_speed(rpm) and inspect
 *      the most recently commanded value with get_compressor_speed().
 *   3) Drive the auger using set_motor_speed(target_auger_rpm); the helper will
 *      automatically manage the brake and direction pins.  Direct calls to the
 *      motor_enable()/disable() helpers are only required for maintenance tasks.
 *   4) Use set_fan_speed(percent) to command both condenser fans and
 *      pwm_fan_freq_set(freq) to change the PWM base frequency when adjusting
 *      acoustics.
 *
 * ADDITIONAL DETAILS
 * ------------------
 *   - All functions are safe to call from the cooperative scheduler.  Timer
 *     registers are updated atomically using HAL macros to avoid race
 *     conditions with interrupts.
 *   - PWM frequency selection is quantised to the resolution offered by the
 *     timer prescaler; see pwm_fan_freq_set() for the available options.
 *   - set_motor_speed() expects the requested RPM of the auger output shaft (on
 *     the drink side of the gearbox), not the BLDC electrical speed.
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
/**
 * @brief Prepare all PWM timers and GPIOs for safe actuator control.
 *
 * Starts each PWM channel with a zero duty cycle, ensures the auger brake is
 * asserted, and sets internal state used to guard subsequent function calls.
 */
void actuators_init(void);

/* --- Compressor --- */
/**
 * @brief Command the compressor inverter to the requested RPM set-point.
 *
 * Values below the minimum safe threshold automatically disable the inverter.
 */
void set_compressor_speed(uint16_t rpm);
/**
 * @brief Retrieve the last compressor speed requested via set_compressor_speed().
 */
uint16_t get_compressor_speed(void);

/* --- Motor --- */
/**
 * @brief Request a target auger speed in RPM (post-gearbox).
 *
 * Automatically handles brake release and duty ramping on first entry.
 */
void set_motor_speed(float target_auger_rpm);
/** Release the mechanical brake immediately (normally automatic). */
void motor_enable(void);
/** Engage the mechanical brake immediately (normally automatic). */
void motor_disable(void);
/** Energise the direction pin for clockwise auger rotation. */
void motor_clockwise(void);
/** Energise the direction pin for anticlockwise auger rotation. */
void motor_anticlockwise(void);

/* --- Fans --- */
/**
 * @brief Set both condenser fans to the same PWM duty cycle (10–100%).
 */
void set_fan_speed(uint8_t percent);
/**
 * @brief Adjust the PWM base frequency used to drive the fans.
 */
void pwm_fan_freq_set(fan_frequency_t freq);

#endif /* INC_ACTUATORS_H_ */
