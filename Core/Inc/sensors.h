#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Sensors module public API
 * --------------------------------------------------------------------------
 * Usage:
 * - Call sensors_init() once after peripheral init (GPIO/ADC/Timers ready).
 *   This performs ADC calibration and starts ADC1+DMA, and enables the
 *   capture timers for motor and fans.
 * - Call update_sensor_data() periodically (e.g., every 1 ms).
 * - From your timer IRQs, forward events via the IRQ hooks below.
 * - Temperature outputs are clamped to your app range (e.g. −20..40 °C).
 * - Lifetime motor hours:
 *     • Call sensors_set_lifetime_hours_base(rounded_hours_from_flash) once
 *       after you load settings from flash.
 *     • get_motor_hours() then returns base + exact fractional session hours.
 */

/* Lifecycle */
void sensors_init(void);
void update_sensor_data(void);

/* IRQ forwarders (call from timer interrupts) */
void sensors_encoder_capture(float dt_seconds); /* motor encoder: time between edges (seconds) */
void sensors_on_fan_one_pulse(void);            /* fan 1 FG rising edge */
void sensors_on_fan_two_pulse(void);            /* fan 2 FG rising edge */

/* Public getters (return 0 on success, non-zero if pointer is NULL) */
uint8_t get_evap_in_temp (float *temperature_out);  /* °C */
uint8_t get_evap_out_temp(float *temperature_out);  /* °C */
uint8_t get_bowl_temp    (float *temperature_out);  /* °C */

uint8_t get_motor_current(float *current_out);      /* A */

uint8_t get_fan_one_speed(float *rpm_out);          /* RPM */
uint8_t get_fan_two_speed(float *rpm_out);          /* RPM */
uint8_t get_auger_speed  (float *rpm_out);          /* RPM */

/* Lifetime motor-hours interface */
void  sensors_set_lifetime_hours_base(uint32_t rounded_hours_from_flash);
/* Returns exact fractional lifetime hours = base (rounded from flash) + session */
float get_motor_hours(void);

/* --------------------------------------------------------------------------
 * Optional build-time knobs (define these BEFORE including this header
 * or via compiler flags to override the defaults in sensors.c):
 *
 *  - NUMBER_OF_ADC_CHANNEL
 *  - ROLLING_AVG_SAMPLES
 *  - ROLLING_AVG_SIZE_FOR_CURRENT
 *  - MOTOR_RPM_AVG_SAMPLES
 *  - MOTOR_PULSES_PER_REV
 *  - FAN_PULSES_PER_REV
 *  - FAN_RPM_WINDOW_MS
 *  - ENC_MIN_DT_S / ENC_MAX_DT_S
 *  - SENSOR_TEMP_MIN_C / SENSOR_TEMP_MAX_C
 *  - NTC_OPER_MIN_C / NTC_OPER_MAX_C
 *  - NTC_TABLE_T_START_C
 * -------------------------------------------------------------------------- */

#ifdef __cplusplus
}
#endif

#endif /* SENSORS_H */
