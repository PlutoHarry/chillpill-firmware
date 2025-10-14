#ifndef FACTORY_TEST_H
#define FACTORY_TEST_H

#include "build_config.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @file factory_test.h
 * @brief Manufacturing and service diagnostics coordinator for the control FSM.
 *
 * The factory test module orchestrates scripted behaviours that exercise the
 * cooling subsystem during production and maintenance. It exposes control hooks
 * for the application layer to start or stop diagnostics while sharing sensor
 * feedback with the estimator and fault handler modules.
 */

#if ENABLE_CONTROL_FSM

/** Per-stage results captured during the factory diagnostics sequence. */
typedef struct {
    bool  leds_passed;              /**< Lighting exercise completed. */
    bool  fans_spinning;            /**< Fans reported motion during actuator test. */
    bool  motor_rpm_in_range;       /**< Auger RPM within configured band. */
    bool  motor_current_in_range;   /**< Auger current within configured band. */
    bool  sensors_in_range;         /**< Evaporator sensors within ambient bounds. */
    bool  cooling_temp_drop_ok;     /**< Cooling stage achieved required temperature drop. */
    bool  overall_pass;             /**< Aggregate pass/fail outcome. */
    bool  aborted;                  /**< True if the test was manually aborted. */
    float motor_rpm_measured;       /**< Observed auger RPM (RPM). */
    float motor_current_measured;   /**< Observed auger current (A). */
    float fan_one_rpm;              /**< Observed condenser fan #1 RPM. */
    float fan_two_rpm;              /**< Observed condenser fan #2 RPM. */
    float evap_in_temp_start;       /**< Evaporator inlet temperature at start of cooling (°C). */
    float evap_in_temp_end;         /**< Evaporator inlet temperature at end of cooling (°C). */
    float evap_out_temp_start;      /**< Evaporator outlet temperature at start of cooling (°C). */
    float evap_out_temp_end;        /**< Evaporator outlet temperature at end of cooling (°C). */
    float evap_in_temp_drop;        /**< Measured inlet temperature drop over cooling stage (°C). */
    float evap_out_temp_drop;       /**< Measured outlet temperature drop over cooling stage (°C). */
} factory_test_results_t;

void factory_test_init(void);
void factory_test_run(uint32_t now_ms);
void factory_test_request_start(void);
void factory_test_request_stop(void);
bool factory_test_is_active(void);
bool factory_test_is_complete(void);
bool factory_test_has_failed(void);
const factory_test_results_t *factory_test_get_results(void);

#else

typedef struct {
    bool  leds_passed;
    bool  fans_spinning;
    bool  motor_rpm_in_range;
    bool  motor_current_in_range;
    bool  sensors_in_range;
    bool  cooling_temp_drop_ok;
    bool  overall_pass;
    bool  aborted;
    float motor_rpm_measured;
    float motor_current_measured;
    float fan_one_rpm;
    float fan_two_rpm;
    float evap_in_temp_start;
    float evap_in_temp_end;
    float evap_out_temp_start;
    float evap_out_temp_end;
    float evap_in_temp_drop;
    float evap_out_temp_drop;
} factory_test_results_t;

static inline void factory_test_init(void) {}
static inline void factory_test_run(uint32_t now_ms) {(void)now_ms;}
static inline void factory_test_request_start(void) {}
static inline void factory_test_request_stop(void) {}
static inline bool factory_test_is_active(void) { return false; }
static inline bool factory_test_is_complete(void) { return false; }
static inline bool factory_test_has_failed(void) { return false; }
static inline const factory_test_results_t *factory_test_get_results(void)
{
    static const factory_test_results_t s_results = {0};
    return &s_results;
}

#endif /* ENABLE_CONTROL_FSM */

#endif /* FACTORY_TEST_H */
