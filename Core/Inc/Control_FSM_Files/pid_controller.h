#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "build_config.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @file pid_controller.h
 * @brief Closed-loop control interface that drives compressor power requests.
 *
 * The PID controller consumes filtered estimates and set-points from the
 * finite state machine. It collaborates closely with the estimator module to
 * maintain stable cabinet temperatures and informs the fault handler when
 * saturation or instability is detected.
 */

#if ENABLE_CONTROL_FSM

void pid_controller_init(void);
void pid_controller_reset(void);
void pid_controller_run(uint32_t now_ms);
void pid_controller_set_target(int32_t temperature_c);
void pid_controller_update_measurements(int32_t cabinet_temp_c, int32_t evaporator_temp_c);
int32_t pid_controller_get_output(void);
bool pid_controller_is_saturated(void);

#else

static inline void pid_controller_init(void) {}
static inline void pid_controller_reset(void) {}
static inline void pid_controller_run(uint32_t now_ms) {(void)now_ms;}
static inline void pid_controller_set_target(int32_t temperature_c)
{
    (void)temperature_c;
}
static inline void pid_controller_update_measurements(int32_t cabinet_temp_c, int32_t evaporator_temp_c)
{
    (void)cabinet_temp_c;
    (void)evaporator_temp_c;
}
static inline int32_t pid_controller_get_output(void)
{
    return 0;
}
static inline bool pid_controller_is_saturated(void)
{
    return false;
}

#endif /* ENABLE_CONTROL_FSM */

#endif /* PID_CONTROLLER_H */
