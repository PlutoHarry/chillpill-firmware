#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "build_config.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @file estimator.h
 * @brief Sensor fusion and state estimation hooks for the control FSM.
 *
 * The estimator refines raw telemetry from the hardware abstraction layer and
 * exposes stable cabinet and evaporator temperature readings for the
 * high-level state machine. The module interacts with the PID controller to
 * provide feed-forward context and with the fault handler to report invalid
 * samples. Only the public APIs declared below should be used by other
 * components.
 */

#if ENABLE_CONTROL_FSM

void estimator_init(void);
void estimator_reset(void);
void estimator_update(uint32_t now_ms);
int32_t estimator_get_cabinet_temperature_c(void);
int32_t estimator_get_evaporator_temperature_c(void);
bool estimator_has_valid_reading(void);

#else

static inline void estimator_init(void) {}
static inline void estimator_reset(void) {}
static inline void estimator_update(uint32_t now_ms) {(void)now_ms;}
static inline int32_t estimator_get_cabinet_temperature_c(void) { return 0; }
static inline int32_t estimator_get_evaporator_temperature_c(void) { return 0; }
static inline bool estimator_has_valid_reading(void) { return false; }

#endif /* ENABLE_CONTROL_FSM */

#endif /* ESTIMATOR_H */
