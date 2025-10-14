#ifndef FAULT_HANDLER_H
#define FAULT_HANDLER_H

#include "build_config.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @file fault_handler.h
 * @brief Alarm aggregation and mitigation services for the control FSM.
 *
 * The fault handler tracks abnormal operating conditions detected by the
 * estimator, PID controller, and hardware monitors. It surfaces actionable
 * fault codes to the finite state machine so recovery and safe shutdown
 * strategies can be applied.
 */

#if ENABLE_CONTROL_FSM

typedef enum
{
    FAULT_HANDLER_FAULT_NONE = 0,
    FAULT_HANDLER_FAULT_SENSOR,
    FAULT_HANDLER_FAULT_ACTUATOR,
    FAULT_HANDLER_FAULT_OVERTEMP,
    FAULT_HANDLER_FAULT_UNKNOWN
} fault_handler_fault_t;

void fault_handler_init(void);
void fault_handler_run(uint32_t now_ms);
void fault_handler_report(fault_handler_fault_t fault);
void fault_handler_clear(fault_handler_fault_t fault);
void fault_handler_clear_all(void);
bool fault_handler_is_active(fault_handler_fault_t fault);
fault_handler_fault_t fault_handler_get_highest_priority(void);

#else

typedef enum
{
    FAULT_HANDLER_FAULT_NONE = 0,
    FAULT_HANDLER_FAULT_SENSOR,
    FAULT_HANDLER_FAULT_ACTUATOR,
    FAULT_HANDLER_FAULT_OVERTEMP,
    FAULT_HANDLER_FAULT_UNKNOWN
} fault_handler_fault_t;

static inline void fault_handler_init(void) {}
static inline void fault_handler_run(uint32_t now_ms) {(void)now_ms;}
static inline void fault_handler_report(fault_handler_fault_t fault) {(void)fault;}
static inline void fault_handler_clear(fault_handler_fault_t fault) {(void)fault;}
static inline void fault_handler_clear_all(void) {}
static inline bool fault_handler_is_active(fault_handler_fault_t fault)
{
    (void)fault;
    return false;
}
static inline fault_handler_fault_t fault_handler_get_highest_priority(void)
{
    return FAULT_HANDLER_FAULT_NONE;
}

#endif /* ENABLE_CONTROL_FSM */

#endif /* FAULT_HANDLER_H */
