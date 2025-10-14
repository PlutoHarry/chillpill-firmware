#ifndef FINITE_STATE_MACHINE_H
#define FINITE_STATE_MACHINE_H

#include "build_config.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @file finite_state_machine.h
 * @brief Top-level coordinator for the refrigeration control finite state machine.
 *
 * The FSM sequences through power-up, active cooling, defrost, and fault
 * handling states based on feedback from the estimator, PID controller, and
 * fault handler modules. It owns the scheduler-facing entry points that other
 * application tasks will call.
 */

#if ENABLE_CONTROL_FSM

typedef enum
{
    CONTROL_FSM_STATE_POWER_ON = 0, /**< Hardware has just powered on. */
    CONTROL_FSM_STATE_STARTUP,      /**< Performing initial settling. */
    CONTROL_FSM_STATE_PULL_DOWN,    /**< Aggressive initial cooling. */
    CONTROL_FSM_STATE_STEADY,       /**< Normal regulated operation. */
    CONTROL_FSM_STATE_DEICING,      /**< Dedicated de-icing routine. */
    CONTROL_FSM_STATE_SHUTDOWN,     /**< Controlled shutdown. */
    CONTROL_FSM_STATE_FAULT,        /**< Latched safety fault. */
    CONTROL_FSM_STATE_SERVICE,      /**< Service / diagnostics display. */
    CONTROL_FSM_STATE_FACTORY_TEST, /**< Automated factory test script. */
    CONTROL_FSM_STATE_DEBUG         /**< Manual debug override mode. */
} control_fsm_state_t;

void control_fsm_init(void);
void control_fsm_reset(void);
void control_fsm_run_tick(uint32_t now_ms);
void control_fsm_set_target_temperature(int32_t target_c);
void control_fsm_request_state(control_fsm_state_t state);
control_fsm_state_t control_fsm_get_state(void);
bool control_fsm_is_transition_pending(void);
void control_fsm_set_debug_mode(bool enabled);
bool control_fsm_is_debug_mode(void);

#else

typedef enum
{
    CONTROL_FSM_STATE_POWER_ON = 0,
    CONTROL_FSM_STATE_STARTUP,
    CONTROL_FSM_STATE_PULL_DOWN,
    CONTROL_FSM_STATE_STEADY,
    CONTROL_FSM_STATE_DEICING,
    CONTROL_FSM_STATE_SHUTDOWN,
    CONTROL_FSM_STATE_FAULT,
    CONTROL_FSM_STATE_SERVICE,
    CONTROL_FSM_STATE_FACTORY_TEST,
    CONTROL_FSM_STATE_DEBUG
} control_fsm_state_t;

static inline void control_fsm_init(void) {}
static inline void control_fsm_reset(void) {}
static inline void control_fsm_run_tick(uint32_t now_ms) {(void)now_ms;}
static inline void control_fsm_set_target_temperature(int32_t target_c)
{
    (void)target_c;
}
static inline void control_fsm_request_state(control_fsm_state_t state)
{
    (void)state;
}
static inline control_fsm_state_t control_fsm_get_state(void)
{
    return CONTROL_FSM_STATE_POWER_ON;
}
static inline bool control_fsm_is_transition_pending(void)
{
    return false;
}
static inline void control_fsm_set_debug_mode(bool enabled)
{
    (void)enabled;
}
static inline bool control_fsm_is_debug_mode(void)
{
    return false;
}

#endif /* ENABLE_CONTROL_FSM */

#endif /* FINITE_STATE_MACHINE_H */
