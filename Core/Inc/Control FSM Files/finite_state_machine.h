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
    CONTROL_FSM_STATE_POWER_ON = 0,
    CONTROL_FSM_STATE_IDLE,
    CONTROL_FSM_STATE_ACTIVE,
    CONTROL_FSM_STATE_DEFROST,
    CONTROL_FSM_STATE_FAULT,
} control_fsm_state_t;

void control_fsm_init(void);
void control_fsm_reset(void);
void control_fsm_run_tick(uint32_t now_ms);
void control_fsm_set_target_temperature(int32_t target_c);
void control_fsm_request_state(control_fsm_state_t state);
control_fsm_state_t control_fsm_get_state(void);
bool control_fsm_is_transition_pending(void);

#else

typedef enum
{
    CONTROL_FSM_STATE_POWER_ON = 0,
    CONTROL_FSM_STATE_IDLE,
    CONTROL_FSM_STATE_ACTIVE,
    CONTROL_FSM_STATE_DEFROST,
    CONTROL_FSM_STATE_FAULT,
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

#endif /* ENABLE_CONTROL_FSM */

#endif /* FINITE_STATE_MACHINE_H */
