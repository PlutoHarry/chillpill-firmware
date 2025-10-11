/**
 * @file finite_state_machine.c
 * @brief Placeholder high-level control state machine wiring.
 *
 * This translation unit will eventually coordinate estimator, PID controller
 * and actuator modules. It is currently stubbed so firmware builds can
 * progress while the detailed logic is developed.
 */

#include "build_config.h"
#include <stdint.h>
#include <stdbool.h>

#include "Control_FSM_Files/finite_state_machine.h"

#if ENABLE_CONTROL_FSM

static control_fsm_state_t s_current_state = CONTROL_FSM_STATE_POWER_ON;
static bool s_transition_pending = false;

void control_fsm_init(void)
{
    // TODO: Implement control FSM initialisation when ENABLE_CONTROL_FSM is set.
    s_current_state = CONTROL_FSM_STATE_POWER_ON;
    s_transition_pending = false;
}

void control_fsm_reset(void)
{
    // TODO: Implement control FSM reset behaviour when ENABLE_CONTROL_FSM is set.
    s_current_state = CONTROL_FSM_STATE_POWER_ON;
    s_transition_pending = false;
}

void control_fsm_run_tick(uint32_t now_ms)
{
    (void)now_ms;
    // TODO: Implement periodic FSM processing when ENABLE_CONTROL_FSM is set.
}

void control_fsm_set_target_temperature(int32_t target_c)
{
    (void)target_c;
    // TODO: Handle target temperature updates when ENABLE_CONTROL_FSM is set.
}

void control_fsm_request_state(control_fsm_state_t state)
{
    // TODO: Queue state transitions when ENABLE_CONTROL_FSM is set.
    s_current_state = state;
    s_transition_pending = true;
}

control_fsm_state_t control_fsm_get_state(void)
{
    // TODO: Report the active FSM state when ENABLE_CONTROL_FSM is set.
    return s_current_state;
}

bool control_fsm_is_transition_pending(void)
{
    // TODO: Report transition activity when ENABLE_CONTROL_FSM is set.
    return s_transition_pending;
}

#else /* ENABLE_CONTROL_FSM */

void control_fsm_init(void)
{
}

void control_fsm_reset(void)
{
}

void control_fsm_run_tick(uint32_t now_ms)
{
    (void)now_ms;
}

void control_fsm_set_target_temperature(int32_t target_c)
{
    (void)target_c;
}

void control_fsm_request_state(control_fsm_state_t state)
{
    (void)state;
}

control_fsm_state_t control_fsm_get_state(void)
{
    return CONTROL_FSM_STATE_POWER_ON;
}

bool control_fsm_is_transition_pending(void)
{
    return false;
}

#endif /* ENABLE_CONTROL_FSM */
