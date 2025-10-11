/**
 * @file fault_handler.c
 * @brief Placeholder fault management for the control FSM.
 *
 * The final module will aggregate and prioritise hardware and software fault
 * signals for the control finite state machine. Stubbed behaviour keeps the
 * build healthy until the full implementation lands.
 */

#include "build_config.h"
#include <stdint.h>
#include <stdbool.h>

#include "Control FSM Files/fault_handler.h"

#if ENABLE_CONTROL_FSM

void fault_handler_init(void)
{
    // TODO: Initialise fault tracking structures when ENABLE_CONTROL_FSM is set.
}

void fault_handler_run(uint32_t now_ms)
{
    (void)now_ms;
    // TODO: Periodically evaluate fault conditions when ENABLE_CONTROL_FSM is set.
}

void fault_handler_report(fault_handler_fault_t fault)
{
    (void)fault;
    // TODO: Record reported faults when ENABLE_CONTROL_FSM is set.
}

void fault_handler_clear(fault_handler_fault_t fault)
{
    (void)fault;
    // TODO: Clear recorded faults when ENABLE_CONTROL_FSM is set.
}

void fault_handler_clear_all(void)
{
    // TODO: Clear all recorded faults when ENABLE_CONTROL_FSM is set.
}

bool fault_handler_is_active(fault_handler_fault_t fault)
{
    (void)fault;
    // TODO: Report active faults when ENABLE_CONTROL_FSM is set.
    return false;
}

fault_handler_fault_t fault_handler_get_highest_priority(void)
{
    // TODO: Return the highest priority active fault when ENABLE_CONTROL_FSM is set.
    return FAULT_HANDLER_FAULT_NONE;
}

#else /* ENABLE_CONTROL_FSM */

void fault_handler_init(void)
{
    /* Fault handling is inactive while the control FSM is disabled. */
}

void fault_handler_run(uint32_t now_ms)
{
    (void)now_ms;
    /* No periodic evaluation required when the control FSM is disabled. */
}

void fault_handler_report(fault_handler_fault_t fault)
{
    (void)fault;
}

void fault_handler_clear(fault_handler_fault_t fault)
{
    (void)fault;
}

void fault_handler_clear_all(void)
{
}

bool fault_handler_is_active(fault_handler_fault_t fault)
{
    (void)fault;
    return false;
}

fault_handler_fault_t fault_handler_get_highest_priority(void)
{
    return FAULT_HANDLER_FAULT_NONE;
}

#endif /* ENABLE_CONTROL_FSM */
