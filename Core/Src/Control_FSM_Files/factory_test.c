/**
 * @file factory_test.c
 * @brief Placeholder factory test coordinator for the control FSM.
 *
 * The final implementation will orchestrate the production diagnostics
 * sequence that validates sensors and actuators. While the control FSM remains
 * disabled the functions below are inert so the firmware continues to link
 * cleanly.
 */

#include "build_config.h"
#include <stdint.h>
#include <stdbool.h>

#include "Control_FSM_Files/factory_test.h"

#if ENABLE_CONTROL_FSM

void factory_test_init(void)
{
    // TODO: Initialise factory test state when ENABLE_CONTROL_FSM is set.
}

void factory_test_run(uint32_t now_ms)
{
    (void)now_ms;
    // TODO: Execute factory test sequencing when ENABLE_CONTROL_FSM is set.
}

void factory_test_request_start(void)
{
    // TODO: Start the factory test routine when ENABLE_CONTROL_FSM is set.
}

void factory_test_request_stop(void)
{
    // TODO: Stop the factory test routine when ENABLE_CONTROL_FSM is set.
}

bool factory_test_is_active(void)
{
    // TODO: Report the active state once the factory test is implemented.
    return false;
}

bool factory_test_has_failed(void)
{
    // TODO: Report failure state once the factory test is implemented.
    return false;
}

#else /* ENABLE_CONTROL_FSM */

void factory_test_init(void)
{
    /* Factory test support is disabled when the control FSM is inactive. */
}

void factory_test_run(uint32_t now_ms)
{
    (void)now_ms;
    /* No diagnostics are executed while the control FSM is disabled. */
}

void factory_test_request_start(void)
{
    /* Ignore start requests while diagnostics are disabled. */
}

void factory_test_request_stop(void)
{
    /* Ignore stop requests while diagnostics are disabled. */
}

bool factory_test_is_active(void)
{
    return false;
}

bool factory_test_has_failed(void)
{
    return false;
}

#endif /* ENABLE_CONTROL_FSM */
