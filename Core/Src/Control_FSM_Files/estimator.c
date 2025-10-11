/**
 * @file estimator.c
 * @brief Placeholder estimator module for the control finite state machine.
 *
 * The estimator will eventually fuse raw telemetry into stable cabinet and
 * evaporator temperature estimates that downstream controllers can consume.
 * Until the control FSM is enabled the functions below remain inert so that
 * the rest of the firmware can link successfully.
 */

#include "build_config.h"
#include <stdint.h>
#include <stdbool.h>

#include "Control_FSM_Files/estimator.h"

#if ENABLE_CONTROL_FSM

void estimator_init(void)
{
    // TODO: Implement estimator initialisation when ENABLE_CONTROL_FSM is set.
}

void estimator_reset(void)
{
    // TODO: Implement estimator reset logic when ENABLE_CONTROL_FSM is set.
}

void estimator_update(uint32_t now_ms)
{
    (void)now_ms;
    // TODO: Implement estimator update routine when ENABLE_CONTROL_FSM is set.
}

int32_t estimator_get_cabinet_temperature_c(void)
{
    // TODO: Return the latest cabinet temperature estimate when available.
    return 0;
}

int32_t estimator_get_evaporator_temperature_c(void)
{
    // TODO: Return the latest evaporator temperature estimate when available.
    return 0;
}

bool estimator_has_valid_reading(void)
{
    // TODO: Report estimator validity once the estimator is implemented.
    return false;
}

#else /* ENABLE_CONTROL_FSM */

void estimator_init(void)
{
    /* Estimator inactive while the control FSM is disabled. */
}

void estimator_reset(void)
{
    /* Nothing to reset while the estimator is inactive. */
}

void estimator_update(uint32_t now_ms)
{
    (void)now_ms;
    /* Updates are ignored when the control FSM is disabled. */
}

int32_t estimator_get_cabinet_temperature_c(void)
{
    return 0;
}

int32_t estimator_get_evaporator_temperature_c(void)
{
    return 0;
}

bool estimator_has_valid_reading(void)
{
    return false;
}

#endif /* ENABLE_CONTROL_FSM */
