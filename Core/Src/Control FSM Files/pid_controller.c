/**
 * @file pid_controller.c
 * @brief Placeholder compressor PID regulator for the control FSM.
 *
 * The production version will close the loop around cabinet temperature and
 * deliver compressor drive requests. The stubbed functions satisfy linker
 * requirements until the full PID logic is implemented.
 */

#include "build_config.h"
#include <stdint.h>
#include <stdbool.h>

#include "Control FSM Files/pid_controller.h"

#if ENABLE_CONTROL_FSM

static int32_t s_last_output = 0;
static bool s_is_saturated = false;

void pid_controller_init(void)
{
    // TODO: Initialise PID terms when ENABLE_CONTROL_FSM is set.
    s_last_output = 0;
    s_is_saturated = false;
}

void pid_controller_reset(void)
{
    // TODO: Reset accumulated PID state when ENABLE_CONTROL_FSM is set.
    s_last_output = 0;
    s_is_saturated = false;
}

void pid_controller_run(uint32_t now_ms)
{
    (void)now_ms;
    // TODO: Update PID output when ENABLE_CONTROL_FSM is set.
}

void pid_controller_set_target(int32_t temperature_c)
{
    (void)temperature_c;
    // TODO: Store target set-point when ENABLE_CONTROL_FSM is set.
}

void pid_controller_update_measurements(int32_t cabinet_temp_c, int32_t evaporator_temp_c)
{
    (void)cabinet_temp_c;
    (void)evaporator_temp_c;
    // TODO: Process measurement inputs when ENABLE_CONTROL_FSM is set.
}

int32_t pid_controller_get_output(void)
{
    // TODO: Return computed PID output when ENABLE_CONTROL_FSM is set.
    return s_last_output;
}

bool pid_controller_is_saturated(void)
{
    // TODO: Report saturation status when ENABLE_CONTROL_FSM is set.
    return s_is_saturated;
}

#else /* ENABLE_CONTROL_FSM */

void pid_controller_init(void)
{
}

void pid_controller_reset(void)
{
}

void pid_controller_run(uint32_t now_ms)
{
    (void)now_ms;
}

void pid_controller_set_target(int32_t temperature_c)
{
    (void)temperature_c;
}

void pid_controller_update_measurements(int32_t cabinet_temp_c, int32_t evaporator_temp_c)
{
    (void)cabinet_temp_c;
    (void)evaporator_temp_c;
}

int32_t pid_controller_get_output(void)
{
    return 0;
}

bool pid_controller_is_saturated(void)
{
    return false;
}

#endif /* ENABLE_CONTROL_FSM */
