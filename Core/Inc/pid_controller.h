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

typedef enum
{
    PID_CONTROLLER_MODE_COLD_DRINK = 0,
    PID_CONTROLLER_MODE_LIGHT_SLUSH,
    PID_CONTROLLER_MODE_MEDIUM_SLUSH,
    PID_CONTROLLER_MODE_HEAVY_SLUSH,
    PID_CONTROLLER_MODE_COUNT
} pid_controller_mode_t;

typedef struct
{
    float temperature_c;
    float texture_index;
} pid_controller_targets_t;

typedef struct
{
    float compressor_frequency_hz;
    float motor_rpm;
} pid_controller_output_t;

void pid_controller_init(void);
void pid_controller_reset(void);
void pid_controller_run(uint32_t now_ms);
void pid_controller_set_mode(pid_controller_mode_t mode);
void pid_controller_set_targets(const pid_controller_targets_t *targets);
void pid_controller_set_target(int32_t temperature_c);
void pid_controller_set_texture_target(int32_t texture_permille);
void pid_controller_update_state(float bowl_temp_c, float texture_index, float volume_fraction);
void pid_controller_update_measurements(int32_t cabinet_temp_c, int32_t evaporator_temp_c);
pid_controller_mode_t pid_controller_get_current_mode(void);
pid_controller_output_t pid_controller_get_output_struct(void);
int32_t pid_controller_get_output(void);
float pid_controller_get_motor_speed(void);
bool pid_controller_is_saturated(void);

#else

static inline void pid_controller_init(void) {}
static inline void pid_controller_reset(void) {}
static inline void pid_controller_run(uint32_t now_ms) {(void)now_ms;}
static inline void pid_controller_set_mode(pid_controller_mode_t mode)
{
    (void)mode;
}
static inline void pid_controller_set_targets(const pid_controller_targets_t *targets)
{
    (void)targets;
}
static inline void pid_controller_set_target(int32_t temperature_c)
{
    (void)temperature_c;
}
static inline void pid_controller_set_texture_target(int32_t texture_permille)
{
    (void)texture_permille;
}
static inline void pid_controller_update_state(float bowl_temp_c, float texture_index, float volume_fraction)
{
    (void)bowl_temp_c;
    (void)texture_index;
    (void)volume_fraction;
}
static inline void pid_controller_update_measurements(int32_t cabinet_temp_c, int32_t evaporator_temp_c)
{
    (void)cabinet_temp_c;
    (void)evaporator_temp_c;
}
static inline pid_controller_mode_t pid_controller_get_current_mode(void)
{
    return PID_CONTROLLER_MODE_COLD_DRINK;
}
static inline pid_controller_output_t pid_controller_get_output_struct(void)
{
    pid_controller_output_t output = {0.0f, 0.0f};
    return output;
}
static inline int32_t pid_controller_get_output(void)
{
    return 0;
}
static inline float pid_controller_get_motor_speed(void)
{
    return 0.0f;
}
static inline bool pid_controller_is_saturated(void)
{
    return false;
}

#endif /* ENABLE_CONTROL_FSM */

#endif /* PID_CONTROLLER_H */
