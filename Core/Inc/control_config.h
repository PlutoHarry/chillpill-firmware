/**
 * @file control_config.h
 * @brief Shared configuration data for the control subsystem.
 *
 * The values defined in ::control_config_t provide tunable parameters for the
 * control finite state machine (FSM).  They are initialised with defaults in
 * control_config.c but may be overridden at runtime if the application exposes
 * a suitable interface.
 */
#ifndef CONTROL_CONFIG_H
#define CONTROL_CONFIG_H

#include "build_config.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration values that govern the behaviour of the control FSM.
 */
typedef struct {
    float     pull_down_max_freq_initial;
    float     pull_down_min_freq;
    float     pull_down_freq_step;
    float     icing_delta_threshold;
    float     icing_delta_exit_threshold;
    float     volume_low_slope_threshold;
    float     volume_med_slope_threshold;
    uint32_t  volume_update_interval_ms;
    uint32_t  periodic_deice_interval_ms;
    uint32_t  shutdown_fan_runtime_ms;
    float     condenser_hot_delta_threshold;
    float     condenser_hot_temp_threshold;
    uint32_t  ready_indicator_flash_count;
    uint32_t  ready_indicator_flash_period_ms;
    float     deice_motor_speed;
    uint32_t  led_long_press_2s_ms;
    uint32_t  led_long_press_5s_ms;
    uint32_t  service_entry_hold_ms;
    uint32_t  service_flash_on_ms;
    uint32_t  service_flash_off_ms;
    uint32_t  service_group_pause_ms;
    float     cold_pull_down_max_freq;
    float     cold_pull_down_motor_speed;
    float     cold_drink_temp_setpoint;
    float     texture_target_light;
    float     texture_target_medium;
    float     texture_target_heavy;
    float     pid_temp_band;
    float     pid_texture_band;
    float     pid_freq_step;
    float     steady_min_freq;
    float     steady_max_freq;
    bool      factory_test_enable;
    uint32_t  factory_test_actuator_test_duration_ms;
    float     factory_test_motor_rpm_min;
    float     factory_test_motor_rpm_max;
    float     factory_test_motor_current_min;
    float     factory_test_motor_current_max;
    float     factory_test_temp_min;
    float     factory_test_temp_max;
    uint32_t  factory_test_stage1_duration_ms;
    uint32_t  factory_test_stage2_duration_ms;
    uint32_t  factory_test_stage3_duration_ms;
    uint32_t  factory_test_stage4_duration_ms;
    float     factory_test_temp_drop_threshold;
} control_config_t;

/** Global control configuration instance defined in control_config.c. */
extern control_config_t g_control_config;

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_CONFIG_H */
