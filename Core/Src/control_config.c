/*
 * control_config.c
 *
 * Default configuration values for the slush machine controller.
 *
 * This module defines the global g_control_config structure declared
 * in control_config.h.  Values here are initialised with sensible
 * defaults but may be overridden at runtime.  Consolidating all
 * tunable parameters in one place makes it easy to adjust the
 * controller without hunting through the codebase.  End users
 * experimenting with different liquids or ambient conditions can
 * tweak these values to achieve the desired performance.
 */

#include "control_config.h"

/* Global configuration instance.  Modify these values before
 * compilation or at runtime (if exposed via a UI) to tune the
 * controller. */
control_config_t g_control_config = {
    /* --------------------------------------------------------------------- */
    /* Pull-down and icing heuristics                                       */
    /* --------------------------------------------------------------------- */
    .pull_down_max_freq_initial   = 150.0f,  /* Hz, aggressive first pull-down */
    .pull_down_min_freq           = 45.0f,   /* Hz, protect compressor */
    .pull_down_freq_step          = 10.0f,   /* Hz reduction per icing event */
    .pull_down_temp_margin_c      = 0.4f,    /* °C margin for declaring ready */
    .pull_down_texture_margin     = 0.08f,   /* Texture index hysteresis */
    .icing_delta_threshold        = 10.0f,   /* °C delta to call icing */
    .icing_delta_exit_threshold   = 5.0f,    /* °C delta to exit icing */

    /* --------------------------------------------------------------------- */
    /* Volume estimator                                                      */
    /* --------------------------------------------------------------------- */
    .volume_low_slope_threshold   = 2.0f,    /* |°C/s| -> empty */
    .volume_med_slope_threshold   = 0.5f,    /* |°C/s| -> full */
    .volume_update_interval_ms    = 10000U,  /* 10 s update cadence */
    .periodic_deice_interval_ms   = 600000U, /* 10 min watchdog */
    .shutdown_fan_runtime_ms      = 90000U,  /* 90 s cool-down */
    .condenser_hot_delta_threshold = 8.0f,   /* °C */
    .condenser_hot_temp_threshold  = 30.0f,  /* °C */

    /* --------------------------------------------------------------------- */
    /* Indicators and UI timing                                              */
    /* --------------------------------------------------------------------- */
    .ready_indicator_flash_count     = 10,
    .ready_indicator_flash_period_ms = 500U,
    .deice_motor_speed               = 25.0f, /* RPM during dedicated de-ice */
    .led_long_press_2s_ms            = 2000U,
    .led_long_press_5s_ms            = 5000U,
    .service_entry_hold_ms           = 5000U,
    .service_flash_on_ms             = 300U,
    .service_flash_off_ms            = 300U,
    .service_group_pause_ms          = 1000U,

    /* --------------------------------------------------------------------- */
    /* Mode specific profiles                                                */
    /* --------------------------------------------------------------------- */
    .mode_profiles = {
        [CONTROL_MODE_COLD_DRINK] = {
            .freeze_temp_c          = 4.0f,
            .texture_target         = 0.0f,
            .compressor_min_freq_hz = 45.0f,
            .compressor_max_freq_hz = 65.0f,
            .auger_rpm              = 10.0f,
            .deice_enter_ice_index  = 1.0f,   /* effectively disabled */
            .deice_exit_ice_index   = 0.0f,
            .indicator_rgb          = { .r = 0.0f, .g = 0.7f, .b = 0.2f },
        },
        [CONTROL_MODE_LIGHT_SLUSH] = {
            .freeze_temp_c          = -1.5f,
            .texture_target         = 0.30f,
            .compressor_min_freq_hz = 45.0f,
            .compressor_max_freq_hz = 140.0f,
            .auger_rpm              = 22.0f,
            .deice_enter_ice_index  = 0.55f,
            .deice_exit_ice_index   = 0.30f,
            .indicator_rgb          = { .r = 0.2f, .g = 0.7f, .b = 1.0f },
        },
        [CONTROL_MODE_MEDIUM_SLUSH] = {
            .freeze_temp_c          = -2.5f,
            .texture_target         = 0.60f,
            .compressor_min_freq_hz = 45.0f,
            .compressor_max_freq_hz = 150.0f,
            .auger_rpm              = 25.0f,
            .deice_enter_ice_index  = 0.60f,
            .deice_exit_ice_index   = 0.35f,
            .indicator_rgb          = { .r = 0.1f, .g = 0.4f, .b = 1.0f },
        },
        [CONTROL_MODE_HEAVY_SLUSH] = {
            .freeze_temp_c          = -3.5f,
            .texture_target         = 0.90f,
            .compressor_min_freq_hz = 45.0f,
            .compressor_max_freq_hz = 150.0f,
            .auger_rpm              = 28.0f,
            .deice_enter_ice_index  = 0.65f,
            .deice_exit_ice_index   = 0.40f,
            .indicator_rgb          = { .r = 0.8f, .g = 0.1f, .b = 0.9f },
        },
    },

    /* --------------------------------------------------------------------- */
    /* PID controller tuning                                                 */
    /* --------------------------------------------------------------------- */
    .pid_temp_band                 = 0.5f,
    .pid_texture_band              = 0.05f,
    .pid_freq_step                 = 2.0f,
    .pid_temp_kp                   = 3.0f,
    .pid_temp_ki                   = 0.08f,
    .pid_texture_kp                = 18.0f,
    .pid_texture_ki                = 0.6f,
    .pid_integrator_limit          = 45.0f,
    .pid_texture_motor_gain        = 10.0f,
    .pid_texture_motor_limit_rpm   = 5.0f,
    .pid_motor_rpm_min             = 0.0f,
    .pid_motor_rpm_max             = 120.0f,
    .pid_volume_dispense_threshold = 0.06f,
    .pid_volume_refill_threshold   = 0.08f,
    .pid_dispense_boost_hz         = 10.0f,
    .pid_dispense_boost_decay_ms   = 20000U,
    .pid_refill_boost_hz           = 6.0f,
    .pid_refill_hold_ms            = 40000U,

    /* --------------------------------------------------------------------- */
    /* Estimator tuning                                                      */
    /* --------------------------------------------------------------------- */
    .estimator_torque_baseline_floor   = 0.2f,
    .estimator_torque_norm_scale       = 1.2f,
    .estimator_texture_lowpass_hz      = 2.5f,
    .estimator_texture_torque_weight   = 0.6f,
    .estimator_texture_freeze_weight   = 0.4f,
    .estimator_ice_lowpass_hz          = 5.0f,
    .estimator_ice_delta_weight        = 0.65f,
    .estimator_ice_torque_weight       = 0.35f,
    .estimator_ice_detect_ms           = 4000U,
    .estimator_ice_clear_ms            = 3000U,
    .estimator_volume_texture_factor   = 0.15f,

    /* --------------------------------------------------------------------- */
    /* Factory test defaults                                                 */
    /* --------------------------------------------------------------------- */
    .factory_test_enable                = false,
    .factory_test_actuator_test_duration_ms = 5000U,
    .factory_test_motor_rpm_min          = 20.0f,
    .factory_test_motor_rpm_max          = 40.0f,
    .factory_test_motor_current_min      = 0.2f,
    .factory_test_motor_current_max      = 3.0f,
    .factory_test_temp_min               = 10.0f,
    .factory_test_temp_max               = 40.0f,
    .factory_test_stage1_duration_ms     = 180000U, /* 3 minutes */
    .factory_test_stage2_duration_ms     = 60000U,  /* 1 minute off */
    .factory_test_stage3_duration_ms     = 120000U, /* 2 minutes half */
    .factory_test_stage4_duration_ms     = 60000U,  /* 1 minute low */
    .factory_test_temp_drop_threshold    = 5.0f,
};
