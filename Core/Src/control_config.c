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
    /* Maximum compressor frequency during pull‑down (Hz).  Start
     * aggressively to reach slush temperature quickly. */
    .pull_down_max_freq_initial  = 150.0f,
    /* Minimum compressor frequency allowed during pull‑down (Hz).  Do
     * not drive the compressor below this value to avoid poor
     * lubrication and stalling. */
    .pull_down_min_freq          = 45.0f,
    /* Amount to reduce the maximum compressor frequency after each
     * icing event during pull‑down (Hz). */
    .pull_down_freq_step         = 10.0f,
    /* Evaporator delta threshold for declaring an icing condition
     * (°C).  A high positive delta indicates poor heat transfer due
     * to ice build‑up. */
    .icing_delta_threshold       = 10.0f,
    /* Evaporator delta threshold for exiting an icing condition
     * (°C).  Use hysteresis to prevent rapid flapping. */
    .icing_delta_exit_threshold  = 5.0f,
    /* Bowl cooling rate (|°C/s|) above which we assume the bowl is
     * relatively empty.  Used by the volume estimator. */
    .volume_low_slope_threshold  = 2.0f,
    /* Bowl cooling rate (|°C/s|) above which we assume a medium
     * fill.  Below this the bowl is considered full. */
    .volume_med_slope_threshold  = 0.5f,
    /* Interval in milliseconds after which the volume estimate is
     * re‑evaluated.  This should be long enough to avoid noise but
     * short enough to track slow changes due to consumption or
     * refilling. */
    .volume_update_interval_ms   = 10000U, /* 10 seconds */
    /* Force a de‑icing cycle every this many milliseconds even if
     * conditions are only moderate.  Regular de‑icing keeps the
     * torque estimator calibrated. */
    .periodic_deice_interval_ms  = 600000U, /* 10 minutes */
    /* Run the condenser fans for this many milliseconds after the
     * user powers down when the condenser is hot. */
    .shutdown_fan_runtime_ms     = 90000U, /* 90 seconds */
    /* Evaporator delta at which the condenser is considered hot (°C). */
    .condenser_hot_delta_threshold = 8.0f,
    /* Evaporator bottom temperature above which the condenser is
     * considered hot (°C). */
    .condenser_hot_temp_threshold = 30.0f

    ,/* Ready indicator: flash 10 times with 250 ms on/off cycles */
    .ready_indicator_flash_count      = 10,
    .ready_indicator_flash_period_ms  = 500U,

    /* Default de‑ice auger speed (RPM).  Keep the auger moving
     * during de‑icing without splashing. */
    .deice_motor_speed               = 25.0f,

    /* Long‑press thresholds for the light button */
    .led_long_press_2s_ms           = 2000U,
    .led_long_press_5s_ms           = 5000U,

    /* Service entry hold: hold light+freeze buttons for 5 seconds */
    .service_entry_hold_ms          = 5000U,

    /* Service indicator timing: 300 ms on, 300 ms off, 1 s between groups */
    .service_flash_on_ms            = 300U,
    .service_flash_off_ms           = 300U,
    .service_group_pause_ms         = 1000U

    ,/* Cold drink mode defaults.  For the +4 °C mode we limit the
     * compressor frequency during pull‑down and run the auger more
     * slowly since no slush formation is required. */
    .cold_pull_down_max_freq        = 60.0f,
    .cold_pull_down_motor_speed     = 10.0f,

    /* Cold drink temperature setpoint (°C).  Maintain the bowl at
     * approximately 4 degrees for cold beverages without forming
     * slush. */
    .cold_drink_temp_setpoint       = 4.0f,

    /* Target texture indices for slush modes.  These values were
     * determined experimentally for light, medium and heavy slush
     * consistencies.  Increase for thicker slush. */
    .texture_target_light           = 0.30f,
    .texture_target_medium          = 0.60f,
    .texture_target_heavy           = 0.90f,

    /* Temperature targets for the corresponding slush modes (°C). */
    .freeze_temp_light_setpoint     = -1.5f,
    .freeze_temp_medium_setpoint    = -2.5f,
    .freeze_temp_heavy_setpoint     = -3.5f,

    /* PID tolerances: small bands around the setpoints within which
     * the compressor frequency is held constant. */
    .pid_temp_band                  = 0.5f,
    .pid_texture_band               = 0.05f,

    /* PID frequency step (Hz) and steady state frequency limits. */
    .pid_freq_step                  = 2.0f,
    .steady_min_freq                = 45.0f,
    .steady_max_freq                = 150.0f,

    /* PID gains and anti-windup configuration. */
    .pid_temp_kp                    = 3.0f,
    .pid_temp_ki                    = 0.08f,
    .pid_texture_kp                 = 18.0f,
    .pid_texture_ki                 = 0.6f,
    .pid_integrator_limit           = 45.0f,

    /* Volume-event handling (fractional change of bowl volume). */
    .pid_volume_dispense_threshold  = 0.06f,
    .pid_volume_refill_threshold    = 0.08f,
    .pid_dispense_boost_hz          = 10.0f,
    .pid_dispense_boost_decay_ms    = 20000U,
    .pid_refill_boost_hz            = 6.0f,
    .pid_refill_hold_ms             = 40000U,

    /* Factory test defaults.  These values define the test
     * procedure executed when factory_test_enable is true. */
    .factory_test_enable               = false,
    .factory_test_actuator_test_duration_ms = 5000U,
    .factory_test_motor_rpm_min         = 20.0f,
    .factory_test_motor_rpm_max         = 40.0f,
    .factory_test_motor_current_min     = 0.2f,
    .factory_test_motor_current_max     = 3.0f,
    .factory_test_temp_min              = 10.0f,
    .factory_test_temp_max              = 40.0f,
    .factory_test_stage1_duration_ms    = 180000U, /* 3 minutes */
    .factory_test_stage2_duration_ms    = 60000U,  /* 1 minute off */
    .factory_test_stage3_duration_ms    = 120000U, /* 2 minutes half */
    .factory_test_stage4_duration_ms    = 60000U,  /* 1 minute low */
    .factory_test_temp_drop_threshold   = 5.0f
};
