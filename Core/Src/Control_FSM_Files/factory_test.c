/*
 * factory_test.c
 *
 * Implements a simple factory test routine for the slush machine.
 * The test exercises the auger motor, condenser fans, temperature
 * sensors and cooling system.  It runs a sequence of stages and
 * verifies that measured parameters fall within expected ranges.
 * During the test the front RGB LED is set to a test colour
 * (light blue/purple).  Upon completion the LED shows green for
 * pass or red for fail.  The ring LED remains off during the test.
 */

#include "factory_test.h"
#include "actuators.h"
#include "control_config.h"
#include "sensors.h"
#include "error_handler.h"
#include "stm32f1xx_hal.h"
#include <stdbool.h>

/* Internal state machine for the factory test */
typedef enum {
    FT_STATE_IDLE = 0,
    FT_STATE_ACTUATORS,
    FT_STATE_SENSORS,
    FT_STATE_COOLING,
    FT_STATE_PASS,
    FT_STATE_FAIL
} ft_state_t;

static ft_state_t  ft_state           = FT_STATE_IDLE;
static uint32_t    ft_state_start_ms  = 0U;
static bool        ft_running         = false;
static bool        ft_pass            = false;
/* Record initial evaporator temperatures at the start of the cooling test */
static float       t_bottom_start     = 0.0f;
static float       t_top_start        = 0.0f;

/* Helper to stop all actuators and ensure pulse mode is off */
static void factory_test_stop_all(void)
{
    control_stop_compressor();
    control_stop_motor();
    control_stop_fan();
    control_set_led_pulse(false);
    /* Turn off ring LED brightness */
    control_set_led_brightness(0.0f);
}

void factory_test_init(void)
{
    ft_state          = FT_STATE_IDLE;
    ft_state_start_ms = 0U;
    ft_running        = false;
    ft_pass           = false;
    t_bottom_start    = 0.0f;
    t_top_start       = 0.0f;
}

void factory_test_start(uint32_t now_ms)
{
    ft_state          = FT_STATE_ACTUATORS;
    ft_state_start_ms = now_ms;
    ft_running        = true;
    ft_pass           = false;
    /* Stop any ongoing actuation and reset LED states */
    factory_test_stop_all();
    /* Use light blue to approximate purple for the test indicator */
    freeze_rgb_led_set_color(FREEZE_RGB_LED_LIGHTBLUE);
}

bool factory_test_is_running(void)
{
    return ft_running;
}

bool factory_test_is_finished(void)
{
    return (ft_state == FT_STATE_PASS || ft_state == FT_STATE_FAIL);
}

bool factory_test_passed(void)
{
    return ft_pass && factory_test_is_finished();
}

void factory_test_run(uint32_t now_ms)
{
    if (!ft_running) return;

    /* Access configuration via global g_control_config */
    extern control_config_t g_control_config;
    /* Access sensor arrays defined in sensor_ntc_adc.c */
    extern float temperature_ntc[3];
    extern float motor_current;
    extern volatile float motor_rpm;
    /* Access fan alive check from pwm driver */
    extern bool fan_check_if_alive(void);

    switch (ft_state) {
    case FT_STATE_ACTUATORS:
        /* Drive the auger at test speed and run fans at full duty. */
        control_set_motor_speed(g_control_config.deice_motor_speed);
        control_set_fan_speed(100);
        /* Compressor remains off during actuator test */
        control_stop_compressor();
        /* Wait until the configured actuator test duration has elapsed */
        if ((now_ms - ft_state_start_ms) >= g_control_config.factory_test_actuator_test_duration_ms) {
            /* Evaluate RPM and current ranges */
            bool ok = true;
            if (motor_rpm < g_control_config.factory_test_motor_rpm_min ||
                motor_rpm > g_control_config.factory_test_motor_rpm_max) {
                ok = false;
            }
            if (motor_current < g_control_config.factory_test_motor_current_min ||
                motor_current > g_control_config.factory_test_motor_current_max) {
                ok = false;
            }
            /* Evaluate fan health using the low level helper. */
            if (!fan_check_if_alive()) {
                ok = false;
            }
            if (!ok) {
                ft_state = FT_STATE_FAIL;
                ft_running = false;
                break;
            }
            /* Proceed to sensor test */
            ft_state = FT_STATE_SENSORS;
            ft_state_start_ms = now_ms;
        }
        break;
    case FT_STATE_SENSORS:
        /* Sensor test: verify evaporator sensors are within ambient range. */
        {
            bool ok = true;
            /* Check bottom and top sensors only; ignore bowl sensor. */
            float t0 = temperature_ntc[0];
            float t1 = temperature_ntc[1];
            if (t0 < g_control_config.factory_test_temp_min || t0 > g_control_config.factory_test_temp_max) {
                ok = false;
            }
            if (t1 < g_control_config.factory_test_temp_min || t1 > g_control_config.factory_test_temp_max) {
                ok = false;
            }
            if (!ok) {
                ft_state = FT_STATE_FAIL;
                ft_running = false;
                break;
            }
        }
        /* Proceed immediately to cooling test */
        ft_state = FT_STATE_COOLING;
        ft_state_start_ms = now_ms;
        /* Record initial temperatures for drop measurement */
        t_bottom_start = temperature_ntc[0];
        t_top_start    = temperature_ntc[1];
        /* Start high‑speed cooling stage */
        break;
    case FT_STATE_COOLING:
        {
            uint32_t elapsed = now_ms - ft_state_start_ms;
            uint32_t stage1 = g_control_config.factory_test_stage1_duration_ms;
            uint32_t stage2 = g_control_config.factory_test_stage2_duration_ms;
            uint32_t stage3 = g_control_config.factory_test_stage3_duration_ms;
            uint32_t stage4 = g_control_config.factory_test_stage4_duration_ms;
            if (elapsed < stage1) {
                /* Full speed */
                control_set_compressor_frequency((uint8_t)g_control_config.pull_down_max_freq_initial);
            } else if (elapsed < (stage1 + stage2)) {
                /* Off */
                control_stop_compressor();
            } else if (elapsed < (stage1 + stage2 + stage3)) {
                /* Half speed */
                control_set_compressor_frequency((uint8_t)(g_control_config.pull_down_max_freq_initial / 2));
            } else if (elapsed < (stage1 + stage2 + stage3 + stage4)) {
                /* Minimum speed */
                control_set_compressor_frequency((uint8_t)g_control_config.pull_down_min_freq);
            } else {
                /* Cooling test complete – evaluate temperature drop */
                float t_btm = temperature_ntc[0];
                float t_top = temperature_ntc[1];
                float drop_btm = t_bottom_start - t_btm;
                float drop_top = t_top_start - t_top;
                bool ok = (drop_btm >= g_control_config.factory_test_temp_drop_threshold &&
                           drop_top >= g_control_config.factory_test_temp_drop_threshold);
                /* Stop compressor and fans */
                factory_test_stop_all();
                if (ok) {
                    ft_state = FT_STATE_PASS;
                    ft_running = false;
                    ft_pass = true;
                } else {
                    ft_state = FT_STATE_FAIL;
                    ft_running = false;
                }
            }
        }
        break;
    case FT_STATE_PASS:
        /* Indicate pass with green LED; remain in pass state */
        freeze_rgb_led_set_color(FREEZE_RGB_LED_GREEN);
        break;
    case FT_STATE_FAIL:
        /* Indicate fail with red LED; remain in fail state */
        freeze_rgb_led_set_color(FREEZE_RGB_LED_RED);
        break;
    default:
        break;
    }
}
