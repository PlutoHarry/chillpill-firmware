/*
 * factory_test.c
 *
 * Manufacturing diagnostics sequence for the ChillPill controller.
 * The routine exercises the user-visible lighting as well as the
 * condenser fans, auger motor and compressor.  Sensor feedback is
 * evaluated against the thresholds defined in control_config.c and a
 * pass/fail summary is exposed via factory_test_get_results().
 */

#include "factory_test.h"

#include "actuators.h"
#include "control_config.h"
#include "lights.h"
#include "sensors.h"

#include <stdbool.h>
#include <string.h>

/* Duration of the dedicated lighting exercise stage (ms). */
#define FACTORY_TEST_LIGHT_STAGE_MS 2000U

/* Internal state machine for the factory test */
typedef enum {
    FT_STATE_IDLE = 0,
    FT_STATE_LIGHTS,
    FT_STATE_ACTUATORS,
    FT_STATE_SENSORS,
    FT_STATE_COOLING,
    FT_STATE_PASS,
    FT_STATE_FAIL
} ft_state_t;

static ft_state_t             ft_state           = FT_STATE_IDLE;
static uint32_t               ft_state_start_ms  = 0U;
static bool                   ft_active          = false;
static bool                   ft_complete        = false;
static bool                   ft_start_requested = false;
static bool                   ft_stop_requested  = false;
static factory_test_results_t ft_results;

/* Forward declaration */
static void factory_test_transition(ft_state_t new_state, uint32_t now_ms);

extern control_config_t g_control_config;

/* -------------------------------------------------------------------------- */
/*                             Helper functions                               */
/* -------------------------------------------------------------------------- */

static void factory_test_reset_results(void)
{
    memset(&ft_results, 0, sizeof(ft_results));
}

static void factory_test_stop_all(void)
{
    set_compressor_speed(0U);
    set_motor_speed(0.0f);
    set_fan_speed(0U);
    set_ring_led_pulse_enable(false);
    set_ring_led_level_percent(0U);
    set_freeze_btn_color(FREEZE_BTN_OFF);
    set_light_btn_led(false);
    set_on_btn_led(false);
}

static uint16_t freq_hz_to_rpm(float freq_hz)
{
    if (freq_hz <= 0.0f) {
        return 0U;
    }

    float rpm = freq_hz * 60.0f;
    if (rpm < 0.0f) {
        rpm = 0.0f;
    }
    if (rpm > 6000.0f) {
        rpm = 6000.0f;
    }
    return (uint16_t)(rpm + 0.5f);
}

static void factory_test_enter_state(ft_state_t state, uint32_t now_ms)
{
    (void)now_ms;
    const control_config_t *cfg = &g_control_config;

    switch (state) {
    case FT_STATE_IDLE:
        factory_test_stop_all();
        ft_active   = false;
        ft_complete = false;
        break;

    case FT_STATE_LIGHTS:
        factory_test_stop_all();
        set_ring_led_level_percent(100U);
        set_ring_led_pulse_enable(true);
        set_front_rgb_default(false);
        set_front_rgb_preview_selection_10s(RGB_LIGHTBLUE);
        set_freeze_btn_color(FREEZE_BTN_LIGHTBLUE);
        set_light_btn_led(true);
        set_on_btn_led(true);
        ft_results.leds_passed = true;
        break;

    case FT_STATE_ACTUATORS:
        set_ring_led_pulse_enable(false);
        set_ring_led_level_percent(100U);
        set_motor_speed(cfg->deice_motor_speed);
        set_fan_speed(100U);
        ft_results.motor_rpm_in_range     = false;
        ft_results.motor_current_in_range = false;
        ft_results.fans_spinning          = false;
        break;

    case FT_STATE_SENSORS: {
        float evap_in  = 0.0f;
        float evap_out = 0.0f;
        bool  have_in  = (get_evap_in_temp(&evap_in)  == 0U);
        bool  have_out = (get_evap_out_temp(&evap_out) == 0U);

        if (!have_in)  { evap_in  = 0.0f; }
        if (!have_out) { evap_out = 0.0f; }

        ft_results.evap_in_temp_start  = evap_in;
        ft_results.evap_out_temp_start = evap_out;

        bool in_range = have_in && have_out &&
                        (evap_in  >= cfg->factory_test_temp_min) &&
                        (evap_in  <= cfg->factory_test_temp_max) &&
                        (evap_out >= cfg->factory_test_temp_min) &&
                        (evap_out <= cfg->factory_test_temp_max);

        ft_results.sensors_in_range = in_range;

        if (!in_range) {
            factory_test_transition(FT_STATE_FAIL, now_ms);
        } else {
            factory_test_transition(FT_STATE_COOLING, now_ms);
        }
        return;
    }

    case FT_STATE_COOLING:
        set_motor_speed(cfg->deice_motor_speed);
        set_fan_speed(100U);
        set_compressor_speed(freq_hz_to_rpm(cfg->pull_down_max_freq_initial));
        break;

    case FT_STATE_PASS:
        factory_test_stop_all();
        set_front_rgb_preview_selection_10s(RGB_GREEN);
        set_freeze_btn_color(FREEZE_BTN_GREEN);
        ft_results.overall_pass = ft_results.leds_passed &&
                                  ft_results.fans_spinning &&
                                  ft_results.motor_rpm_in_range &&
                                  ft_results.motor_current_in_range &&
                                  ft_results.sensors_in_range &&
                                  ft_results.cooling_temp_drop_ok &&
                                  !ft_results.aborted;
        ft_active   = false;
        ft_complete = true;
        break;

    case FT_STATE_FAIL:
        factory_test_stop_all();
        set_front_rgb_preview_selection_10s(RGB_RED);
        set_freeze_btn_color(FREEZE_BTN_OFF);
        ft_results.overall_pass = false;
        ft_active   = false;
        ft_complete = true;
        break;

    default:
        break;
    }
}

static void factory_test_transition(ft_state_t new_state, uint32_t now_ms)
{
    ft_state          = new_state;
    ft_state_start_ms = now_ms;
    factory_test_enter_state(new_state, now_ms);
}

/* -------------------------------------------------------------------------- */
/*                              Public API                                    */
/* -------------------------------------------------------------------------- */

void factory_test_init(void)
{
    ft_state           = FT_STATE_IDLE;
    ft_state_start_ms  = 0U;
    ft_active          = false;
    ft_complete        = false;
    ft_start_requested = false;
    ft_stop_requested  = false;
    factory_test_reset_results();
    factory_test_stop_all();
}

void factory_test_request_start(void)
{
    if (ft_active) {
        return;
    }

    ft_start_requested = true;
    ft_stop_requested  = false;
}

void factory_test_request_stop(void)
{
    ft_stop_requested = true;
}

bool factory_test_is_active(void)
{
    return ft_active;
}

bool factory_test_is_complete(void)
{
    return ft_complete;
}

bool factory_test_has_failed(void)
{
    return ft_complete && !ft_results.overall_pass;
}

const factory_test_results_t *factory_test_get_results(void)
{
    return &ft_results;
}

void factory_test_run(uint32_t now_ms)
{
    const control_config_t *cfg = &g_control_config;

    if (ft_stop_requested) {
        ft_stop_requested = false;
        if (ft_active) {
            ft_results.aborted = true;
            factory_test_transition(FT_STATE_FAIL, now_ms);
        }
        return;
    }

    if (ft_start_requested) {
        ft_start_requested = false;
        factory_test_reset_results();
        ft_results.aborted     = false;
        ft_results.overall_pass = false;
        ft_active              = true;
        ft_complete            = false;
        factory_test_transition(FT_STATE_LIGHTS, now_ms);
    }

    if (!ft_active) {
        return;
    }

    switch (ft_state) {
    case FT_STATE_LIGHTS:
        if ((now_ms - ft_state_start_ms) >= FACTORY_TEST_LIGHT_STAGE_MS) {
            factory_test_transition(FT_STATE_ACTUATORS, now_ms);
        }
        break;

    case FT_STATE_ACTUATORS:
        if ((now_ms - ft_state_start_ms) >= cfg->factory_test_actuator_test_duration_ms) {
            float rpm     = 0.0f;
            float current = 0.0f;
            float fan1    = 0.0f;
            float fan2    = 0.0f;

            if (get_auger_speed(&rpm) != 0U) {
                rpm = 0.0f;
            }
            if (get_motor_current(&current) != 0U) {
                current = 0.0f;
            }
            if (get_fan_one_speed(&fan1) != 0U) {
                fan1 = 0.0f;
            }
            if (get_fan_two_speed(&fan2) != 0U) {
                fan2 = 0.0f;
            }

            ft_results.motor_rpm_measured     = rpm;
            ft_results.motor_current_measured = current;
            ft_results.fan_one_rpm            = fan1;
            ft_results.fan_two_rpm            = fan2;

            bool rpm_ok     = (rpm >= cfg->factory_test_motor_rpm_min) &&
                              (rpm <= cfg->factory_test_motor_rpm_max);
            bool current_ok = (current >= cfg->factory_test_motor_current_min) &&
                              (current <= cfg->factory_test_motor_current_max);
            bool fans_ok    = (fan1 > 0.0f) && (fan2 > 0.0f);

            ft_results.motor_rpm_in_range     = rpm_ok;
            ft_results.motor_current_in_range = current_ok;
            ft_results.fans_spinning          = fans_ok;

            if (rpm_ok && current_ok && fans_ok) {
                factory_test_transition(FT_STATE_SENSORS, now_ms);
            } else {
                factory_test_transition(FT_STATE_FAIL, now_ms);
            }
        }
        break;

    case FT_STATE_COOLING: {
        uint32_t elapsed = now_ms - ft_state_start_ms;
        uint32_t stage1  = cfg->factory_test_stage1_duration_ms;
        uint32_t stage2  = cfg->factory_test_stage2_duration_ms;
        uint32_t stage3  = cfg->factory_test_stage3_duration_ms;
        uint32_t stage4  = cfg->factory_test_stage4_duration_ms;
        uint32_t total   = stage1 + stage2 + stage3 + stage4;

        if (elapsed < stage1) {
            set_compressor_speed(freq_hz_to_rpm(cfg->pull_down_max_freq_initial));
        } else if (elapsed < (stage1 + stage2)) {
            set_compressor_speed(0U);
        } else if (elapsed < (stage1 + stage2 + stage3)) {
            set_compressor_speed(freq_hz_to_rpm(cfg->pull_down_max_freq_initial * 0.5f));
        } else if (elapsed < total) {
            set_compressor_speed(freq_hz_to_rpm(cfg->pull_down_min_freq));
        } else {
            float evap_in_end  = 0.0f;
            float evap_out_end = 0.0f;
            bool  have_in      = (get_evap_in_temp(&evap_in_end)  == 0U);
            bool  have_out     = (get_evap_out_temp(&evap_out_end) == 0U);

            if (!have_in)  { evap_in_end  = 0.0f; }
            if (!have_out) { evap_out_end = 0.0f; }

            ft_results.evap_in_temp_end   = evap_in_end;
            ft_results.evap_out_temp_end  = evap_out_end;
            ft_results.evap_in_temp_drop  = ft_results.evap_in_temp_start  - evap_in_end;
            ft_results.evap_out_temp_drop = ft_results.evap_out_temp_start - evap_out_end;

            bool drop_ok = have_in && have_out &&
                            (ft_results.evap_in_temp_drop  >= cfg->factory_test_temp_drop_threshold) &&
                            (ft_results.evap_out_temp_drop >= cfg->factory_test_temp_drop_threshold);

            ft_results.cooling_temp_drop_ok = drop_ok;

            factory_test_transition(drop_ok ? FT_STATE_PASS : FT_STATE_FAIL, now_ms);
        }
        break;
    }

    default:
        break;
    }
}
