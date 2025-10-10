/*
 * actuators.c
 * Updated: Oct 2025
 * Author: Harry Lawton
 *
 * Responsible for all PWM-based and GPIO-based outputs:
 *  - Compressor inverter drive
 *  - Motor drive (BLDC auger)
 *  - Fan drive(s)
 *  - Basic init control sequencing
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "actuators.h"
#include "main.h"
#include "sensors.h"
#include "stm32f1xx_hal.h"

/* Global compressor RPM latch */
static uint16_t compressor_rpm = 0U;

/* -------------------------------------------------------------------------- */
/*                              INITIALIZATION                                */
/* -------------------------------------------------------------------------- */

void actuators_init(void)
{
    /* Motor PWM */
    HAL_TIM_Base_Start(&MOTOR_PWM_TIM);
    HAL_TIM_PWM_Init(&MOTOR_PWM_TIM);
    HAL_TIM_PWM_Start(&MOTOR_PWM_TIM, MOTOR_PWM_CHANNEL);
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, MOTOR_PWM_CHANNEL, 0U);
    motor_anticlockwise();
    motor_disable();

    /* Fans PWM */
    HAL_TIM_Base_Start(&FAN_PWM_TIM);
    HAL_TIM_PWM_Init(&FAN_PWM_TIM);
    HAL_TIM_PWM_Start(&FAN_PWM_TIM, FAN_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&FAN_PWM_TIM, FAN2_PWM_CHANNEL);
    __HAL_TIM_SET_COMPARE(&FAN_PWM_TIM, FAN_PWM_CHANNEL, 0U);
    __HAL_TIM_SET_COMPARE(&FAN_PWM_TIM, FAN2_PWM_CHANNEL, 0U);

    pwm_fan_freq_set(FAN_FREQ_20K);
    set_fan_speed(10U);

    /* Compressor inverter PWM */
    HAL_TIM_Base_Start(&INVERTER_PWM_TIM);
    HAL_TIM_PWM_Init(&INVERTER_PWM_TIM);
    __HAL_TIM_SET_COMPARE(&INVERTER_PWM_TIM, INVERTER_PWM_CHANNEL, 0U);
    HAL_TIM_PWM_Start(&INVERTER_PWM_TIM, INVERTER_PWM_CHANNEL);
}

/* -------------------------------------------------------------------------- */
/*                              COMPRESSOR CONTROL                            */
/* -------------------------------------------------------------------------- */

void set_compressor_speed(uint16_t requested_rpm)
{
    const uint32_t TIMER_BASE_HZ = 10000U;
    const uint16_t OFF_THRESH_RPM = 900U;
    const uint16_t SNAP_LOW_RPM = 1400U;
    const uint16_t MAX_RPM = 4500U;
    const float SMALL_FRACTION = 0.30f;
    const uint32_t STEP_MS = 20000U;
    const uint8_t MAX_STEPS = 4U;

    static bool ramp_active = false;
    static uint16_t ramp_start_rpm = 0U, ramp_target_rpm = 0U;
    static uint8_t ramp_steps_total = 0U;
    static uint32_t ramp_start_ms = 0U;

    /* Turn OFF instantly below threshold */
    if (requested_rpm < OFF_THRESH_RPM) {
        HAL_TIM_PWM_Stop(&INVERTER_PWM_TIM, INVERTER_PWM_CHANNEL);
        compressor_rpm = 0U;
        ramp_active = false;
        return;
    }

    /* Clamp requested rpm */
    if (requested_rpm > MAX_RPM) requested_rpm = MAX_RPM;
    if (requested_rpm < SNAP_LOW_RPM) requested_rpm = SNAP_LOW_RPM;

    uint16_t current_rpm = compressor_rpm;
    uint16_t target_rpm = requested_rpm;

    uint16_t band_min = (current_rpm == 0U) ? 0U : SNAP_LOW_RPM;
    uint16_t range = (MAX_RPM > band_min) ? (MAX_RPM - band_min) : 1U;
    float frac = fabsf((float)target_rpm - (float)current_rpm) / (float)range;

    uint8_t steps_needed = 0U;
    if (frac > SMALL_FRACTION) {
        steps_needed = (uint8_t)ceilf(fminf(frac * MAX_STEPS, MAX_STEPS));
    }

    uint32_t now = HAL_GetTick();
    if (steps_needed == 0U) {
        ramp_active = false;
        current_rpm = target_rpm;
    } else {
        ramp_active = true;
        ramp_start_rpm = current_rpm;
        ramp_target_rpm = target_rpm;
        ramp_steps_total = steps_needed;
        ramp_start_ms = now;
    }

    if (ramp_active) {
        uint32_t elapsed_ms = now - ramp_start_ms;
        uint8_t steps_elapsed = (uint8_t)(elapsed_ms / STEP_MS);
        if (steps_elapsed > ramp_steps_total) steps_elapsed = ramp_steps_total;

        int32_t span = (int32_t)ramp_target_rpm - (int32_t)ramp_start_rpm;
        int32_t allowed = (int32_t)ramp_start_rpm + (span * steps_elapsed) / ramp_steps_total;
        if (steps_elapsed >= ramp_steps_total) ramp_active = false;
        current_rpm = (uint16_t)((allowed < 0) ? 0 : allowed);
    }

    uint32_t freq_hz = (uint32_t)((current_rpm + 15U) / 30U);
    if (freq_hz < 30U) freq_hz = 30U;
    if (freq_hz > 150U) freq_hz = 150U;
    compressor_rpm = freq_hz * 30U;

    HAL_TIM_PWM_Stop(&INVERTER_PWM_TIM, INVERTER_PWM_CHANNEL);

    uint32_t period = (TIMER_BASE_HZ / freq_hz) - 1U;
    __HAL_TIM_SET_AUTORELOAD(&INVERTER_PWM_TIM, period);
    __HAL_TIM_SET_COMPARE(&INVERTER_PWM_TIM, INVERTER_PWM_CHANNEL, (period + 1U) / 2U);
    __HAL_TIM_SET_COUNTER(&INVERTER_PWM_TIM, 0U);
    HAL_TIM_GenerateEvent(&INVERTER_PWM_TIM, TIM_EVENTSOURCE_UPDATE);
    HAL_TIM_PWM_Start(&INVERTER_PWM_TIM, INVERTER_PWM_CHANNEL);
}

uint16_t get_compressor_speed(void)
{
    return compressor_rpm;
}

/* -------------------------------------------------------------------------- */
/*                               MOTOR CONTROL                                */
/* -------------------------------------------------------------------------- */

void set_motor_speed(float target_rpm)
{
    const float Kp = 0.05f, Ki = 0.15f, Kd = 0.001f;
    const float FF_OFFSET = 0.20f, FF_GRAD = 0.031f;
    const float DUTY_MIN = 0.00f, DUTY_MAX = 1.00f;
    const float INT_LIM = 0.35f, INT_LEAK = 0.15f;

    static float integral = 0.0f, last_err = 0.0f, d_filt = 0.0f, ramp_rpm = 0.0f;
    static uint32_t last_ms = 0U;
    static bool pid_ready = false;

    uint32_t now_ms = HAL_GetTick();
    float dt = (now_ms - last_ms) * 0.001f;
    if (dt < 0.002f) dt = 0.002f;

    float delta = target_rpm - ramp_rpm;
    float step = (fabsf(delta) / 2.0f) * dt;
    if (fabsf(delta) > 0.01f) ramp_rpm += copysignf(fminf(step, fabsf(delta)), delta);

    if (target_rpm < 0.05f && ramp_rpm < 0.05f) {
        integral = last_err = d_filt = ramp_rpm = 0.0f;
        pid_ready = false;
        __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, MOTOR_PWM_CHANNEL, 0U);
        last_ms = now_ms;
        return;
    }

    float auger_meas = 0.0f;
    (void)get_auger_speed(&auger_meas);

    if (!pid_ready) {
        integral = 0.0f;
        last_err = ramp_rpm - auger_meas;
        pid_ready = true;
    }

    float ff_duty = FF_OFFSET + FF_GRAD * ramp_rpm;
    float err = ramp_rpm - auger_meas;

    integral += err * dt;
    if (fabsf(err) < 0.5f) integral *= (1.0f - INT_LEAK * dt);
    if (integral > (INT_LIM / Ki)) integral = (INT_LIM / Ki);
    if (integral < -(INT_LIM / Ki)) integral = -(INT_LIM / Ki);

    float deriv = (err - last_err) / dt;
    const float D_ALPHA = 0.10f;
    d_filt += D_ALPHA * (deriv - d_filt);
    deriv = d_filt;

    float duty = ff_duty + Kp * err + Ki * integral + Kd * deriv;
    if (duty > DUTY_MAX) { duty = DUTY_MAX; integral -= err * dt; }
    if (duty < DUTY_MIN) { duty = DUTY_MIN; integral -= err * dt; }

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&MOTOR_PWM_TIM);
    uint32_t pulse = (uint32_t)(duty * (float)period + 0.5f);
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, MOTOR_PWM_CHANNEL, pulse);

    last_err = err;
    last_ms = now_ms;
}

void motor_enable(void)       { HAL_GPIO_WritePin(MOTOR_BRAK_PB1_GPIO_Port, MOTOR_BRAK_PB1_Pin, GPIO_PIN_RESET); }
void motor_disable(void)      { HAL_GPIO_WritePin(MOTOR_BRAK_PB1_GPIO_Port, MOTOR_BRAK_PB1_Pin, GPIO_PIN_SET); }
void motor_clockwise(void)    { HAL_GPIO_WritePin(MOTOR_ROT_PA7_GPIO_Port, MOTOR_ROT_PA7_Pin, GPIO_PIN_SET); }
void motor_anticlockwise(void){ HAL_GPIO_WritePin(MOTOR_ROT_PA7_GPIO_Port, MOTOR_ROT_PA7_Pin, GPIO_PIN_RESET); }

/* -------------------------------------------------------------------------- */
/*                                   FANS                                     */
/* -------------------------------------------------------------------------- */

void set_fan_speed(uint8_t percent)
{
    if (percent > 100U) percent = 100U;
    if (percent == 0U) {
        __HAL_TIM_SET_COMPARE(&FAN_PWM_TIM, FAN_PWM_CHANNEL, 0U);
        __HAL_TIM_SET_COMPARE(&FAN_PWM_TIM, FAN2_PWM_CHANNEL, 0U);
        return;
    }
    if (percent < 10U) percent = 10U;

    const float DUTY_MIN = 0.10f, DUTY_MAX = 0.95f;
    float t = ((float)percent - 10.0f) / 90.0f;
    float duty = DUTY_MIN + t * (DUTY_MAX - DUTY_MIN);

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&FAN_PWM_TIM);
    uint32_t pulse = (uint32_t)(duty * (float)period + 0.5f);
    __HAL_TIM_SET_COMPARE(&FAN_PWM_TIM, FAN_PWM_CHANNEL, pulse);
    __HAL_TIM_SET_COMPARE(&FAN_PWM_TIM, FAN2_PWM_CHANNEL, pulse);
}

void pwm_fan_freq_set(fan_frequency_t freq)
{
    __HAL_TIM_DISABLE(&FAN_PWM_TIM);
    HAL_Delay(1);

    switch (freq)
    {
        case FAN_FREQ_166:  __HAL_TIM_SET_PRESCALER(&FAN_PWM_TIM, TIMER_CLOCK_36M_166_SET - 1U); break;
        case FAN_FREQ_1K:   __HAL_TIM_SET_PRESCALER(&FAN_PWM_TIM, TIMER_CLOCK_36M_1K_SET  - 1U); break;
        case FAN_FREQ_2K:   __HAL_TIM_SET_PRESCALER(&FAN_PWM_TIM, TIMER_CLOCK_36M_2K_SET  - 1U); break;
        case FAN_FREQ_4K:   __HAL_TIM_SET_PRESCALER(&FAN_PWM_TIM, TIMER_CLOCK_36M_4K_SET  - 1U); break;
        case FAN_FREQ_5K:   __HAL_TIM_SET_PRESCALER(&FAN_PWM_TIM, TIMER_CLOCK_36M_5K_SET  - 1U); break;
        case FAN_FREQ_20K:  __HAL_TIM_SET_PRESCALER(&FAN_PWM_TIM, TIMER_CLOCK_36M_20K_SET - 1U); break;
        default: break;
    }

    __HAL_TIM_SET_COUNTER(&FAN_PWM_TIM, 0U);
    __HAL_TIM_ENABLE(&FAN_PWM_TIM);
    HAL_Delay(1);
}
