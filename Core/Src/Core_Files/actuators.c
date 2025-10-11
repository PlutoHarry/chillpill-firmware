/*
 * actuators.c
 * Updated: Oct 2025
 * Author: Harry Lawton
 *
 * PURPOSE
 * -------
 *   Implements all low-level actuation outputs for the ChillPill control
 *   board.  This module is responsible for bringing up and driving
 *   pulse-width-modulated (PWM) channels as well as several GPIO lines
 *   used to control the compressor inverter, the auger (BLDC) motor and
 *   associated brake/direction pins, plus two fans.  It also exposes
 *   helper routines for changing the compressor frequency, setting a
 *   desired auger rotational speed via a PID loop, adjusting fan
 *   brightness and frequency, and toggling motor direction/enable.
 *
 * DEPENDENCIES
 * ------------
 *   This module depends on the STM32 HAL (stm32f1xx_hal.h) for timer and
 *   GPIO handling.  It requires the timers defined in main.h to be
 *   initialised by CubeMX prior to calling actuators_init().  The PID
 *   controller in set_motor_speed() relies on a valid auger speed
 *   measurement from sensors.c (get_auger_speed()).
 *
 * GLOBAL STATE
 * ------------
 *   - compressor_rpm: a latch of the most recently programmed compressor
 *     speed (RPM).  Use get_compressor_speed() to read it.
 *   - A static motor_enabled flag tracks whether the auger brake has been
 *     released by set_motor_speed().  This prevents attempting to drive
 *     the motor before the PID controller has executed at least once.
 *
 * PUBLIC API
 * ---------
 *   void actuators_init(void);
 *     Initialise all PWM outputs (motor, fans, compressor).  Must be
 *     called once after the timers are configured but before any calls
 *     to set_* functions.  This routine also sets the initial duty
 *     cycle to zero and asserts the motor brake.
 *
 *   void set_compressor_speed(uint16_t requested_rpm);
 *     Change the compressor speed by selecting an appropriate PWM
 *     frequency on the inverter timer.  Ramps quickly between speeds
 *     and disables the inverter below a threshold.  The last requested
 *     RPM can be queried via get_compressor_speed().
 *
 *   uint16_t get_compressor_speed(void);
 *     Return the current compressor setpoint in RPM.
 *
 *   void set_motor_speed(float target_auger_rpm);
 *     Request an auger rotational speed.  Internally this applies a
 *     simple PID loop with feed-forward to the motor PWM duty.  On the
 *     first call with a non-zero setpoint, this function automatically
 *     releases the motor brake and applies a starting duty to overcome
 *     static friction before entering the closed-loop control phase.
 *     Passing a near-zero target (<0.05 RPM) will stop and brake the
 *     motor.  The BLDC is geared 1:144, so the requested RPM is the auger
 *     output speed measured downstream of the gearbox.
 *
 *   void motor_enable(void), void motor_disable(void);
 *     Manually release or engage the brake.  These are wrapped by
 *     set_motor_speed() and should not normally be called elsewhere.
 *
 *   void motor_clockwise(void), void motor_anticlockwise(void);
 *     Select motor direction (for auger).  Clockwise/anticlockwise is
 *     application defined.
 *
 *   void set_fan_speed(uint8_t percent);
 *     Set both fans to a duty cycle between 10–100%.  Values below
 *     10% are clamped up to preserve airflow.
 *
 *   void pwm_fan_freq_set(fan_frequency_t freq);
 *     Change the prescaler on the fan PWM timer to adjust the PWM
 *     frequency.  See lights.c for available frequency enum values.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "actuators.h"
#include "main.h"

/* ===== Private state ===================================================== */

static uint16_t compressor_rpm = 0U;
static bool motor_enabled_flag = false; /* tracks brake state */

/* ===== Helpers =========================================================== */

static inline uint32_t clamp_u32(uint32_t v, uint32_t lo, uint32_t hi)
{ return (v < lo ? lo : (v > hi ? hi : v)); }

/* ===== Public: Basic controls =========================================== */

void motor_enable(void)       { HAL_GPIO_WritePin(MOTOR_BRAK_PB1_GPIO_Port, MOTOR_BRAK_PB1_Pin, GPIO_PIN_RESET); }
void motor_disable(void)      { HAL_GPIO_WritePin(MOTOR_BRAK_PB1_GPIO_Port, MOTOR_BRAK_PB1_Pin, GPIO_PIN_SET);  }
void motor_clockwise(void)    { HAL_GPIO_WritePin(MOTOR_ROT_PA7_GPIO_Port,  MOTOR_ROT_PA7_Pin,  GPIO_PIN_RESET); }
void motor_anticlockwise(void){ HAL_GPIO_WritePin(MOTOR_ROT_PA7_GPIO_Port,  MOTOR_ROT_PA7_Pin,  GPIO_PIN_SET);   }

/* ----- Fan PWM frequency (prescaler sets frequency band) ---------------- */

void pwm_fan_freq_set(fan_frequency_t freq)
{
    const uint32_t timer_clk_hz = 36000000UL;
    const uint32_t default_freq_hz = 2000UL;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&FAN_PWM_TIM);
    uint32_t selected_freq_hz = default_freq_hz;

    switch (freq) {
        case FAN_FREQ_166:  selected_freq_hz = 166UL;   break;
        case FAN_FREQ_1K:   selected_freq_hz = 1000UL;  break;
        case FAN_FREQ_2K:   selected_freq_hz = 2000UL;  break;
        case FAN_FREQ_4K:   selected_freq_hz = 4000UL;  break;
        case FAN_FREQ_5K:   selected_freq_hz = 5000UL;  break;
        case FAN_FREQ_20K:  selected_freq_hz = 20000UL; break;
        default:            selected_freq_hz = default_freq_hz; break;
    }

    uint32_t psc = 0U;
    uint32_t denom = selected_freq_hz * (arr + 1U);

    if ((denom == 0U) || ((timer_clk_hz / denom) == 0U)) {
        selected_freq_hz = default_freq_hz;
        denom = selected_freq_hz * (arr + 1U);
    }

    if (denom > 0U) {
        uint32_t divider = timer_clk_hz / denom;
        if (divider > 0U) {
            psc = divider - 1U;
        }
    }

    __HAL_TIM_SET_PRESCALER(&FAN_PWM_TIM, psc);
}

/* ----- Fan duty 10–100% (clamped) -------------------------------------- */

void set_fan_speed(uint8_t percent)
{
    if (percent < 10U) percent = 10U;
    if (percent > 100U) percent = 100U;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&FAN_PWM_TIM);
    uint32_t pulse = (uint32_t)((((uint32_t)percent) * (arr+1U))/100U);

    __HAL_TIM_SET_COMPARE(&FAN_PWM_TIM, FAN_PWM_CHANNEL,  pulse);
    __HAL_TIM_SET_COMPARE(&FAN_PWM_TIM, FAN2_PWM_CHANNEL, pulse);
}

/* ===== Init ============================================================== */

void actuators_init(void)
{
    /* Motor PWM (TIM3 CH3) duty = 0, brake engaged */
    __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, MOTOR_PWM_CHANNEL, 0U);
    HAL_TIM_PWM_Start(&MOTOR_PWM_TIM, MOTOR_PWM_CHANNEL);
    motor_disable();
    motor_enabled_flag = false;

    /* Fan PWM (TIM4 CH1 & CH3) */
    __HAL_TIM_SET_COMPARE(&FAN_PWM_TIM,  FAN_PWM_CHANNEL,  0U);
    __HAL_TIM_SET_COMPARE(&FAN_PWM_TIM,  FAN2_PWM_CHANNEL, 0U);
    HAL_TIM_PWM_Start(&FAN_PWM_TIM, FAN_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&FAN_PWM_TIM, FAN2_PWM_CHANNEL);

    /* Compressor inverter PWM (TIM1 CH1) initially off */
    __HAL_TIM_SET_COMPARE(&INVERTER_PWM_TIM, INVERTER_PWM_CHANNEL, 0U);
    HAL_TIM_PWM_Start(&INVERTER_PWM_TIM, INVERTER_PWM_CHANNEL);
}

/* -------------------------------------------------------------------------- */
/*                              COMPRESSOR CONTROL                            */
/* -------------------------------------------------------------------------- */

void set_compressor_speed(uint16_t requested_rpm)
{
    const uint32_t MIN_FREQ_HZ = 40U;   /* Embraco VCC3 frequency window */
    const uint32_t MAX_FREQ_HZ = 150U;
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
    uint32_t range = (MAX_RPM > band_min) ? ((uint32_t)MAX_RPM - (uint32_t)band_min) : 1U;
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
    freq_hz = clamp_u32(freq_hz, MIN_FREQ_HZ, MAX_FREQ_HZ);

    /* Determine the effective timer base frequency after prescaling so we
     * can program an accurate inverter drive frequency.  TIM1 resides on
     * APB2, which doubles the timer clock when the bus prescaler is >1. */
    uint32_t timer_clk_hz = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_HCLK_DIV1) {
        timer_clk_hz *= 2U;
    }

    uint32_t prescaler = (uint32_t)(INVERTER_PWM_TIM.Instance->PSC) + 1U;
    uint32_t ticks_per_period = 0U;

    if ((prescaler > 0U) && (freq_hz > 0U)) {
        uint32_t base_hz = timer_clk_hz / prescaler;
        if (base_hz >= freq_hz) {
            ticks_per_period = base_hz / freq_hz;
        }
    }

    if (ticks_per_period == 0U) {
        /* Fallback: keep current ARR (avoids div-by-zero) */
        ticks_per_period = __HAL_TIM_GET_AUTORELOAD(&INVERTER_PWM_TIM) + 1U;
    }

    __HAL_TIM_SET_AUTORELOAD(&INVERTER_PWM_TIM, ticks_per_period - 1U);
    __HAL_TIM_SET_COMPARE(&INVERTER_PWM_TIM, INVERTER_PWM_CHANNEL, ticks_per_period / 2U);
    HAL_TIM_PWM_Start(&INVERTER_PWM_TIM, INVERTER_PWM_CHANNEL);

    compressor_rpm = current_rpm;
}

uint16_t get_compressor_speed(void) { return compressor_rpm; }

/* -------------------------------------------------------------------------- */
/*                                   MOTOR                                    */
/* -------------------------------------------------------------------------- */

extern uint8_t get_auger_speed(float *rpm_out);

void set_motor_speed(float target_auger_rpm)
{
    /* Feed-forward + PID, with soft target ramp to avoid jerk */
    const float FF_OFFSET = 0.12f;  /* base duty */
    const float FF_GRAD   = 0.0025f;/* duty per rpm */
    const float Kp = 0.035f, Ki = 0.55f, Kd = 0.000f;
    const float DUTY_MIN = 0.12f, DUTY_MAX = 0.85f;
    const float INT_LIM  = 0.20f, INT_LEAK = 0.04f;

    static float integral = 0.0f, last_err = 0.0f, d_filt = 0.0f, ramp_rpm = 0.0f;
    static uint32_t last_ms = 0U;
    static bool pid_ready = false;

    uint32_t now_ms = HAL_GetTick();
    float dt = (now_ms - last_ms) * 0.001f;
    if (dt < 0.002f) dt = 0.002f;

    /* Automatic motor bring-up: on the very first non-zero request,
     * release the brake and drive with a fixed starting duty.  This
     * ensures the auger begins to spin even before any encoder
     * feedback is available.  Once started, subsequent calls will
     * enter the PID loop. */
    if (!motor_enabled_flag && target_auger_rpm > 1.0f) {
        motor_enable();
        motor_enabled_flag = true;
        /* Apply a conservative starting duty (30%) to overcome static
         * friction.  After this initial pulse the regular PID loop
         * takes over on the next invocation. */
        const float START_DUTY = 0.30f;
        uint32_t period_start = __HAL_TIM_GET_AUTORELOAD(&MOTOR_PWM_TIM);
        uint32_t pulse_start  = (uint32_t)(START_DUTY * (float)period_start + 0.5f);
        __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, MOTOR_PWM_CHANNEL, pulse_start);
        /* Seed ramp target to a fraction of the requested rpm to avoid
         * an abrupt jump in the next iteration. */
        ramp_rpm = target_auger_rpm * 0.2f;
        /* Reset integrator state and timers */
        integral = last_err = d_filt = 0.0f;
        pid_ready = false;
        last_ms = now_ms;
        return;
    }

    float delta = target_auger_rpm - ramp_rpm;
    float step = (fabsf(delta) / 2.0f) * dt;
    if (fabsf(delta) > 0.01f) ramp_rpm += copysignf(fminf(step, fabsf(delta)), delta);

    if (target_auger_rpm < 0.05f && ramp_rpm < 0.05f) {
        /* Stop the PID and brake the motor when the target is near zero. */
        integral = last_err = d_filt = ramp_rpm = 0.0f;
        pid_ready = false;
        __HAL_TIM_SET_COMPARE(&MOTOR_PWM_TIM, MOTOR_PWM_CHANNEL, 0U);
        motor_disable();
        motor_enabled_flag = false;
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
