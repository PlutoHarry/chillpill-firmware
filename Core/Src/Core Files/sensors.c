/*
 * sensors.c
 *
 * PURPOSE
 *   Acquire and process sensors:
 *     - 3x NTC temperatures + 1x motor current via ADC1+DMA (rolling averages)
 *     - Motor/auger RPM from encoder capture (forwarded by IRQ layer)
 *     - Fan RPM derived from FG pulses over a sliding time window
 *   Provides clean getters and lifetime motor-hours accumulation with
 *   flash-seeded rounded base hours.
 *
 * PUBLIC API
 *   void sensors_init(void);
 *   void update_sensor_data(void);     // call every 1 ms
 *   void sensors_encoder_capture(float dt_seconds);
 *   void sensors_on_fan_one_pulse(void);
 *   void sensors_on_fan_two_pulse(void);
 *   void sensors_set_lifetime_hours_base(uint32_t rounded_hours_from_flash);
 *   uint8_t get_* getters for temps, current, fan RPM, auger RPM and hours
 */

#include "sensors.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include <stdint.h>

/* ---------- External peripherals (from main.c) ---------- */
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern void Error_Handler(void);

/* ---------- Defaults (overridable if defined elsewhere) ---------- */
#ifndef NUMBER_OF_ADC_CHANNEL
#define NUMBER_OF_ADC_CHANNEL 4   /* 3x NTC + 1x motor current */
#endif

/* Filter depths */
#ifndef ROLLING_AVG_SAMPLES
#define ROLLING_AVG_SAMPLES 16
#endif
#ifndef ROLLING_AVG_SIZE_FOR_CURRENT
#define ROLLING_AVG_SIZE_FOR_CURRENT 32
#endif

/* Motor RPM smoothing window (moving average samples) */
#ifndef MOTOR_RPM_AVG_SAMPLES
#define MOTOR_RPM_AVG_SAMPLES 20
#endif

/* Pulses-per-rev (adjust for your encoders/fans) */
#ifndef MOTOR_PULSES_PER_REV
#define MOTOR_PULSES_PER_REV 2.0f
#endif
#ifndef FAN_PULSES_PER_REV
#define FAN_PULSES_PER_REV  2.0f
#endif

/* Fan RPM integration window */
#ifndef FAN_RPM_WINDOW_MS
#define FAN_RPM_WINDOW_MS 250U
#endif

/* Acceptable encoder dt window (seconds) to reject spurious captures */
#ifndef ENC_MIN_DT_S
#define ENC_MIN_DT_S 0.0066f
#endif
#ifndef ENC_MAX_DT_S
#define ENC_MAX_DT_S 0.0600f
#endif

/* Requested application temp clamp */
#ifndef SENSOR_TEMP_MIN_C
#define SENSOR_TEMP_MIN_C  (-20.0f)
#endif
#ifndef SENSOR_TEMP_MAX_C
#define SENSOR_TEMP_MAX_C  ( 40.0f)
#endif

/* Sensor operating bounds (typical NTC) */
#ifndef NTC_OPER_MIN_C
#define NTC_OPER_MIN_C  (-30.0f)
#endif
#ifndef NTC_OPER_MAX_C
#define NTC_OPER_MAX_C  (105.0f)
#endif

/* Motor "running" detection threshold for hours accumulation */
#ifndef MOTOR_RUN_RPM_THRESHOLD
#define MOTOR_RUN_RPM_THRESHOLD 5.0f
#endif

/* ---------- NTC lookup table (descending resistance, 1 °C steps) ---------- */
static const uint32_t ntc_table[] = {
    340928,318877,298397,279368,261676,245221,229907,215648,202366,189987,
    178445,167678,157629,148246,139480,131288,123629,116464,109760,103482,
    97603,92094,86930,82087,77544,73279,69275,65515,61980,58658,
    55534,52595,49829,47225,44772,42462,40284,38230,36294,34466,
    32742,31113,29575,28122,26749,25451,24223,23061,21962,20921,
    19936,19002,18118,17280,16485,15731,15016,14337,13693,13081,
    12500,11948,11423,10925,10451,10000,9570,9162,8773,8403,
    8051,7715,7395,7090,6799,6522,6257,6005,5765,5535,
    5315,5105,4905,4713,4530,4355,4188,4028,3875,3729,
    3589,3455,3326,3203,3086,2973,2865,2761,2662,2566,
    2475,2387,2303,2223,2145,2071,1999,1931,1865,1801,
    1741,1682,1626,1572,1520,1470,1422,1375,1331,1288,
    1247,1207,1169,1132,1096,1062,1029,997,966,937,
    908,881,854,828,804,780,757,735,713,692,
    672,653,634,616,599,582
};
#define NTC_TABLE_SIZE (sizeof(ntc_table)/sizeof(ntc_table[0]))

/* First table entry corresponds to −40 °C (typical). */
#ifndef NTC_TABLE_T_START_C
#define NTC_TABLE_T_START_C (-40.0f)
#endif

/* ---------- ADC buffer (DMA) ---------- */
static volatile uint16_t adc_buffer[NUMBER_OF_ADC_CHANNEL] = {0};

/* ---------- Rolling average buffers ---------- */
static float temp_samples[3][ROLLING_AVG_SAMPLES] = {{0}};
static float current_samples[ROLLING_AVG_SIZE_FOR_CURRENT] = {0};
static uint8_t temp_idx = 0, temp_count = 0;
static uint8_t curr_idx = 0, curr_count = 0;

/* ---------- Clean outputs (module state) ---------- */
static float temperature_c[3] = {0.0f,0.0f,0.0f}; /* 0: evap-in, 1: evap-out, 2: bowl */
static float motor_current_a  = 0.0f;
static float motor_rpm        = 0.0f;
static float auger_rpm        = 0.0f;
static float fan1_rpm         = 0.0f;
static float fan2_rpm         = 0.0f;

/* ---------- Motor RPM smoothing ---------- */
static float motor_rpm_ma[MOTOR_RPM_AVG_SAMPLES] = {0.0f};
static uint8_t motor_rpm_ma_idx  = 0;
static uint8_t motor_rpm_ma_fill = 0;

/* ---------- Fan pulse counters (windowed) ---------- */
static volatile uint32_t fan1_pulses = 0;
static volatile uint32_t fan2_pulses = 0;
static uint32_t fan_window_start_ms = 0;

/* ---------- Lifetime motor-hours accumulation ---------- */
static uint32_t lifetime_hours_base_rounded = 0; /* set from flash on boot */
static uint32_t session_on_ms_accum         = 0; /* accumulated ON ms this run */
static uint32_t last_update_ms              = 0; /* for dt integration */

/* ============================== Helpers ============================== */
static inline uint32_t now_ms(void) { return HAL_GetTick(); }

static float average_f(const float *buf, uint8_t n)
{
    if (n == 0) return 0.0f;
    float s = 0.0f;
    for (uint8_t i = 0; i < n; ++i) s += buf[i];
    return s / (float)n;
}

/* R_ntc = R_series * ADC / (4096 - ADC), with R_series = 10k */
static float adc_to_ntc_resistance(float adc_avg)
{
    const float R_SERIES = 10000.0f;
    if (adc_avg <= 0.0f)     return 1e9f;
    if (adc_avg >= 4095.0f)  return 1e-3f;
    return (R_SERIES * adc_avg) / (4096.0f - adc_avg);
}

/* Binary search lower index in descending table: first idx where table[idx] <= r */
static int ntc_table_lower_idx_desc(float r_ntc)
{
    int lo = 0, hi = (int)NTC_TABLE_SIZE - 1;
    while (lo < hi) {
        int mid = (lo + hi) >> 1;
        if (r_ntc > (float)ntc_table[mid]) {
            lo = mid + 1;  /* go “warmer” (lower R, higher idx) */
        } else {
            hi = mid;
        }
    }
    return lo;
}

/* Convert resistance to °C using the LUT, then clamp to both the sensor’s
   operating range and the requested application range [-20, 40]. */
static float ntc_resistance_to_degC(float r_ntc)
{
    /* Bounds vs table */
    if (r_ntc >= (float)ntc_table[0]) {
        return (SENSOR_TEMP_MIN_C > NTC_OPER_MIN_C) ? SENSOR_TEMP_MIN_C : NTC_OPER_MIN_C;
    }
    if (r_ntc <= (float)ntc_table[NTC_TABLE_SIZE - 1]) {
        return (SENSOR_TEMP_MAX_C < NTC_OPER_MAX_C) ? SENSOR_TEMP_MAX_C : NTC_OPER_MAX_C;
    }

    int idx = ntc_table_lower_idx_desc(r_ntc);
    if (idx <= 0)                    return (SENSOR_TEMP_MIN_C > NTC_OPER_MIN_C) ? SENSOR_TEMP_MIN_C : NTC_OPER_MIN_C;
    if (idx >= (int)NTC_TABLE_SIZE)  return (SENSOR_TEMP_MAX_C < NTC_OPER_MAX_C) ? SENSOR_TEMP_MAX_C : NTC_OPER_MAX_C;

    /* Bracket: [idx-1]=colder (higher R), [idx]=warmer (lower R) */
    float r_hi = (float)ntc_table[idx - 1];
    float r_lo = (float)ntc_table[idx];
    float frac = (r_hi - r_ntc) / ((r_hi - r_lo) + 1e-6f);

    /* Table rows are 1 °C steps starting at NTC_TABLE_T_START_C */
    float t_lo = NTC_TABLE_T_START_C + (float)idx;
    float t    = t_lo - frac;  /* interpolate “upwards” towards colder */

    /* Clamp */
    if (t < NTC_OPER_MIN_C) t = NTC_OPER_MIN_C;
    if (t > NTC_OPER_MAX_C) t = NTC_OPER_MAX_C;
    if (t < SENSOR_TEMP_MIN_C) t = SENSOR_TEMP_MIN_C;
    if (t > SENSOR_TEMP_MAX_C) t = SENSOR_TEMP_MAX_C;
    return t;
}

/* ============================== Init ============================== */

void sensors_init(void)
{
    /* ADC calibration + DMA start */
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, NUMBER_OF_ADC_CHANNEL) != HAL_OK) {
        Error_Handler();
    }

    /* Motor encoder capture (base + IC) */
    HAL_TIM_Base_Start_IT(&MOTOR_FG_TIM);
    HAL_TIM_IC_Start_IT(&MOTOR_FG_TIM, MOTOR_FG_CHANNEL);

    /* Fan FG capture */
    HAL_TIM_Base_Start_IT(&FAN_FG_TIM);
    HAL_TIM_IC_Start_IT(&FAN_FG_TIM, FAN_FG_CHANNEL);
    HAL_TIM_IC_Start_IT(&FAN2_FG_TIM, FAN2_FG_CHANNEL);

    fan_window_start_ms = now_ms();
    last_update_ms      = fan_window_start_ms;
}

/* ============================== IRQ Hooks ============================== */
/* Called by your timer ISR layer to feed captures/pulses */

void sensors_encoder_capture(float dt_seconds)
{
    if (dt_seconds < ENC_MIN_DT_S || dt_seconds > ENC_MAX_DT_S) return;

    /* RPM = (1/dt) / pulses_per_rev * 60 */
    float rpm_inst = (1.0f / dt_seconds) * (60.0f / MOTOR_PULSES_PER_REV);

    motor_rpm_ma[motor_rpm_ma_idx++] = rpm_inst;
    if (motor_rpm_ma_idx >= MOTOR_RPM_AVG_SAMPLES) motor_rpm_ma_idx = 0;
    if (motor_rpm_ma_fill < MOTOR_RPM_AVG_SAMPLES) motor_rpm_ma_fill++;

    motor_rpm = average_f(motor_rpm_ma, motor_rpm_ma_fill);
    auger_rpm = motor_rpm / 144.0f;  /* project scale */
}

void sensors_on_fan_one_pulse(void) { fan1_pulses++; }
void sensors_on_fan_two_pulse(void) { fan2_pulses++; }

/* ============================== Core Update ============================== */

static void update_temperatures(void)
{
    /* Push raw ADC for NTC channels (0..2) into rolling windows */
    for (uint8_t ch = 0; ch < 3; ++ch) {
        temp_samples[ch][temp_idx] = (float)adc_buffer[ch];
    }
    temp_idx = (uint8_t)((temp_idx + 1) % ROLLING_AVG_SAMPLES);
    if (temp_count < ROLLING_AVG_SAMPLES) temp_count++;

    /* Average -> resistance -> °C */
    for (uint8_t ch = 0; ch < 3; ++ch) {
        float sum = 0.0f;
        for (uint8_t i = 0; i < temp_count; ++i) sum += temp_samples[ch][i];
        float adc_avg = sum / (float)temp_count;

        float r_ntc = adc_to_ntc_resistance(adc_avg);
        temperature_c[ch] = ntc_resistance_to_degC(r_ntc);
    }
}

static void update_motor_current(void)
{
    /* Rolling average on ADC channel 3 */
    current_samples[curr_idx] = (float)adc_buffer[3];
    curr_idx = (uint8_t)((curr_idx + 1) % ROLLING_AVG_SIZE_FOR_CURRENT);
    if (curr_count < ROLLING_AVG_SIZE_FOR_CURRENT) curr_count++;

    float sum = 0.0f;
    for (uint8_t i = 0; i < curr_count; ++i) sum += current_samples[i];
    float raw_avg = sum / (float)curr_count;

    /* (((code * 2 * 3.3) / 4096) / 0.5) -> adjust SHUNT_GAIN as needed */
    const float VREF = 3.3f;
    const float SCALE = (2.0f * VREF) / 4096.0f;
    const float SHUNT_GAIN = 0.5f;  /* adjust to hardware */
    motor_current_a = (raw_avg * SCALE) / SHUNT_GAIN;
}

static void update_fan_rpm_windowed(void)
{
    uint32_t t = now_ms();
    if ((t - fan_window_start_ms) < FAN_RPM_WINDOW_MS) return;

    float dt_s = (float)(t - fan_window_start_ms) / 1000.0f;
    if (dt_s <= 0.0f) dt_s = (float)FAN_RPM_WINDOW_MS / 1000.0f;

    /* RPM = (pulses / dt) * (60 / pulses_per_rev) */
    fan1_rpm = ((float)fan1_pulses / dt_s) * (60.0f / FAN_PULSES_PER_REV);
    fan2_rpm = ((float)fan2_pulses / dt_s) * (60.0f / FAN_PULSES_PER_REV);

    fan1_pulses = 0;
    fan2_pulses = 0;
    fan_window_start_ms = t;
}

static void update_motor_hours_accum(void)
{
    uint32_t t = now_ms();
    uint32_t dt = t - last_update_ms;   /* unsigned wrap-safe */
    last_update_ms = t;

    /* Accumulate ON-time only when motor is actually spinning */
    if (motor_rpm > MOTOR_RUN_RPM_THRESHOLD) {
        session_on_ms_accum += dt;
    }
}

void update_sensor_data(void)
{
    update_motor_current();
    update_temperatures();
    update_fan_rpm_windowed();
    update_motor_hours_accum();
    /* motor_rpm & auger_rpm maintained by sensors_encoder_capture() */
}

/* ============================== Public API ============================== */

uint8_t get_evap_in_temp(float *temperature_out)
{
    if (!temperature_out) return 1;
    *temperature_out = temperature_c[0];
    return 0;
}

uint8_t get_evap_out_temp(float *temperature_out)
{
    if (!temperature_out) return 1;
    *temperature_out = temperature_c[1];
    return 0;
}

uint8_t get_bowl_temp(float *temperature_out)
{
    if (!temperature_out) return 1;
    *temperature_out = temperature_c[2];
    return 0;
}

uint8_t get_motor_current(float *current_out)
{
    if (!current_out) return 1;
    *current_out = motor_current_a;
    return 0;
}

uint8_t get_fan_one_speed(float *rpm_out)
{
    if (!rpm_out) return 1;
    *rpm_out = fan1_rpm;
    return 0;
}

uint8_t get_fan_two_speed(float *rpm_out)
{
    if (!rpm_out) return 1;
    *rpm_out = fan2_rpm;
    return 0;
}

uint8_t get_auger_speed(float *rpm_out)
{
    if (!rpm_out) return 1;
    *rpm_out = auger_rpm;
    return 0;
}

/* Seed the lifetime hours base (rounded hours loaded from flash on boot) */
void sensors_set_lifetime_hours_base(uint32_t rounded_hours_from_flash)
{
    lifetime_hours_base_rounded = rounded_hours_from_flash;
    session_on_ms_accum = 0;
    last_update_ms = now_ms();
}

/* Exact (fractional) lifetime motor hours = base (rounded from flash) + session */
float get_motor_hours(void)
{
    const float SESSION_H = (float)session_on_ms_accum / 3600000.0f;
    return (float)lifetime_hours_base_rounded + SESSION_H;
}
