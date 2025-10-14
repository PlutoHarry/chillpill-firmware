/*
 * buttons.c
 *
 * PURPOSE
 * -------
 *   Robust, non-blocking detection of three buttons:
 *     - POWER / FREEZE / LIGHT
 *   Features:
 *     - Per-button debouncing and short-gap merging
 *     - Multi-button holds with a “gesture” emitted on idle gap
 *     - Forced emit on very long holds (20 s, 30 s, 40 s…)
 *     - Durations are rounded to seconds and capped at 10 s per button
 *   The final gesture is delivered to user_settings via change_user_settings().
 *
 * DEPENDENCIES
 * ------------
 *   - HAL GPIO reads for each button pin
 *   - HAL_GetTick() timing source
 *
 * PUBLIC API
 * ----------
 *   void buttons_init(void);
 *   void buttons_task_20ms(uint32_t now_ms);
 *   (Internally calls change_user_settings(...).)
 */

/* full existing implementation follows — unchanged behavior-wise except header */
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "debug_log.h"
#include "buttons.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "user_settings.h"

/* ======================= Hardware mapping (from main.h) ======================= */
#define BTN_POWER_PORT   PC2_DET_EXT12_GPIO_Port
#define BTN_POWER_PIN    PC2_DET_EXT12_Pin
#define BTN_FREEZE_PORT  FREEZ_BTN_EXT10_GPIO_Port
#define BTN_FREEZE_PIN   FREEZ_BTN_EXT10_Pin
#define BTN_LIGHT_PORT   LIGHT_BTN_EXT11_GPIO_Port
#define BTN_LIGHT_PIN    LIGHT_BTN_EXT11_Pin

/* If any button is wired active-low, set its ACTIVE_HIGH to 0 */
#define POWER_ACTIVE_HIGH   1
#define FREEZE_ACTIVE_HIGH  1
#define LIGHT_ACTIVE_HIGH   1

/* ============================== Timing policy ============================== */
#define DEBOUNCE_MS         30U    /* contact debounce window                    */
#define IGNORE_GAP_MS      150U    /* merge short dropouts into same press       */
#define GESTURE_GAP_MS     150U    /* after last release, wait before emit       */

/* Long-hold policy */
#define MAX_ACCEPTED_MS  10000U    /* per-button max reported time (10 s)        */
#define FORCE_START_MS   20000U    /* do not force-emit before 20 s              */
#define FORCE_STEP_MS    10000U    /* emit at 20 s, 30 s, 40 s ... ONLY ONCE     */

/* Debug printing (set to 1 to enable) */
#ifndef BUTTONS_DEBUG
#define BUTTONS_DEBUG 0
#endif

/* ============================== Button core =============================== */
typedef struct {
    GPIO_TypeDef* port;
    uint16_t      pin;
    uint8_t       active_high;     /* 1: SET = pressed, 0: RESET = pressed */

    /* Debounce/filter */
    bool          debounced;       /* current debounced pressed state */
    bool          raw_last;        /* last raw level */
    uint32_t      last_change_ms;  /* last time raw changed */
    bool          waiting_stable;  /* in debounce window */

    /* Press accounting (gap tolerant) */
    bool          in_press;        /* currently considered "pressed" for timing */
    uint32_t      press_start_ms;  /* moment in_press started (or resumed) */
    uint32_t      total_ms;        /* accumulated time in this gesture */
    uint32_t      last_release_ms; /* last time it went (debounced) released */

    /* Gesture participation */
    bool          participated;    /* this button participated in current gesture */
} button_t;

typedef enum { IDX_POWER = 0, IDX_FREEZE, IDX_LIGHT, BTN_COUNT } btn_index_t;

static button_t g_btn[BTN_COUNT];

/* Aggregator for gesture */
static uint8_t  g_pressed_count = 0;        /* how many are debounced pressed now */
static bool     g_in_gesture     = false;
static uint32_t g_idle_start_ms  = 0;

/* Long-hold force-emit controller (per continuous gesture) */
static bool     g_force_emitted          = false;  /* once per long hold */
static uint32_t g_force_anchor_ms        = 0;      /* when gesture first started */
static uint32_t g_force_last_boundary_ms = 0;      /* last boundary we passed */

/* After a forced emission, block further emissions until all buttons are released */
static bool     g_block_until_all_released = false;

/* --------------------------- Helpers (defined before use) --------------------------- */

static inline bool raw_is_pressed(GPIO_TypeDef* port, uint16_t pin, uint8_t active_high)
{
    GPIO_PinState s = HAL_GPIO_ReadPin(port, pin);
    return active_high ? (s == GPIO_PIN_SET) : (s == GPIO_PIN_RESET);
}

static void button_setup(button_t* b,
                         GPIO_TypeDef* port, uint16_t pin, uint8_t active_high,
                         uint32_t now_ms)
{
    b->port         = port;
    b->pin          = pin;
    b->active_high  = active_high;

    bool raw        = raw_is_pressed(port, pin, active_high);
    b->debounced    = raw;            /* assume stable at boot */
    b->raw_last     = raw;
    b->last_change_ms = now_ms;
    b->waiting_stable = false;

    b->in_press       = raw;
    b->press_start_ms = raw ? now_ms : 0U;
    b->total_ms       = 0U;
    b->last_release_ms= now_ms;

    b->participated   = raw;
}

/* Finish (close) any active press and add to total_ms */
static void button_finish_press(button_t* b, uint32_t now_ms)
{
    if (!b->in_press) return;

    uint32_t held = (now_ms > b->press_start_ms) ? (now_ms - b->press_start_ms) : 0U;
    b->total_ms += held;
    b->in_press = false;
    b->last_release_ms = now_ms;
}

/* Start (or resume) a press; also marks gesture participation */
static void button_start_press(button_t* b, uint32_t now_ms)
{
    if (!b->in_press) {
        /* Merge short dropouts into the same press */
        if ((now_ms - b->last_release_ms) > IGNORE_GAP_MS) {
            /* treated as a new segment in the same gesture; accumulation continues in total_ms */
        }
        b->in_press = true;
        b->press_start_ms = now_ms;
    }
    b->participated = true;
}

/* Debounce + dropout-tolerant state machine per button */
static void button_update(button_t* b, uint32_t now_ms)
{
    bool raw = raw_is_pressed(b->port, b->pin, b->active_high);

    if (raw != b->raw_last) {
        b->raw_last = raw;
        b->last_change_ms = now_ms;
        b->waiting_stable = true;
        return;
    }

    if (b->waiting_stable) {
        if ((now_ms - b->last_change_ms) >= DEBOUNCE_MS) {
            b->waiting_stable = false;

            /* Stable change accepted */
            if (raw != b->debounced) {
                b->debounced = raw;

                if (b->debounced) {
                    /* transitioned to PRESSED */
                    button_start_press(b, now_ms);
                    g_pressed_count++;
                } else {
                    /* transitioned to RELEASED */
                    if (g_pressed_count > 0) g_pressed_count--;
                    /* Do not end the press immediately; allow IGNORE_GAP_MS for dropout merge.
                       Accumulation closes when idle or on force-emit. */
                }
            }
        }
    }

    /* If debounced released but we still consider "in_press", see if the gap exceeded IGNORE_GAP_MS */
    if (!b->debounced && b->in_press) {
        if ((now_ms - b->last_change_ms) > IGNORE_GAP_MS) {
            /* finalize this press segment */
            button_finish_press(b, b->last_change_ms);
        }
    }
}

/* Round ms to nearest second, capped at 10 s */
static inline uint8_t round_cap_sec(uint32_t ms)
{
    uint32_t capped = (ms > MAX_ACCEPTED_MS) ? MAX_ACCEPTED_MS : ms;
    return (uint8_t)((capped + 500U) / 1000U);
}

#if BUTTONS_DEBUG
static void debug_print_button_line(const char* name, uint8_t sec)
{
    /* Format: "FREEZE BTN PUSHED: 5 seconds" (one line per button that participated) */
    LOG_INFO("%s BTN PUSHED: %u seconds\r\n", name, (unsigned)sec);
}
#endif

/* Emit the current gesture (capping per-button to 10 s) and reset state. */
static void gesture_emit_and_reset(uint32_t now_ms)
{
    /* Compute per-button total including any active segment up to now_ms, then cap */
    uint32_t msP = g_btn[IDX_POWER].total_ms + (g_btn[IDX_POWER].in_press  ? (now_ms - g_btn[IDX_POWER].press_start_ms)  : 0U);
    uint32_t msF = g_btn[IDX_FREEZE].total_ms+ (g_btn[IDX_FREEZE].in_press ? (now_ms - g_btn[IDX_FREEZE].press_start_ms) : 0U);
    uint32_t msL = g_btn[IDX_LIGHT].total_ms + (g_btn[IDX_LIGHT].in_press  ? (now_ms - g_btn[IDX_LIGHT].press_start_ms)  : 0U);

    uint8_t secP = round_cap_sec(msP);
    uint8_t secF = round_cap_sec(msF);
    uint8_t secL = round_cap_sec(msL);

    /* Only call if any button actually participated */
    if (g_btn[IDX_POWER].participated ||
        g_btn[IDX_FREEZE].participated ||
        g_btn[IDX_LIGHT].participated)
    {
#if BUTTONS_DEBUG
        if (g_btn[IDX_POWER].participated)  debug_print_button_line("POWER",  secP);
        if (g_btn[IDX_FREEZE].participated) debug_print_button_line("FREEZE", secF);
        if (g_btn[IDX_LIGHT].participated)  debug_print_button_line("LIGHT",  secL);
#endif
        change_user_settings(secP, secF, secL);
    }

    /* Reset for next gesture */
    for (int i = 0; i < BTN_COUNT; ++i) {
        g_btn[i].total_ms       = 0U;
        g_btn[i].in_press       = false;
        g_btn[i].participated   = false;
        g_btn[i].press_start_ms = 0U;
        g_btn[i].last_release_ms= now_ms;
    }

    g_in_gesture                 = false;
    g_force_emitted              = false;
    g_force_anchor_ms            = 0U;
    g_force_last_boundary_ms     = 0U;
    g_block_until_all_released   = false;
    g_idle_start_ms              = now_ms;
}

/* Emit gesture if idle long enough and not blocked; reset accumulators */
static void gesture_maybe_emit_on_idle(uint32_t now_ms)
{
    if (!g_in_gesture) return;
    if (g_block_until_all_released) return; /* after forced emit, we wait for all release */

    if (g_pressed_count == 0) {
        if ((now_ms - g_idle_start_ms) >= GESTURE_GAP_MS) {
            gesture_emit_and_reset(now_ms);
        }
    }
}

/* Check long-hold auto-emit at 20 s, 30 s, 40 s ... (only once per gesture). */
static void gesture_maybe_force_emit(uint32_t now_ms)
{
    if (!g_in_gesture) return;
    if (g_force_emitted) return;                  /* already emitted for this hold */
    if (g_pressed_count == 0) return;             /* nothing pressed – idle path handles emit */
    if (g_block_until_all_released) return;       /* guard */

    /* Establish anchor when gesture first becomes active */
    if (g_force_anchor_ms == 0U) {
        g_force_anchor_ms = now_ms;
        g_force_last_boundary_ms = 0U;
        return;
    }

    /* Elapsed since gesture anchor (earliest current continuous-hold moment) */
    uint32_t elapsed = now_ms - g_force_anchor_ms;

    if (elapsed < FORCE_START_MS) return;         /* do not emit before 20 s */

    /* Compute the largest 10 s boundary we have passed since anchor */
    uint32_t boundary = (elapsed / FORCE_STEP_MS) * FORCE_STEP_MS;   /* 20, 30, 40 ... */

    if (boundary >= FORCE_START_MS && boundary != g_force_last_boundary_ms) {
        /* Emit ONCE at the first eligible boundary (20 s or higher) */
        g_force_last_boundary_ms = boundary;

        /* Report durations capped to 10 s (per requirement) */
        gesture_emit_and_reset(now_ms);

        /* Block any further emissions until all buttons are released */
        g_block_until_all_released = true;
        g_force_emitted            = true;
    }
}

/* -------------------------------------------------------------------------- */
/*                               Initializing                                  */
/* -------------------------------------------------------------------------- */
void buttons_init(void)
{
    uint32_t now = HAL_GetTick();
    button_setup(&g_btn[IDX_POWER],  BTN_POWER_PORT,  BTN_POWER_PIN,  POWER_ACTIVE_HIGH,  now);
    button_setup(&g_btn[IDX_FREEZE], BTN_FREEZE_PORT, BTN_FREEZE_PIN, FREEZE_ACTIVE_HIGH, now);
    button_setup(&g_btn[IDX_LIGHT],  BTN_LIGHT_PORT,  BTN_LIGHT_PIN,  LIGHT_ACTIVE_HIGH,  now);

    /* initialize pressed counter + gesture state */
    g_pressed_count = (g_btn[IDX_POWER].debounced ? 1U : 0U) +
                      (g_btn[IDX_FREEZE].debounced ? 1U : 0U) +
                      (g_btn[IDX_LIGHT].debounced ? 1U : 0U);

    g_in_gesture               = (g_pressed_count > 0);
    g_idle_start_ms            = now;
    g_force_emitted            = false;
    g_force_anchor_ms          = g_in_gesture ? now : 0U;
    g_force_last_boundary_ms   = 0U;
    g_block_until_all_released = false;
}

/* =============================== Public API =============================== */

/* Call every ~20 ms */
void buttons_task_20ms(uint32_t now_ms)
{
    /* If we previously forced an emission, wait for full release before tracking new gestures */
    if (g_block_until_all_released) {
        /* Update debouncers to detect releases */
        button_update(&g_btn[IDX_POWER],  now_ms);
        button_update(&g_btn[IDX_FREEZE], now_ms);
        button_update(&g_btn[IDX_LIGHT],  now_ms);

        if (g_pressed_count == 0) {
            /* All released → clear the block; next press will start a new gesture */
            g_block_until_all_released = false;
            g_in_gesture               = false;
            g_force_anchor_ms          = 0U;
            g_force_last_boundary_ms   = 0U;
        }
        return;
    }

    /* Update each button normally */
    uint8_t before = g_pressed_count;

    button_update(&g_btn[IDX_POWER],  now_ms);
    button_update(&g_btn[IDX_FREEZE], now_ms);
    button_update(&g_btn[IDX_LIGHT],  now_ms);

    /* Track gesture state */
    if (g_pressed_count > 0) {
        if (!g_in_gesture) {
            g_in_gesture = true;
            g_force_anchor_ms = now_ms;        /* start long-hold anchor */
            g_force_last_boundary_ms = 0U;
            g_force_emitted = false;
        }
        /* Evaluate long-hold rule (20 s, 30 s, 40 s ...) */
        gesture_maybe_force_emit(now_ms);
    } else {
        /* all released → start/refresh idle gap timer, if we just transitioned */
        if (before != 0 && g_pressed_count == 0) {
            g_idle_start_ms = now_ms;
        }
        /* while idle, see if we can emit (short holds and <20 s long holds) */
        gesture_maybe_emit_on_idle(now_ms);
    }
}
