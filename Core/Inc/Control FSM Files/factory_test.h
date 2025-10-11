#ifndef FACTORY_TEST_H
#define FACTORY_TEST_H

#include "build_config.h"
#include <stdbool.h>
#include <stdint.h>

/**
 * @file factory_test.h
 * @brief Manufacturing and service diagnostics coordinator for the control FSM.
 *
 * The factory test module orchestrates scripted behaviours that exercise the
 * cooling subsystem during production and maintenance. It exposes control hooks
 * for the application layer to start or stop diagnostics while sharing sensor
 * feedback with the estimator and fault handler modules.
 */

#if ENABLE_CONTROL_FSM

void factory_test_init(void);
void factory_test_run(uint32_t now_ms);
void factory_test_request_start(void);
void factory_test_request_stop(void);
bool factory_test_is_active(void);
bool factory_test_has_failed(void);

#else

static inline void factory_test_init(void) {}
static inline void factory_test_run(uint32_t now_ms) {(void)now_ms;}
static inline void factory_test_request_start(void) {}
static inline void factory_test_request_stop(void) {}
static inline bool factory_test_is_active(void) { return false; }
static inline bool factory_test_has_failed(void) { return false; }

#endif /* ENABLE_CONTROL_FSM */

#endif /* FACTORY_TEST_H */
