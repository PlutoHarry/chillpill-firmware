#ifndef ESTIMATORS_H
#define ESTIMATORS_H

#include "build_config.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file estimator.h
 * @brief High level derived-state estimators for the slush machine controller.
 *
 * The estimators module fuses raw sensor data (NTC probes, compressor command,
 * fan speeds and motor feedback) into higher level, slowly varying quantities
 * that the finite state machine and the control loops can consume directly.
 * Only the public API is exposed here; the implementation and its working
 * state remain private to estimator.c so the rest of the firmware cannot rely
 * on internal details.
 */
void estimator_init(void);

/**
 * @brief Run the estimator update step. Call every 20 ms.
 */
void estimator_update(void);

/** Initialise estimator state and configuration. */
void estimator_init(void);

/** Execute the estimator update step. Call every 20 ms. */
void estimator_update(void);

/** Retrieve the compensated liquid temperature estimate (degrees Celsius). */
float estimator_get_real_bowl_temp(void);

/** Fetch the current slush texture index in the range [0, 1]. */
float estimator_get_texture_index(void);

/** Return the condenser load index (dimensionless). */
float estimator_get_condenser_load(void);

/** Obtain the continuous volume estimate from 0.0 (empty) to 1.0 (full). */
float estimator_get_volume(void);

/** Get the discretised volume level (0=low, 1=medium, 2=high). */
uint8_t estimator_get_volume_estimate(void);

/** Return the evaporator ice accumulation index in the range [0, 1]. */
float estimator_get_ice_index(void);

/** Indicate whether the machine has required de-icing for a sustained time. */
bool estimator_needs_deicing(void);

/** Reset the volume estimator state after refilling or maintenance. */
void estimator_reset_volume(void);

/** Update the torque baseline during a known de-iced calibration window. */
void update_slush_torque_reference(void);

#else /* ENABLE_CONTROL_FSM */

static inline void estimator_init(void) {}
static inline void estimator_update(void) {}
static inline float estimator_get_real_bowl_temp(void) { return 0.0f; }
static inline float estimator_get_texture_index(void) { return 0.0f; }
static inline float estimator_get_condenser_load(void) { return 0.0f; }
static inline float estimator_get_volume(void) { return 0.0f; }
static inline uint8_t estimator_get_volume_estimate(void) { return 0U; }
static inline float estimator_get_ice_index(void) { return 0.0f; }
static inline bool estimator_needs_deicing(void) { return false; }
static inline void estimator_reset_volume(void) {}
static inline void update_slush_torque_reference(void) {}

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
}
#endif

#endif /* ESTIMATOR_H */
