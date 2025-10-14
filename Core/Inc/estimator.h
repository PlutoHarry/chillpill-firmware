#ifndef ESTIMATORS_H
#define ESTIMATORS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise estimator state and configuration.
 */
void estimator_init(void);

/**
 * @brief Run the estimator update step. Call every 20 ms.
 */
void estimator_update(void);

/**
 * @brief Retrieve the fused liquid temperature estimate (°C).
 */
float estimator_get_real_bowl_temp(void);

/**
 * @brief Fetch the current slush texture index in the range [0, 1].
 */
float estimator_get_texture_index(void);

/**
 * @brief Return the condenser load index (dimensionless).
 */
float estimator_get_condenser_load(void);

/**
 * @brief Obtain the continuous volume estimate from 0.0 (empty) to 1.0 (full).
 */
float estimator_get_volume(void);

/**
 * @brief Get the discretised volume level (0=low, 1=medium, 2=high).
 */
uint8_t estimator_get_volume_estimate(void);

/**
 * @brief Return the evaporator ice accumulation index in the range [0, 1].
 */
float estimator_get_ice_index(void);

/**
 * @brief Indicate whether the machine has required de-icing for a sustained time.
 */
bool estimator_needs_deicing(void);

/**
 * @brief Reset the volume estimator state after refilling or maintenance.
 */
void estimator_reset_volume(void);

/**
 * @brief Update the torque baseline during a known de-iced calibration window.
 */
void update_slush_torque_reference(void);

/**
 * @brief Retrieve the filtered torque baseline used for texture normalisation.
 */
float estimator_get_torque_baseline(void);

/**
 * @brief Emit a debug telemetry line (rate-limited internally).
 */
void estimator_print_debug_data(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* ESTIMATORS_H */
