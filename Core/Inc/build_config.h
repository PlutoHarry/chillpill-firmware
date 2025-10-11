/**
 * @file build_config.h
 * @brief Global build-time configuration flags for the ChillPill firmware.
 *
 * This header centralises compile-time feature toggles shared across the
 * firmware.  Define the macros before including this file to override the
 * defaults for a particular build configuration.
 */
#ifndef BUILD_CONFIG_H
#define BUILD_CONFIG_H

/* Enable the control finite state machine (1) or stub it out (0 by default). */
#ifndef ENABLE_CONTROL_FSM
#define ENABLE_CONTROL_FSM 0
#endif

#endif /* BUILD_CONFIG_H */
