#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include "build_config.h"

#ifndef ENABLE_DEBUG_LOGGING
#define ENABLE_DEBUG_LOGGING 0
#endif

#if ENABLE_DEBUG_LOGGING
#include <stdio.h>
#define LOG_INFO(...)  do { printf(__VA_ARGS__); } while (0)
#define LOG_WARN(...)  do { printf(__VA_ARGS__); } while (0)
#define LOG_ERROR(...) do { printf(__VA_ARGS__); } while (0)
#else
#define LOG_INFO(...)  ((void)0)
#define LOG_WARN(...)  ((void)0)
#define LOG_ERROR(...) ((void)0)
#endif

#endif /* DEBUG_LOG_H */
