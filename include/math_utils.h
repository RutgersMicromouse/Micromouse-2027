#pragma once

#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

/**
 * @brief Normalizes an angle into the range (-180.0, +180.0] degrees in O(1) time.
 * Eliminates multi-iteration while-loops and handles large positive/negative offsets cleanly.
 */
static inline float normalizeAngle180(float deg) {
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg - 180.0f;
}

/**
 * @brief Normalizes an angle into the range [0.0, 360.0) degrees in O(1) time.
 */
static inline float normalizeAngle360(float deg) {
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

/**
 * @brief Computes the shortest signed angular difference (target - current) in (-180.0, +180.0] degrees.
 * Positive = CCW turn, Negative = CW turn.
 */
static inline float shortestAngularDifference(float target_deg, float current_deg) {
    return normalizeAngle180(target_deg - current_deg);
}

/**
 * @brief Deadband function: zeros out signals smaller than threshold.
 */
static inline float applyDeadband(float value, float threshold) {
    if (fabsf(value) < threshold) return 0.0f;
    return value;
}

/**
 * @brief Constrains a floating point value between min_val and max_val.
 */
static inline float clampFloat(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}
