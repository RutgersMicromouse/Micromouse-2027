#include "control/pid/pid.h"

// ==============================================================================
// PID CONTROLLER
// ==============================================================================

PIDController::PIDController()
    : kp_(0.0f), ki_(0.0f), kd_(0.0f),
      max_output_(1.0f), max_integral_(0.5f),
      integral_(0.0f), prev_error_(0.0f),
      prev_derivative_(0.0f), first_run_(true) {}

PIDController::PIDController(float kp, float ki, float kd, float max_out, float max_integral)
    : kp_(kp), ki_(ki), kd_(kd),
      max_output_(max_out),
      max_integral_(max_integral > 0.0f ? max_integral : max_out * 0.5f),
      integral_(0.0f), prev_error_(0.0f),
      prev_derivative_(0.0f), first_run_(true) {}

void PIDController::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(float max_out, float max_integral) {
    max_output_ = max_out;
    max_integral_ = (max_integral > 0.0f) ? max_integral : (max_out * 0.5f);
}

void PIDController::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    prev_derivative_ = 0.0f;
    first_run_ = true;
}

float PIDController::update(float error, float dt_seconds) {
    if (dt_seconds <= 0.00001f) {
        dt_seconds = 0.001f;
    }

    // 1. Proportional term
    float p_term = kp_ * error;

    // 2. Integral term with anti-windup clamping
    integral_ += error * dt_seconds;
    if (integral_ > max_integral_)  integral_ = max_integral_;
    if (integral_ < -max_integral_) integral_ = -max_integral_;
    float i_term = ki_ * integral_;

    // 3. Derivative term with low-pass filtering
    float raw_derivative = 0.0f;
    if (!first_run_) {
        raw_derivative = (error - prev_error_) / dt_seconds;
    } else {
        first_run_ = false;
    }
    prev_error_ = error;

    // Filter derivative to suppress high frequency sensor noise (cutoff ~50Hz)
    const float alpha = 0.3f;
    float filtered_derivative = (alpha * raw_derivative) + ((1.0f - alpha) * prev_derivative_);
    prev_derivative_ = filtered_derivative;
    float d_term = kd_ * filtered_derivative;

    // 4. Sum terms and saturate output
    float output = p_term + i_term + d_term;
    if (output > max_output_)  output = max_output_;
    if (output < -max_output_) output = -max_output_;

    return output;
}
