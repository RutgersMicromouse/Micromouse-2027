#include "pid.h"

PIDController::PIDController()
    : kp_(0.0f), ki_(0.0f), kd_(0.0f), kf_(0.0f),
      out_min_(-800.0f), out_max_(800.0f),
      int_min_(-300.0f), int_max_(300.0f),
      integrator_(0.0f), prev_error_(0.0f),
      filtered_derivative_(0.0f),
      p_term_(0.0f), i_term_(0.0f), d_term_(0.0f) {
}

PIDController::PIDController(float kp, float ki, float kd, float kf, float out_min, float out_max)
    : kp_(kp), ki_(ki), kd_(kd), kf_(kf),
      out_min_(out_min), out_max_(out_max),
      int_min_(out_min * 0.5f), int_max_(out_max * 0.5f),
      integrator_(0.0f), prev_error_(0.0f),
      filtered_derivative_(0.0f),
      p_term_(0.0f), i_term_(0.0f), d_term_(0.0f) {
}

void PIDController::setGains(float kp, float ki, float kd, float kf) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    kf_ = kf;
}

void PIDController::setOutputLimits(float min_val, float max_val) {
    out_min_ = min_val;
    out_max_ = max_val;
}

void PIDController::setIntegratorLimits(float min_val, float max_val) {
    int_min_ = min_val;
    int_max_ = max_val;
}

void PIDController::reset() {
    integrator_ = 0.0f;
    prev_error_ = 0.0f;
    filtered_derivative_ = 0.0f;
    p_term_ = 0.0f;
    i_term_ = 0.0f;
    d_term_ = 0.0f;
}

float PIDController::update(float target, float feedback, float dt_seconds) {
    float error = target - feedback;
    float ff = target * kf_;
    float out = updateError(error, dt_seconds) + ff;
    return constrain(out, out_min_, out_max_);
}

float PIDController::updateError(float error, float dt_seconds) {
    if (dt_seconds <= 0.00001f) return 0.0f;

    // Proportional term
    p_term_ = kp_ * error;

    // Integral term with anti-windup clamping
    integrator_ += error * dt_seconds;
    integrator_ = constrain(integrator_, int_min_, int_max_);
    i_term_ = ki_ * integrator_;

    // Derivative term with low-pass filtering (cuts high frequency sensor noise)
    float raw_derivative = (error - prev_error_) / dt_seconds;
    const float alpha = 0.7f; // Filter weight on new derivative
    filtered_derivative_ = alpha * raw_derivative + (1.0f - alpha) * filtered_derivative_;
    d_term_ = kd_ * filtered_derivative_;

    prev_error_ = error;

    // Sum and clamp to output limits
    float output = p_term_ + i_term_ + d_term_;
    return constrain(output, out_min_, out_max_);
}
