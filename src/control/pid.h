#pragma once

#include <Arduino.h>

// =============================================================================
// Discrete PID Controller with Derivative Filtering & Anti-Windup
// =============================================================================

class PIDController {
public:
    PIDController();
    PIDController(float kp, float ki, float kd, float kf = 0.0f,
                  float out_min = -800.0f, float out_max = 800.0f);

    void setGains(float kp, float ki, float kd, float kf = 0.0f);
    void setOutputLimits(float min_val, float max_val);
    void setIntegratorLimits(float min_val, float max_val);

    void reset();

    float update(float target, float feedback, float dt_seconds);
    float updateError(float error, float dt_seconds);

    float getProportional() const { return p_term_; }
    float getIntegral() const     { return i_term_; }
    float getDerivative() const   { return d_term_; }

private:
    float kp_;
    float ki_;
    float kd_;
    float kf_;

    float out_min_;
    float out_max_;
    float int_min_;
    float int_max_;

    float integrator_;
    float prev_error_;
    float filtered_derivative_;

    float p_term_;
    float i_term_;
    float d_term_;
};
