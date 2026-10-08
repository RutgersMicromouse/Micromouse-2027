#pragma once

// Generic PID loop

#include "config.h"
#include "types.h"

// ==============================================================================
// PID CONTROLLER
// ==============================================================================

#include <Arduino.h>

class PIDController {
public:
    PIDController();
    PIDController(float kp, float ki, float kd, float max_out, float max_integral = 0.0f);

    void setGains(float kp, float ki, float kd);
    void getGains(float& kp, float& ki, float& kd) const { kp = kp_; ki = ki_; kd = kd_; }
    void setOutputLimits(float max_out, float max_integral = 0.0f);
    void reset();

    // Compute PID output given current error and time delta
    float update(float error, float dt_seconds);

private:
    float kp_;
    float ki_;
    float kd_;
    float max_output_;
    float max_integral_;

    float integral_;
    float prev_error_;
    float prev_derivative_;
    bool first_run_;
};
