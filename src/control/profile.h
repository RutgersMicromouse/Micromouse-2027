#pragma once

#include <Arduino.h>

class TrapezoidalProfile {
public:
    TrapezoidalProfile();

    // Start a new trapezoidal movement trajectory with optional initial and final boundary speeds
    void start(float target_distance, float max_speed, float acceleration,
               float start_speed = 0.0f, float end_speed = 0.0f);

    // Compute target position and velocity for the current time step
    void update(float dt_seconds);

    float getTargetDistance() const;
    float getTargetVelocity() const;
    bool isFinished() const;
    void stop();

private:
    float target_total_dist_;
    float max_speed_;
    float accel_;
    float v_start_;
    float v_peak_;
    float v_end_;

    float current_dist_;
    float current_vel_;

    float d_accel_;
    float d_cruise_;
    float d_decel_;

    float t_accel_;
    float t_cruise_;
    float t_decel_;
    float t_total_;
    float elapsed_time_;

    bool finished_;
};
