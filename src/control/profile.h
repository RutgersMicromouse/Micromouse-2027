#pragma once

#include <Arduino.h>

// =============================================================================
// Real-Time Trapezoidal Velocity Profile Generator
// Handles both point-to-point stop motions and continuous multi-cell cruising
// =============================================================================

class TrapezoidalProfile {
public:
    TrapezoidalProfile();

    // Start a new linear or angular profile
    void start(float total_distance, float start_speed, float end_speed,
               float max_speed, float acceleration, float deceleration);

    // Update profile state by dt seconds
    void update(float dt_seconds);

    // Use measured travel to close the position loop around a velocity profile.
    void updateWithFeedback(float dt_seconds, float measured_distance);

    // Queries
    float getCurrentPosition() const { return current_pos_; }
    float getCurrentVelocity() const { return current_vel_; }
    float getTotalDistance() const   { return total_distance_; }
    bool isFinished() const          { return is_finished_; }
    bool isDecelerating() const {
        return !is_finished_ && (current_pos_ >= (accel_distance_ + cruise_distance_) ||
               (total_distance_ - current_pos_ <= decel_distance_ + 2.0f));
    }
    float getRemainingDistance() const {
        return max(0.0f, total_distance_ - current_pos_);
    }

    void stopNow();

private:
    float total_distance_;
    float start_speed_;
    float end_speed_;
    float max_speed_;
    float accel_;
    float decel_;

    float current_pos_;
    float current_vel_;

    float accel_distance_;
    float decel_distance_;
    float cruise_distance_;

    bool is_finished_;
    float direction_sign_;
};
