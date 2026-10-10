#include "profile.h"
#include "config.h"

TrapezoidalProfile::TrapezoidalProfile()
    : total_distance_(0.0f),
      start_speed_(0.0f),
      end_speed_(0.0f),
      max_speed_(0.0f),
      accel_(0.0f),
      decel_(0.0f),
      current_pos_(0.0f),
      current_vel_(0.0f),
      accel_distance_(0.0f),
      decel_distance_(0.0f),
      cruise_distance_(0.0f),
      is_finished_(true),
      direction_sign_(1.0f) {
}

void TrapezoidalProfile::start(float total_distance, float start_speed, float end_speed,
                               float max_speed, float acceleration, float deceleration) {
    if (fabsf(total_distance) < 0.001f) {
        current_pos_ = total_distance;
        current_vel_ = end_speed;
        is_finished_ = true;
        return;
    }

    direction_sign_ = (total_distance >= 0.0f) ? 1.0f : -1.0f;
    total_distance_ = fabsf(total_distance);
    start_speed_    = fabsf(start_speed);
    end_speed_      = fabsf(end_speed);
    max_speed_      = fabsf(max_speed);
    accel_          = fabsf(acceleration);
    decel_          = fabsf(deceleration);

    if (accel_ < 1.0f) accel_ = 100.0f;
    if (decel_ < 1.0f) decel_ = 100.0f;

    // Minimum distance needed to ramp between start_speed and max_speed:
    // v^2 = u^2 + 2*a*s -> s = (v^2 - u^2) / (2*a)
    float min_accel_dist = (max_speed_ * max_speed_ - start_speed_ * start_speed_) / (2.0f * accel_);
    if (min_accel_dist < 0.0f) min_accel_dist = 0.0f;

    float min_decel_dist = (max_speed_ * max_speed_ - end_speed_ * end_speed_) / (2.0f * decel_);
    if (min_decel_dist < 0.0f) min_decel_dist = 0.0f;

    if (min_accel_dist + min_decel_dist > total_distance_) {
        // Triangular profile: Peak speed cannot be reached
        float numerator = 2.0f * accel_ * decel_ * total_distance_ + decel_ * start_speed_ * start_speed_ + accel_ * end_speed_ * end_speed_;
        float denominator = accel_ + decel_;
        float v_peak_sq = numerator / denominator;
        if (v_peak_sq > 0.0f) {
            max_speed_ = sqrtf(v_peak_sq);
        } else {
            max_speed_ = start_speed_;
        }
        accel_distance_ = (max_speed_ * max_speed_ - start_speed_ * start_speed_) / (2.0f * accel_);
        if (accel_distance_ < 0.0f) accel_distance_ = 0.0f;
        cruise_distance_ = 0.0f;
        decel_distance_ = total_distance_ - accel_distance_;
    } else {
        // Trapezoidal profile: has cruising phase
        accel_distance_ = min_accel_dist;
        decel_distance_ = min_decel_dist;
        cruise_distance_ = total_distance_ - (accel_distance_ + decel_distance_);
    }

    current_pos_ = 0.0f;
    current_vel_ = start_speed_;
    is_finished_ = false;
}

void TrapezoidalProfile::update(float dt_seconds) {
    if (is_finished_) return;

    if (current_pos_ < accel_distance_) {
        // Accelerating
        current_vel_ += accel_ * dt_seconds;
        if (current_vel_ > max_speed_) current_vel_ = max_speed_;
    } else if (current_pos_ < accel_distance_ + cruise_distance_) {
        // Cruising
        current_vel_ = max_speed_;
    } else {
        // Decelerating
        current_vel_ -= decel_ * dt_seconds;
        float floor_speed = (end_speed_ > 0.0f) ? end_speed_ : 0.0f;
        if (current_vel_ < floor_speed) current_vel_ = floor_speed;
    }

    current_pos_ += current_vel_ * dt_seconds;

    if (current_pos_ >= total_distance_ || (end_speed_ <= 0.0f && current_vel_ <= 0.0f && current_pos_ >= (total_distance_ - 10.0f))) {
        current_pos_ = total_distance_;
        current_vel_ = end_speed_;
        is_finished_ = true;
    }
}

void TrapezoidalProfile::updateWithFeedback(float dt_seconds, float measured_distance) {
    if (is_finished_ || dt_seconds <= 0.0f) return;

    current_pos_ = constrain(fabsf(measured_distance), 0.0f, total_distance_);
    const float remaining = total_distance_ - current_pos_;
    if (remaining <= 0.5f || (end_speed_ <= 0.0f && remaining <= MOTION_DISTANCE_TOLERANCE_MM && current_vel_ <= 8.0f)) {
        current_pos_ = total_distance_;
        current_vel_ = end_speed_;
        is_finished_ = true;
        return;
    }

    // Highest speed from which we can still reach the requested end speed over
    // the remaining encoder-measured distance (v^2 = u^2 + 2as).
    const float braking_speed = sqrtf(end_speed_ * end_speed_ + 2.0f * decel_ * remaining);
    const float target_speed = min(max_speed_, braking_speed);
    if (current_vel_ < target_speed) {
        current_vel_ = min(target_speed, current_vel_ + accel_ * dt_seconds);
    } else {
        current_vel_ = max(target_speed, current_vel_ - decel_ * dt_seconds);
    }
}

void TrapezoidalProfile::stopNow() {
    is_finished_ = true;
    current_vel_ = 0.0f;
}
