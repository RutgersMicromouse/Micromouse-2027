#include "profile.h"

TrapezoidalProfile::TrapezoidalProfile()
    : target_total_dist_(0.0f), max_speed_(0.0f), accel_(0.0f),
      v_start_(0.0f), v_peak_(0.0f), v_end_(0.0f),
      current_dist_(0.0f), current_vel_(0.0f),
      d_accel_(0.0f), d_cruise_(0.0f), d_decel_(0.0f),
      t_accel_(0.0f), t_cruise_(0.0f), t_decel_(0.0f), t_total_(0.0f),
      elapsed_time_(0.0f), finished_(true) {}

void TrapezoidalProfile::start(float target_distance, float max_speed, float acceleration,
                               float start_speed, float end_speed) {
    target_total_dist_ = target_distance;
    max_speed_ = fabsf(max_speed);
    accel_ = fabsf(acceleration);
    v_start_ = fabsf(start_speed);
    v_end_ = fabsf(end_speed);

    elapsed_time_ = 0.0f;
    current_dist_ = 0.0f;
    current_vel_ = v_start_;
    finished_ = false;

    if (accel_ < 1.0f) accel_ = 1.0f;
    if (max_speed_ < 1.0f) max_speed_ = 1.0f;
    if (v_start_ > max_speed_) max_speed_ = v_start_;
    if (v_end_ > max_speed_) v_end_ = max_speed_;

    float dist_abs = fabsf(target_distance);
    if (dist_abs < 0.1f) {
        current_vel_ = v_end_;
        finished_ = true;
        return;
    }

    // Distance required to transition v_start -> max_speed and max_speed -> v_end
    float da = fabsf(max_speed_ * max_speed_ - v_start_ * v_start_) / (2.0f * accel_);
    float dd = fabsf(max_speed_ * max_speed_ - v_end_ * v_end_) / (2.0f * accel_);

    if ((da + dd) > dist_abs) {
        // Triangular profile (cannot reach full max_speed)
        float vp_sq = (2.0f * accel_ * dist_abs + v_start_ * v_start_ + v_end_ * v_end_) / 2.0f;
        v_peak_ = sqrtf(fmaxf(0.0f, vp_sq));
        if (v_peak_ > max_speed_) v_peak_ = max_speed_;

        d_accel_ = fabsf(v_peak_ * v_peak_ - v_start_ * v_start_) / (2.0f * accel_);
        d_decel_ = dist_abs - d_accel_;
        d_cruise_ = 0.0f;

        t_accel_ = fabsf(v_peak_ - v_start_) / accel_;
        t_cruise_ = 0.0f;
        t_decel_ = fabsf(v_peak_ - v_end_) / accel_;
        t_total_ = t_accel_ + t_decel_;
    } else {
        // Full trapezoidal profile
        v_peak_ = max_speed_;
        d_accel_ = da;
        d_decel_ = dd;
        d_cruise_ = dist_abs - (da + dd);

        t_accel_ = fabsf(v_peak_ - v_start_) / accel_;
        t_cruise_ = d_cruise_ / max_speed_;
        t_decel_ = fabsf(v_peak_ - v_end_) / accel_;
        t_total_ = t_accel_ + t_cruise_ + t_decel_;
    }
}

void TrapezoidalProfile::update(float dt_seconds) {
    if (finished_) return;

    elapsed_time_ += dt_seconds;
    float sign = (target_total_dist_ >= 0.0f) ? 1.0f : -1.0f;
    float t = elapsed_time_;

    if (t >= t_total_) {
        current_dist_ = target_total_dist_;
        current_vel_ = v_end_ * sign;
        finished_ = true;
        return;
    }

    if (t <= t_accel_) {
        // Phase 1: Acceleration / Initial ramp
        float s_acc = (v_peak_ >= v_start_) ? 1.0f : -1.0f;
        current_vel_ = v_start_ + (s_acc * accel_ * t);
        current_dist_ = (v_start_ * t) + (0.5f * s_acc * accel_ * t * t);
    } else if (t <= (t_accel_ + t_cruise_)) {
        // Phase 2: Cruising at v_peak
        float dt_c = t - t_accel_;
        current_vel_ = v_peak_;
        current_dist_ = d_accel_ + (v_peak_ * dt_c);
    } else {
        // Phase 3: Deceleration to v_end
        float dt_d = t - (t_accel_ + t_cruise_);
        float s_dec = (v_end_ >= v_peak_) ? 1.0f : -1.0f;
        current_vel_ = v_peak_ + (s_dec * accel_ * dt_d);
        current_dist_ = d_accel_ + d_cruise_ + (v_peak_ * dt_d) + (0.5f * s_dec * accel_ * dt_d * dt_d);
    }

    current_dist_ *= sign;
    current_vel_ *= sign;
}

float TrapezoidalProfile::getTargetDistance() const {
    return current_dist_;
}

float TrapezoidalProfile::getTargetVelocity() const {
    return current_vel_;
}

bool TrapezoidalProfile::isFinished() const {
    return finished_;
}

void TrapezoidalProfile::stop() {
    finished_ = true;
    current_vel_ = 0.0f;
}
