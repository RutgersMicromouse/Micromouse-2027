#include "encoders.h"

WheelEncoders encoders;

WheelEncoders::WheelEncoders()
    : enc_left_(PIN_ENC_L_A, PIN_ENC_L_B),
      enc_right_(PIN_ENC_R_A, PIN_ENC_R_B),
      invert_left_(false),
      invert_right_(true), // Right encoder is typically inverted relative to left on opposing wheels
      prev_left_ticks_(0),
      prev_right_ticks_(0),
      left_speed_mm_s_(0.0f),
      right_speed_mm_s_(0.0f),
      forward_speed_mm_s_(0.0f),
      yaw_rate_deg_s_(0.0f),
      left_dist_mm_(0.0f),
      right_dist_mm_(0.0f) {
}

void WheelEncoders::begin() {
    reset();
    Serial.println("[ENCODERS] Quadrature Encoders initialized on pins (2,3) and (4,5).");
}

void WheelEncoders::reset() {
    enc_left_.write(0);
    enc_right_.write(0);
    prev_left_ticks_ = 0;
    prev_right_ticks_ = 0;
    left_dist_mm_ = 0.0f;
    right_dist_mm_ = 0.0f;
    left_speed_mm_s_ = 0.0f;
    right_speed_mm_s_ = 0.0f;
    forward_speed_mm_s_ = 0.0f;
    yaw_rate_deg_s_ = 0.0f;
}

void WheelEncoders::resetDistance() {
    left_dist_mm_ = 0.0f;
    right_dist_mm_ = 0.0f;
}

void WheelEncoders::update(float dt_seconds) {
    if (dt_seconds <= 0.00001f) return;

    int32_t raw_l = enc_left_.read();
    int32_t raw_r = enc_right_.read();

    if (invert_left_)  raw_l = -raw_l;
    if (invert_right_) raw_r = -raw_r;

    int32_t d_left_ticks  = raw_l - prev_left_ticks_;
    int32_t d_right_ticks = raw_r - prev_right_ticks_;

    prev_left_ticks_  = raw_l;
    prev_right_ticks_ = raw_r;

    // Convert delta ticks to distance in mm
    float d_left_mm  = (float)d_left_ticks  * MM_PER_TICK;
    float d_right_mm = (float)d_right_ticks * MM_PER_TICK;

    left_dist_mm_  += d_left_mm;
    right_dist_mm_ += d_right_mm;

    // Instantaneous velocity (with first-order low-pass smoothing)
    float raw_v_l = d_left_mm  / dt_seconds;
    float raw_v_r = d_right_mm / dt_seconds;

    const float alpha = 0.6f; // Low-pass filter coefficient
    left_speed_mm_s_   = alpha * raw_v_l + (1.0f - alpha) * left_speed_mm_s_;
    right_speed_mm_s_  = alpha * raw_v_r + (1.0f - alpha) * right_speed_mm_s_;
    forward_speed_mm_s_ = 0.5f * (left_speed_mm_s_ + right_speed_mm_s_);

    // Differential heading rate: omega = (v_r - v_l) / L (in rad/s) -> convert to deg/s
    float omega_rad_s = (right_speed_mm_s_ - left_speed_mm_s_) / TRACK_WIDTH_MM;
    yaw_rate_deg_s_   = omega_rad_s * (180.0f / 3.1415926535f);
}

int32_t WheelEncoders::getLeftTicks() const {
    int32_t t = enc_left_.read();
    return invert_left_ ? -t : t;
}

int32_t WheelEncoders::getRightTicks() const {
    int32_t t = enc_right_.read();
    return invert_right_ ? -t : t;
}

float WheelEncoders::getLeftDistanceMM() const {
    return left_dist_mm_;
}

float WheelEncoders::getRightDistanceMM() const {
    return right_dist_mm_;
}

float WheelEncoders::getAverageDistanceMM() const {
    if ((left_dist_mm_ > 0 && right_dist_mm_ < 0) || (left_dist_mm_ < 0 && right_dist_mm_ > 0)) {
        return 0.5f * (fabsf(left_dist_mm_) + fabsf(right_dist_mm_));
    }
    return 0.5f * (left_dist_mm_ + right_dist_mm_);
}

float WheelEncoders::getLeftSpeedMM_S() const {
    return left_speed_mm_s_;
}

float WheelEncoders::getRightSpeedMM_S() const {
    return right_speed_mm_s_;
}

float WheelEncoders::getForwardSpeedMM_S() const {
    return forward_speed_mm_s_;
}

float WheelEncoders::getEncoderYawRateDeg_S() const {
    return yaw_rate_deg_s_;
}

void WheelEncoders::setPolarity(bool invert_left, bool invert_right) {
    invert_left_  = invert_left;
    invert_right_ = invert_right;
}
