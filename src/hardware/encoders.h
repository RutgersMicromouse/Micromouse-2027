#pragma once

#include <Arduino.h>
#include <Encoder.h>
#include "config.h"

// =============================================================================
// Optical / Magnetic Quadrature Wheel Encoders
// Pins: Left=(2, 3), Right=(4, 5)
// Library: PaulStoffregen/Encoder
// =============================================================================

class WheelEncoders {
public:
    WheelEncoders();

    void begin();

    // Reset encoder counts and accumulator
    void reset();

    // Reset distance counters to zero
    void resetDistance();

    // High frequency update (called every control tick, e.g. 500 Hz)
    void update(float dt_seconds);

    // Raw tick access
    int32_t getLeftTicks() const;
    int32_t getRightTicks() const;

    // Linear distance traveled in millimeters
    float getLeftDistanceMM() const;
    float getRightDistanceMM() const;
    float getAverageDistanceMM() const;

    // Instantaneous velocities
    float getLeftSpeedMM_S() const;
    float getRightSpeedMM_S() const;
    float getForwardSpeedMM_S() const;

    // Differential heading rate estimated purely from wheel speeds (deg/s)
    float getEncoderYawRateDeg_S() const;

    // Polarity configuration if motor/gearbox phase is flipped
    void setPolarity(bool invert_left, bool invert_right);

private:
    mutable Encoder enc_left_;
    mutable Encoder enc_right_;

    bool invert_left_;
    bool invert_right_;

    int32_t prev_left_ticks_;
    int32_t prev_right_ticks_;

    float left_speed_mm_s_;
    float right_speed_mm_s_;
    float forward_speed_mm_s_;
    float yaw_rate_deg_s_;

    float left_dist_mm_;
    float right_dist_mm_;
};

extern WheelEncoders encoders;
