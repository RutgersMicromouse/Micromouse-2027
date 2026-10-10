#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>
#include "config.h"
#include "types.h"

// =============================================================================
// Pololu MinIMU-9 v5 (LSM6DS33 6-DOF IMU) Interface
// Communication: I2C (Address 0x6B / 0x6A)
// =============================================================================

class IMUDriver {
public:
    IMUDriver();

    // Initialize LSM6DS33 sensor, configure full-scale ranges and ODR
    bool begin();

    // Zero-rate calibration while robot is stationary
    void calibrateStaticBias(uint16_t sample_count = 300);

    // High frequency update (called every control tick, e.g. 500 Hz)
    void update(float dt_seconds, float encoder_yaw_rate, float linear_speed_mm_s);

    // Get current heading in degrees (-180.0 to +180.0)
    float getHeadingDeg() const;

    // Get current heading in radians
    float getHeadingRad() const;

    // Get calibrated yaw rate in degrees per second
    float getYawRateDeg_S() const;

    float getRawYawRateDeg_S() const;
    float getGyroBiasDeg_S() const;

    // Reset heading to a specific angle (e.g. 0.0 or 90.0)
    void resetHeading(float new_heading_deg = 0.0f);

    // Check if hardware is detected and functional
    bool isConnected() const;

    // Retrieve full telemetry reading
    IMUReading getReading() const;

private:
    LSM6 lsm6_;
    bool is_initialized_;

    float gyro_scale_dps_;
    float accel_scale_g_;

    float gyro_bias_z_;
    float raw_yaw_rate_dps_;
    float heading_deg_;
    float yaw_rate_dps_;

    float accel_x_g_;
    float accel_y_g_;

    uint8_t yaw_axis_;
    float yaw_sign_;

    uint32_t poll_tick_;
};

extern IMUDriver imu;
