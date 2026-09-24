#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "types.h"

class IMU {
public:
    IMU();
    void begin();

    // Call inside control loop
    void update(float dt_seconds, float encoder_yaw_rate = 0.0f, float linear_speed_mm_s = 0.0f);

    IMUState getState() const;
    float getHeadingDeg() const;
    float getGyroZ() const;
    void resetHeading(float initial_heading_deg = 0.0f);
    bool isHardwareConnected() const;

private:
    bool initBNO055();
    bool readBNO055Data(float& heading_deg, float& gyro_z);

    IMUState state_;
    bool hardware_detected_;
    float heading_offset_deg_;
    float gyro_bias_z_;
    uint32_t poll_counter_;
};
