#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include "config.h"

class Motors {
public:
    Motors();
    void begin();

    // Effort range: -1.0 (full reverse) to +1.0 (full forward) with trim applied
    void setEffort(float left_effort, float right_effort);

    // Raw effort without trim multipliers applied (used for calibration)
    void setRawEffort(float left_effort, float right_effort);

    // Motor balance trim (Left and Right multipliers)
    void setTrim(float trim_left, float trim_right);
    void getTrim(float& trim_left, float& trim_right) const;

    // Minimum breakaway duty cycle deadbands
    void setDeadband(float deadband_left, float deadband_right);
    void getDeadband(float& deadband_left, float& deadband_right) const;

    // Flash NVS storage for motor balance calibration
    void saveToNVS();
    bool loadFromNVS();

    // Actively brakes both motors
    void brake();

    // Freewheel / coast both motors (0 duty)
    void coast();

    // Enable/disable 12V boost converter power
    void setMotorPowerEnabled(bool enabled);

    // Invert motor polarities if needed
    void setInverted(bool invert_left, bool invert_right);
    void getInverted(bool& invert_left, bool& invert_right) const;

private:
    MotoronI2C mc_;
    bool invert_left_;
    bool invert_right_;
    bool power_enabled_;
    float trim_left_;
    float trim_right_;
    float deadband_left_;
    float deadband_right_;

    // I2C bus bandwidth optimization & state tracking
    bool is_braking_;
    int16_t last_left_cmd_;
    int16_t last_right_cmd_;
    uint32_t last_cmd_time_ms_;
};
