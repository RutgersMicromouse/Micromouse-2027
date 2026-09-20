#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include "config.h"

class Motors {
public:
    Motors();
    void begin();

    // Effort range: -1.0 (full reverse) to +1.0 (full forward)
    void setEffort(float left_effort, float right_effort);

    // Actively brakes both motors
    void brake();

    // Freewheel / coast both motors (0 duty)
    void coast();

    // Enable/disable 12V boost converter power
    void setMotorPowerEnabled(bool enabled);

    // Invert motor polarities if needed
    void setInverted(bool invert_left, bool invert_right);

private:
    MotoronI2C mc_;
    bool invert_left_;
    bool invert_right_;
    bool power_enabled_;
};
