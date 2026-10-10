#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>
#include "config.h"

// =============================================================================
// Motor Controller Interface using Pololu Motoron M2T256
// Communication: I2C (Address 16 / 0x10)
// =============================================================================

class MotorController {
public:
    MotorController();

    // Initialize Motoron hardware, reset flags, and configure acceleration limits
    bool begin();

    // Set motor speeds (-MOTOR_MAX_SPEED to +MOTOR_MAX_SPEED)
    // Positive speed drives the robot forward
    void setSpeeds(int16_t left_speed, int16_t right_speed);

    // Set individual motor speeds
    void setLeftSpeed(int16_t speed);
    void setRightSpeed(int16_t speed);

    // Apply a conservative runtime boost to the slower wheel based on encoder
    // rates measured during the secured startup balance test.
    bool calibrateWheelSpeedBalance(float left_ticks_per_second,
                                    float right_ticks_per_second);

    // Stop both motors immediately (active brake or coast)
    void stop(bool brake = true);

    // Check if Motoron communication is alive
    bool isConnected();

    // Read Motoron status flags for hardware diagnostics.
    uint16_t getStatusFlags();
    int16_t getLeftCurrentSpeed();
    int16_t getRightCurrentSpeed();

    // Reset error flags if any
    void clearErrors();

private:
    MotoronI2C motoron_;
    bool is_initialized_;
    uint32_t last_command_time_;

    int16_t prev_left_cmd_;
    int16_t prev_right_cmd_;
    uint32_t left_kick_end_ms_;
    uint32_t right_kick_end_ms_;
    float left_speed_compensation_;
    float right_speed_compensation_;

    int16_t processChannelCommand(int16_t command, float compensation,
                                  int16_t& prev_cmd, uint32_t& kick_end_ms,
                                  int16_t startup_offset,
                                  int16_t max_kick = MOTOR_KICKSTART_PWM);
};

extern MotorController motors;
