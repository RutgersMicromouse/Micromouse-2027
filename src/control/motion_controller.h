#pragma once

#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "hardware/motors.h"
#include "hardware/encoders.h"
#include "hardware/imu.h"
#include "hardware/ir_sensors.h"
#include "hardware/battery.h"
#include "control/pid.h"
#include "control/profile.h"

// =============================================================================
// Closed-Loop Motion Controller
// Fuses Encoders, Gyroscope IMU, IR Wall Centering, and Motor Drives at 500 Hz
// =============================================================================

class MotionController {
public:
    MotionController();

    // Initialize all sensors and actuators
    bool begin();

    // Periodic control step (called at 500 Hz / every 2ms)
    void update(float dt_seconds = CONTROL_DT_S);

    // High-Level Motion Primitives (Blocking with real-time update loop)
    bool moveForward(float distance_mm, float max_speed = SEARCH_SPEED_MM_S,
                     float end_speed = 0.0f, bool allow_centering = true);

    bool moveForwardCells(int num_cells, float max_speed = SEARCH_SPEED_MM_S,
                          float end_speed = 0.0f, bool allow_centering = true);

    bool turnInPlace(float angle_deg, float turn_speed = TURN_SPEED_DEG_S);

    bool alignFrontWall(float approach_speed = 80.0f, uint16_t timeout_ms = 1500);

    void emergencyStop();

    // Odometry & Pose Queries
    float getTargetHeading() const { return target_heading_deg_; }
    float getCurrentHeading() const { return imu.getHeadingDeg(); }
    float getForwardVelocity() const { return encoders.getForwardSpeedMM_S(); }

    void setTargetHeading(float deg);

private:
    PIDController pid_linear_dist_;
    PIDController pid_linear_vel_;
    PIDController pid_angular_heading_;
    PIDController pid_angular_rate_;

    TrapezoidalProfile linear_profile_;
    TrapezoidalProfile angular_profile_;

    float target_heading_deg_;
    float start_distance_mm_;
    bool centering_enabled_;

    uint32_t last_tick_micros_;
};

extern MotionController motion;
