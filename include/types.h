#pragma once

#include <stdint.h>
#include "maze_constants.h"

// =============================================================================
// Common Types & Data Structures for Ratatouieee
// =============================================================================

struct Coordinate {
    int8_t x;
    int8_t y;

    bool operator==(const Coordinate& other) const {
        return x == other.x && y == other.y;
    }
    bool operator!=(const Coordinate& other) const {
        return !(*this == other);
    }
};

struct Pose {
    float x_mm;
    float y_mm;
    float theta_deg; // 0 to 360 degrees (North = 90 or 0, counter-clockwise)
    float theta_rad;
};

struct DistanceSensors {
    // Raw ADC readings (0-1023)
    uint16_t front;     // FIR (Pin 17)
    uint16_t left_45;   // L1IR (Pin 16)
    uint16_t left_90;   // L2IR (Pin 15)
    uint16_t right_45;  // R1IR (Pin 14)
    uint16_t right_90;  // R2IR (Pin 20)

    // Calibrated physical distance in millimeters (Sharp GP2Y0A51SK0F)
    float front_mm;
    float left_45_mm;
    float left_90_mm;
    float right_45_mm;
    float right_90_mm;

    // Binary wall presence classification
    bool wall_front;
    bool wall_left;
    bool wall_right;

    // Lateral steering deviation (-1.0 to +1.0)
    // Positive means robot is too close to left wall -> steer right
    // Negative means robot is too close to right wall -> steer left
    float centering_error;
};

struct IMUReading {
    float gyro_z_dps;    // Yaw rate in degrees per second
    float accel_x_g;     // Forward linear acceleration
    float accel_y_g;     // Lateral acceleration
    float heading_deg;   // Integrated yaw heading (-180 to +180)
    bool is_ready;
};

enum RobotState : uint8_t {
    STATE_IDLE = 0,
    STATE_CALIBRATION,
    STATE_EXPLORE_TO_CENTER,
    STATE_EXPLORE_TO_START,
    STATE_SPEED_RUN,
    STATE_DIAGNOSTIC_STREAM
};

enum TurnType : uint8_t {
    TURN_NONE = 0,
    TURN_LEFT_90,
    TURN_RIGHT_90,
    TURN_AROUND_180,
    TURN_LEFT_45,
    TURN_RIGHT_45
};

struct MotionProfileState {
    float target_pos;      // Target position (mm or deg)
    float current_pos;     // Current feedback position
    float target_vel;      // Profiled current target velocity
    float max_vel;         // Peak velocity limit
    float max_accel;       // Acceleration limit
    float max_decel;       // Deceleration limit
    bool is_finished;
};
