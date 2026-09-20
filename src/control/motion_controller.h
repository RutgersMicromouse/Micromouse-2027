#pragma once

#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "pid.h"
#include "profile.h"
#include "../hardware/encoders.h"
#include "../hardware/motors.h"
#include "../hardware/ir_sensors.h"
#include "../hardware/imu.h"

class MotionController {
public:
    MotionController(Encoders& encoders, Motors& motors, IRSensors& ir, IMU& imu);
    void begin();

    // Call strictly at 1kHz inside Core 1 task
    void update(float dt_seconds);

    // Command interface
    void executeCommand(const MotionCommand& cmd);
    bool isCommandFinished() const;
    void emergencyStop();

    // Reset reference frames
    void resetTracking();

    // Wall following enable/disable
    void setWallCenteringEnabled(bool enabled);

private:
    Encoders& encoders_;
    Motors& motors_;
    IRSensors& ir_;
    IMU& imu_;

    // Control loops
    PIDController pid_linear_dist_;
    PIDController pid_linear_vel_;
    PIDController pid_angular_heading_;
    PIDController pid_wall_centering_;

    TrapezoidalProfile profile_linear_;
    TrapezoidalProfile profile_angular_;

    MotionCommand active_cmd_;
    bool command_active_;
    bool command_finished_;

    float start_distance_mm_;
    float start_heading_deg_;
    float target_relative_dist_mm_;
    float target_relative_angle_deg_;

    // Continuous unwrapped heading state
    float accumulated_heading_deg_;
    float prev_raw_heading_deg_;

    // Safety: stall detection counter
    uint16_t stall_count_;

    // Alignment state
    uint16_t wall_align_timer_;

    void checkPillarDriftCorrection(float current_dist_mm);

    bool wall_centering_enabled_;
};
