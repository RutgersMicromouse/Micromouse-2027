#include "motion_controller.h"

MotionController::MotionController(Encoders& encoders, Motors& motors, IRSensors& ir, IMU& imu)
    : encoders_(encoders), motors_(motors), ir_(ir), imu_(imu),
      command_active_(false), command_finished_(true),
      start_distance_mm_(0.0f), start_heading_deg_(0.0f),
      target_relative_dist_mm_(0.0f), target_relative_angle_deg_(0.0f),
      accumulated_heading_deg_(0.0f), prev_raw_heading_deg_(0.0f),
      stall_count_(0), wall_align_timer_(0),
      wall_centering_enabled_(true) {

    // PID Gains (Default baseline tuning for micromouse kinematics; adjust as needed)
    // Linear Distance PID: outputs target speed (mm/s)
    pid_linear_dist_.setGains(3.5f, 0.0f, 0.1f);
    pid_linear_dist_.setOutputLimits(800.0f, 200.0f);

    // Linear Velocity PID: outputs motor duty cycle (-1.0 to 1.0)
    pid_linear_vel_.setGains(0.0025f, 0.0005f, 0.00005f);
    pid_linear_vel_.setOutputLimits(1.0f, 0.3f);

    // Angular Heading PID: outputs rotational effort (-1.0 to 1.0)
    pid_angular_heading_.setGains(0.025f, 0.001f, 0.0008f);
    pid_angular_heading_.setOutputLimits(0.6f, 0.2f);

    // Wall Centering PD: injects angle offset (degrees) based on IR centering error
    pid_wall_centering_.setGains(15.0f, 0.0f, 2.0f);
    pid_wall_centering_.setOutputLimits(20.0f, 5.0f); // Max 20 degrees steering trim
}

void MotionController::begin() {
    resetTracking();
}

void MotionController::resetTracking() {
    encoders_.reset();
    imu_.resetHeading(0.0f);
    pid_linear_dist_.reset();
    pid_linear_vel_.reset();
    pid_angular_heading_.reset();
    pid_wall_centering_.reset();
    command_active_ = false;
    command_finished_ = true;
    start_distance_mm_ = 0.0f;
    start_heading_deg_ = 0.0f;
    accumulated_heading_deg_ = 0.0f;
    prev_raw_heading_deg_ = imu_.getHeadingDeg();
    stall_count_ = 0;
    wall_align_timer_ = 0;
    motors_.coast();
}

void MotionController::executeCommand(const MotionCommand& cmd) {
    active_cmd_ = cmd;
    command_finished_ = false;
    command_active_ = true;
    stall_count_ = 0;
    wall_align_timer_ = 0;

    EncoderState enc = encoders_.getState();
    start_distance_mm_ = (enc.left_dist_mm + enc.right_dist_mm) * 0.5f;
    start_heading_deg_ = accumulated_heading_deg_;

    switch (cmd.action) {
        case ACTION_MOVE_FORWARD_CELLS: {
            target_relative_dist_mm_ = cmd.param_value * MAZE_CELL_SIZE_MM;
            target_relative_angle_deg_ = 0.0f;
            profile_linear_.start(target_relative_dist_mm_, cmd.max_speed_mm_s, cmd.acceleration,
                                  cmd.entry_speed_mm_s, cmd.exit_speed_mm_s);
            profile_angular_.stop();
            break;
        }

        case ACTION_MOVE_DISTANCE: {
            target_relative_dist_mm_ = cmd.param_value;
            target_relative_angle_deg_ = 0.0f;
            profile_linear_.start(target_relative_dist_mm_, cmd.max_speed_mm_s, cmd.acceleration,
                                  cmd.entry_speed_mm_s, cmd.exit_speed_mm_s);
            profile_angular_.stop();
            break;
        }

        case ACTION_MOVE_HALF_CELL: {
            target_relative_dist_mm_ = HALF_CELL_SIZE_MM; // 90.0 mm
            target_relative_angle_deg_ = 0.0f;
            profile_linear_.start(target_relative_dist_mm_, cmd.max_speed_mm_s, cmd.acceleration,
                                  cmd.entry_speed_mm_s, cmd.exit_speed_mm_s);
            profile_angular_.stop();
            break;
        }

        case ACTION_MOVE_DIAGONAL_HALF: {
            // 1 diagonal half-step = sqrt(2)/2 * 180mm = 90 * sqrt(2) = 127.27922 mm
            target_relative_dist_mm_ = cmd.param_value * (HALF_CELL_SIZE_MM * 1.41421356f);
            target_relative_angle_deg_ = 0.0f;
            profile_linear_.start(target_relative_dist_mm_, cmd.max_speed_mm_s, cmd.acceleration,
                                  cmd.entry_speed_mm_s, cmd.exit_speed_mm_s);
            profile_angular_.stop();
            break;
        }

        case ACTION_TURN_LEFT_90: {
            target_relative_dist_mm_ = 0.0f;
            target_relative_angle_deg_ = 90.0f; // CCW
            profile_angular_.start(90.0f, cmd.max_speed_mm_s, cmd.acceleration);
            profile_linear_.stop();
            break;
        }

        case ACTION_TURN_RIGHT_90: {
            target_relative_dist_mm_ = 0.0f;
            target_relative_angle_deg_ = -90.0f; // CW
            profile_angular_.start(-90.0f, cmd.max_speed_mm_s, cmd.acceleration);
            profile_linear_.stop();
            break;
        }

        case ACTION_TURN_LEFT_45: {
            target_relative_dist_mm_ = 0.0f;
            target_relative_angle_deg_ = 45.0f; // CCW
            profile_angular_.start(45.0f, cmd.max_speed_mm_s, cmd.acceleration);
            profile_linear_.stop();
            break;
        }

        case ACTION_TURN_RIGHT_45: {
            target_relative_dist_mm_ = 0.0f;
            target_relative_angle_deg_ = -45.0f; // CW
            profile_angular_.start(-45.0f, cmd.max_speed_mm_s, cmd.acceleration);
            profile_linear_.stop();
            break;
        }

        case ACTION_CURVE_LEFT_90: {
            float R = (cmd.param_value > 10.0f) ? cmd.param_value : 80.0f;
            float arc_dist = R * (3.14159265f / 2.0f);
            target_relative_dist_mm_ = arc_dist;
            target_relative_angle_deg_ = 90.0f;
            float v = cmd.max_speed_mm_s > 0.0f ? cmd.max_speed_mm_s : 750.0f;
            float omega = (90.0f / arc_dist) * v;
            profile_linear_.start(arc_dist, v, cmd.acceleration, cmd.entry_speed_mm_s, cmd.exit_speed_mm_s);
            profile_angular_.start(90.0f, omega, cmd.acceleration * (90.0f / arc_dist));
            break;
        }

        case ACTION_CURVE_RIGHT_90: {
            float R = (cmd.param_value > 10.0f) ? cmd.param_value : 80.0f;
            float arc_dist = R * (3.14159265f / 2.0f);
            target_relative_dist_mm_ = arc_dist;
            target_relative_angle_deg_ = -90.0f;
            float v = cmd.max_speed_mm_s > 0.0f ? cmd.max_speed_mm_s : 750.0f;
            float omega = (90.0f / arc_dist) * v;
            profile_linear_.start(arc_dist, v, cmd.acceleration, cmd.entry_speed_mm_s, cmd.exit_speed_mm_s);
            profile_angular_.start(-90.0f, omega, cmd.acceleration * (90.0f / arc_dist));
            break;
        }

        case ACTION_CURVE_LEFT_45: {
            float R = (cmd.param_value > 10.0f) ? cmd.param_value : 80.0f;
            float arc_dist = R * (3.14159265f / 4.0f);
            target_relative_dist_mm_ = arc_dist;
            target_relative_angle_deg_ = 45.0f;
            float v = cmd.max_speed_mm_s > 0.0f ? cmd.max_speed_mm_s : 800.0f;
            float omega = (45.0f / arc_dist) * v;
            profile_linear_.start(arc_dist, v, cmd.acceleration, cmd.entry_speed_mm_s, cmd.exit_speed_mm_s);
            profile_angular_.start(45.0f, omega, cmd.acceleration * (45.0f / arc_dist));
            break;
        }

        case ACTION_CURVE_RIGHT_45: {
            float R = (cmd.param_value > 10.0f) ? cmd.param_value : 80.0f;
            float arc_dist = R * (3.14159265f / 4.0f);
            target_relative_dist_mm_ = arc_dist;
            target_relative_angle_deg_ = -45.0f;
            float v = cmd.max_speed_mm_s > 0.0f ? cmd.max_speed_mm_s : 800.0f;
            float omega = (45.0f / arc_dist) * v;
            profile_linear_.start(arc_dist, v, cmd.acceleration, cmd.entry_speed_mm_s, cmd.exit_speed_mm_s);
            profile_angular_.start(-45.0f, omega, cmd.acceleration * (45.0f / arc_dist));
            break;
        }

        case ACTION_TURN_AROUND_180: {
            target_relative_dist_mm_ = 0.0f;
            target_relative_angle_deg_ = 180.0f;
            profile_angular_.start(180.0f, cmd.max_speed_mm_s, cmd.acceleration);
            profile_linear_.stop();
            break;
        }

        case ACTION_ALIGN_FRONT_WALL: {
            target_relative_dist_mm_ = (cmd.param_value > 0.0f) ? cmd.param_value : 80.0f;
            target_relative_angle_deg_ = 0.0f;
            profile_linear_.start(target_relative_dist_mm_, 70.0f, 600.0f);
            profile_angular_.stop();
            wall_align_timer_ = 0;
            break;
        }

        case ACTION_EMERGENCY_STOP:
        default:
            emergencyStop();
            break;
    }
}

void MotionController::checkPillarDriftCorrection(float current_dist_mm) {
    if (active_cmd_.action != ACTION_MOVE_FORWARD_CELLS && active_cmd_.action != ACTION_MOVE_DISTANCE) {
        return;
    }
    IRReadings ir = ir_.getReadings();
    if (!ir.post_edge_left && !ir.post_edge_right) {
        return;
    }
    float dist_in_seg = current_dist_mm - start_distance_mm_;
    if (dist_in_seg < 30.0f) return;

    float phase = fmodf(dist_in_seg, MAZE_CELL_SIZE_MM);
    float expected_post = HALF_CELL_SIZE_MM; // 90mm post position
    float drift = phase - expected_post;

    if (fabsf(drift) <= 25.0f) {
        // Eliminate longitudinal wheel slip error
        start_distance_mm_ += drift;
    }
}

void MotionController::update(float dt_seconds) {
    if (!command_active_) {
        motors_.brake();
        return;
    }

    EncoderState enc = encoders_.getState();
    float current_distance = (enc.left_dist_mm + enc.right_dist_mm) * 0.5f;

    // 1. Continuous unwrapped heading update
    float raw_heading = imu_.getHeadingDeg();
    float d_h = raw_heading - prev_raw_heading_deg_;
    while (d_h > 180.0f)  d_h -= 360.0f;
    while (d_h <= -180.0f) d_h += 360.0f;
    prev_raw_heading_deg_ = raw_heading;
    accumulated_heading_deg_ += d_h;

    // Check pillar post edge detection to correct distance drift
    checkPillarDriftCorrection(current_distance);

    float forward_effort = 0.0f;
    float rotational_effort = 0.0f;

    // 2. Linear Profile & Controller
    if (!profile_linear_.isFinished()) {
        profile_linear_.update(dt_seconds);

        float dist_traveled = current_distance - start_distance_mm_;
        float target_dist = profile_linear_.getTargetDistance();
        float target_vel = profile_linear_.getTargetVelocity();

        // Distance error
        float dist_err = target_dist - dist_traveled;
        float vel_setpoint = target_vel + pid_linear_dist_.update(dist_err, dt_seconds);

        // Velocity error
        float vel_err = vel_setpoint - enc.linear_speed_mm_s;
        forward_effort = pid_linear_vel_.update(vel_err, dt_seconds);
    }

    // 3. Angular Profile & Controller (Continuous unwrapped heading)
    float heading_traveled = accumulated_heading_deg_ - start_heading_deg_;
    float target_heading_setpoint = 0.0f;

    if (!profile_angular_.isFinished()) {
        profile_angular_.update(dt_seconds);
        target_heading_setpoint = profile_angular_.getTargetDistance();
    } else {
        target_heading_setpoint = target_relative_angle_deg_;

        // Wall Centering steering injection during straight runs
        if (wall_centering_enabled_ && active_cmd_.enable_wall_centering &&
            (active_cmd_.action == ACTION_MOVE_FORWARD_CELLS || active_cmd_.action == ACTION_MOVE_DISTANCE || active_cmd_.action == ACTION_MOVE_HALF_CELL)) {
            
            float centering_err = ir_.getCenteringError();
            // Negative sign: if mouse is too close to left wall, steer right (negative angle trim)
            float wall_trim_angle = -pid_wall_centering_.update(centering_err, dt_seconds);
            target_heading_setpoint += wall_trim_angle;
        }
    }

    float heading_err = target_heading_setpoint - heading_traveled;
    // Shortest-path angular error normalization
    while (heading_err > 180.0f)  heading_err -= 360.0f;
    while (heading_err <= -180.0f) heading_err += 360.0f;
    rotational_effort = pid_angular_heading_.update(heading_err, dt_seconds);

    // 4. Handle ACTION_ALIGN_FRONT_WALL touch detection
    if (active_cmd_.action == ACTION_ALIGN_FRONT_WALL) {
        IRReadings ir_now = ir_.getReadings();
        if (ir_now.front_center > 1300 ||
            (profile_linear_.getTargetVelocity() > 20.0f && fabsf(enc.linear_speed_mm_s) < 8.0f && profile_linear_.getTargetDistance() > 15.0f)) {
            wall_align_timer_++;
            if (wall_align_timer_ > 15) { // 30ms stable wall touch
                profile_linear_.stop();
                command_finished_ = true;
                command_active_ = false;
                motors_.brake();
                accumulated_heading_deg_ = 0.0f;
                start_heading_deg_ = 0.0f;
                imu_.resetHeading(0.0f);
                encoders_.reset();
                start_distance_mm_ = 0.0f;
                return;
            }
        }
    }

    // 5. Motor Stall Detection & Coil Protection
    float cmd_vel = profile_linear_.getTargetVelocity();
    if (command_active_ && fabsf(cmd_vel) > 80.0f && fabsf(enc.linear_speed_mm_s) < 15.0f &&
        (fabsf(forward_effort) > 0.35f || fabsf(rotational_effort) > 0.40f)) {
        stall_count_++;
        if (stall_count_ > 100) { // 100 ticks @ 500 Hz = 200 ms
            Serial.println("\n[SAFETY ALERT] MOTOR STALL DETECTED! Emergency stop engaged to protect motors.");
            emergencyStop();
            stall_count_ = 0;
            return;
        }
    } else {
        stall_count_ = 0;
    }

    // 6. Actuator Mixer
    float left_duty  = forward_effort - rotational_effort;
    float right_duty = forward_effort + rotational_effort;

    motors_.setEffort(left_duty, right_duty);

    // 7. Completion check
    if (profile_linear_.isFinished() && profile_angular_.isFinished()) {
        command_finished_ = true;
        command_active_ = false;
        // Brake if stopping, or maintain velocity if exit speed was requested
        if (active_cmd_.exit_speed_mm_s <= 10.0f) {
            motors_.brake();
        }
    }
}

bool MotionController::isCommandFinished() const {
    return command_finished_;
}

void MotionController::emergencyStop() {
    profile_linear_.stop();
    profile_angular_.stop();
    command_active_ = false;
    command_finished_ = true;
    motors_.brake();
}

void MotionController::setWallCenteringEnabled(bool enabled) {
    wall_centering_enabled_ = enabled;
}
