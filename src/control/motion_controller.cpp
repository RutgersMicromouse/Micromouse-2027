#include "motion_controller.h"
#include "math_utils.h"

MotionController::MotionController(Encoders& encoders, Motors& motors, IRSensors& ir, IMU& imu)
    : encoders_(encoders), motors_(motors), ir_(ir), imu_(imu),
      command_active_(false), command_finished_(true),
      start_distance_mm_(0.0f), start_heading_deg_(0.0f),
      target_relative_dist_mm_(0.0f), target_relative_angle_deg_(0.0f),
      accumulated_heading_deg_(0.0f), prev_raw_heading_deg_(0.0f),
      stall_count_(0), wall_align_timer_(0), chained_coast_timer_(0),
      wall_centering_enabled_(true), calibrating_motors_(false),
      k_wheel_sync_(0.0004f) {

    // PID Gains (Default baseline tuning for micromouse kinematics; adjust as needed)
    // Linear Distance PID: outputs target speed (mm/s)
    pid_linear_dist_.setGains(3.5f, 0.0f, 0.1f);
    pid_linear_dist_.setOutputLimits(SPEEDRUN_DIAG_SPEED_MM_S, 200.0f);

    // Linear Velocity PID: outputs motor duty cycle (-1.0 to 1.0)
    pid_linear_vel_.setGains(0.0025f, 0.0005f, 0.00005f);
    pid_linear_vel_.setOutputLimits(1.0f, 0.3f);

    // Angular Heading PID: outputs rotational effort (-1.0 to 1.0)
    // Ki = 0.025 neutralizes motor/gearbox mechanical friction asymmetry within 150ms
    pid_angular_heading_.setGains(0.045f, 0.025f, 0.0012f);
    pid_angular_heading_.setOutputLimits(0.6f, 0.12f);

    // Wall Centering PD: injects angle offset (degrees) based on IR centering error
    pid_wall_centering_.setGains(35.0f, 0.0f, 3.5f);
    pid_wall_centering_.setOutputLimits(25.0f, 5.0f); // Max 25 degrees steering trim
}

void MotionController::begin() {
    loadFromNVS();
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
    chained_coast_timer_ = 0;
    motors_.coast();
}

void MotionController::executeCommand(const MotionCommand& cmd) {
    active_cmd_ = cmd;
    command_finished_ = false;
    command_active_ = true;
    stall_count_ = 0;
    wall_align_timer_ = 0;
    chained_coast_timer_ = 0;

    EncoderState enc = encoders_.getState();
    start_distance_mm_ = (enc.left_dist_mm + enc.right_dist_mm) * 0.5f;

    // Cardinal & Diagonal Grid Axis Snapping:
    // Eliminates turn exit residual errors by snapping target heading strictly to the maze grid axis
    if (cmd.action == ACTION_MOVE_FORWARD_CELLS || cmd.action == ACTION_MOVE_DISTANCE || cmd.action == ACTION_MOVE_HALF_CELL) {
        float nearest_cardinal = roundf(accumulated_heading_deg_ / 90.0f) * 90.0f;
        if (fabsf(accumulated_heading_deg_ - nearest_cardinal) < 25.0f) {
            start_heading_deg_ = nearest_cardinal; // Snap to pure 0°, 90°, 180°, or -90°
        } else {
            start_heading_deg_ = accumulated_heading_deg_;
        }
    } else if (cmd.action == ACTION_MOVE_DIAGONAL_HALF) {
        float nearest_diagonal = roundf(accumulated_heading_deg_ / 45.0f) * 45.0f;
        if (fabsf(accumulated_heading_deg_ - nearest_diagonal) < 18.0f) {
            start_heading_deg_ = nearest_diagonal; // Snap to pure 45°, 135°, -45°, or -135°
        } else {
            start_heading_deg_ = accumulated_heading_deg_;
        }
    } else {
        start_heading_deg_ = accumulated_heading_deg_;
    }

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
            float v = cmd.max_speed_mm_s > 0.0f ? cmd.max_speed_mm_s : SPEEDRUN_CURVE_SPEED_MM_S;
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
            float v = cmd.max_speed_mm_s > 0.0f ? cmd.max_speed_mm_s : SPEEDRUN_CURVE_SPEED_MM_S;
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

        case ACTION_SQUARE_FRONT_OPTICAL: {
            target_relative_dist_mm_ = 0.0f;
            target_relative_angle_deg_ = 0.0f;
            profile_linear_.stop();
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
    // Correct distance drift on both falling edges (leaving walls) and rising edges (entering walls)
    if (!ir.post_edge_left && !ir.post_edge_right && !ir.post_rising_left && !ir.post_rising_right) {
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
    if (calibrating_motors_) {
        return; // Motion controller paused for motor calibration or bench testing
    }

    if (!command_active_) {
        // Grace period for chained motions (e.g., straight sprint into continuous curve)
        if (active_cmd_.exit_speed_mm_s > 10.0f && chained_coast_timer_ < 35) { // 35 ticks @ 500 Hz = 70 ms
            chained_coast_timer_++;
            EncoderState enc = encoders_.getState();
            float vel_err = active_cmd_.exit_speed_mm_s - enc.linear_speed_mm_s;
            float forward_effort = pid_linear_vel_.update(vel_err, dt_seconds);
            motors_.setEffort(forward_effort, forward_effort);
            return;
        }
        motors_.brake();
        return;
    }

    EncoderState enc = encoders_.getState();
    float current_distance = (enc.left_dist_mm + enc.right_dist_mm) * 0.5f;

    // 1. Continuous unwrapped heading update
    float raw_heading = imu_.getHeadingDeg();
    float d_h = normalizeAngle180(raw_heading - prev_raw_heading_deg_);
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
            if (fabsf(centering_err) > 0.001f) {
                // Negative sign: if mouse is too close to left wall, steer right (negative angle trim)
                float wall_trim_angle = -pid_wall_centering_.update(centering_err, dt_seconds);

                // Gyro yaw rate damping: prevents overshooting and snaking during aggressive centering
                float gyro_z = imu_.getGyroZ(); // deg/s (CCW positive)
                wall_trim_angle -= (gyro_z * 0.035f);

                target_heading_setpoint += wall_trim_angle;
            } else {
                pid_wall_centering_.reset();
            }
        }
    }

    // Shortest-path angular error normalization in (-180, +180]
    float heading_err = shortestAngularDifference(target_heading_setpoint, heading_traveled);
    rotational_effort = pid_angular_heading_.update(heading_err, dt_seconds);

    // Differential Wheel Speed Lock: actively prevents wheel speed divergence during straight lines
    if (active_cmd_.action == ACTION_MOVE_FORWARD_CELLS || active_cmd_.action == ACTION_MOVE_DISTANCE ||
        active_cmd_.action == ACTION_MOVE_HALF_CELL || active_cmd_.action == ACTION_MOVE_DIAGONAL_HALF) {
        float speed_diff = enc.right_speed_mm_s - enc.left_speed_mm_s;
        rotational_effort -= (speed_diff * k_wheel_sync_);
    }

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

    // 4b. Handle ACTION_SQUARE_FRONT_OPTICAL contactless squaring using FL and FR sensors
    if (active_cmd_.action == ACTION_SQUARE_FRONT_OPTICAL) {
        IRReadings ir_now = ir_.getReadings();

        // Safety: If no front wall is detected, finish immediately
        if (!ir_now.wall_front || ir_now.front_center < 200) {
            command_finished_ = true;
            command_active_ = false;
            motors_.brake();
            return;
        }

        float fl = (float)ir_now.front_left;
        float fr = (float)ir_now.front_right;
        float avg_f = (fl + fr) * 0.5f;

        // Normalized difference: positive means FL > FR (tilted CW -> pivot CCW/left)
        float norm_diff = (avg_f > 50.0f) ? ((fl - fr) / avg_f) : 0.0f;

        // Within 4% symmetry = perfectly perpendicular to front wall
        if (fabsf(norm_diff) < 0.04f) {
            wall_align_timer_++;
            motors_.brake();

            if (wall_align_timer_ >= 25) { // 25 ticks @ 500 Hz = 50 ms of stable symmetry
                command_finished_ = true;
                command_active_ = false;
                motors_.brake();

                // Recalibrate heading baseline to exact 0.0°
                accumulated_heading_deg_ = 0.0f;
                start_heading_deg_ = 0.0f;
                imu_.resetHeading(0.0f);
                pid_angular_heading_.reset();
                pid_wall_centering_.reset();
                return;
            }
        } else {
            wall_align_timer_ = 0;
            // Proportional rotation effort with gyro damping to null disparity
            float rot_effort = norm_diff * 0.35f;
            float gz = imu_.getGyroZ();
            rot_effort -= (gz * 0.002f);

            // Friction breakout minimum effort
            if (rot_effort > 0.0f && rot_effort < 0.08f) rot_effort = 0.08f;
            if (rot_effort < 0.0f && rot_effort > -0.08f) rot_effort = -0.08f;

            if (rot_effort > 0.22f)  rot_effort = 0.22f;
            if (rot_effort < -0.22f) rot_effort = -0.22f;

            motors_.setEffort(-rot_effort, rot_effort);
        }

        // Timeout safety (250 ticks = 500 ms)
        stall_count_++;
        if (stall_count_ > 250) {
            command_finished_ = true;
            command_active_ = false;
            motors_.brake();
            accumulated_heading_deg_ = 0.0f;
            start_heading_deg_ = 0.0f;
            imu_.resetHeading(0.0f);
            stall_count_ = 0;
            return;
        }
        return;
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
        chained_coast_timer_ = 0;
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
    calibrating_motors_ = false;
    profile_linear_.stop();
    profile_angular_.stop();
    command_active_ = false;
    command_finished_ = true;
    chained_coast_timer_ = 255;
    motors_.brake();
}

void MotionController::setWallCenteringEnabled(bool enabled) {
    wall_centering_enabled_ = enabled;
}

bool MotionController::calibrateMotors() {
    Serial.println("\n=======================================================");
    Serial.println("  AUTOMATED MOTOR SPEED & TRIM BALANCING BENCHMARK");
    Serial.println("=======================================================");
    Serial.println("[CALIB] Place robot on a stand/box with wheels freewheeling.");
    Serial.println("[CALIB] Starting multi-point optical tachometer routine in 1.5s...");
    delay(1500);

    calibrating_motors_ = true;
    motors_.coast();
    delay(200);

    // Test points: 25%, 50%, 75% raw duty
    const float test_duties[] = {0.25f, 0.50f, 0.75f};
    const int num_points = 3;
    float ratios[3] = {1.0f, 1.0f, 1.0f};

    for (int i = 0; i < num_points; i++) {
        float duty = test_duties[i];
        Serial.printf("\n[CALIB] Step %d/%d: Testing raw duty %2.0f%%...\n", i + 1, num_points, duty * 100.0f);

        // Spin up with raw effort (bypassing current trim)
        motors_.setRawEffort(duty, duty);
        delay(400); // Wait for motors to reach steady-state velocity

        // Measure encoder tick deltas over 1000ms
        uint32_t t_start = millis();
        EncoderState enc_start = encoders_.getState();
        int32_t start_ticks_l = enc_start.left_ticks_total;
        int32_t start_ticks_r = enc_start.right_ticks_total;

        delay(1000);

        uint32_t t_end = millis();
        EncoderState enc_end = encoders_.getState();
        int32_t end_ticks_l = enc_end.left_ticks_total;
        int32_t end_ticks_r = enc_end.right_ticks_total;

        float dt_s = (float)(t_end - t_start) / 1000.0f;
        int32_t delta_l = abs(end_ticks_l - start_ticks_l);
        int32_t delta_r = abs(end_ticks_r - start_ticks_r);

        // Safety check: verify wheels are actually turning
        if (delta_l < 30 || delta_r < 30) {
            Serial.printf("[CALIB ERROR] Wheel rotation not detected! (L_ticks=%d, R_ticks=%d)\n", delta_l, delta_r);
            Serial.println("[CALIB ERROR] Ensure 12V boost is active and robot wheels are not physically jammed.");
            motors_.coast();
            calibrating_motors_ = false;
            resetTracking();
            return false;
        }

        // RPM = (Delta_Ticks / ENCODER_TOTAL_CPR) / dt_s * 60
        float rpm_l = ((float)delta_l / ENCODER_TOTAL_CPR) / dt_s * 60.0f;
        float rpm_r = ((float)delta_r / ENCODER_TOTAL_CPR) / dt_s * 60.0f;

        ratios[i] = rpm_l / rpm_r;
        Serial.printf("[CALIB] -> Left: %6.1f RPM | Right: %6.1f RPM | Ratio (L/R): %.4f\n", rpm_l, rpm_r, ratios[i]);
    }

    motors_.coast();
    delay(300);

    // Compute average ratio across test points
    float avg_ratio = (ratios[0] + ratios[1] + ratios[2]) / 3.0f;

    // Calculate trims: scale down the faster motor so both match the slower one
    float trim_left = 1.0f;
    float trim_right = 1.0f;

    if (avg_ratio > 1.002f) {
        // Left is faster -> trim Left down
        trim_left = 1.0f / avg_ratio;
        trim_right = 1.0f;
    } else if (avg_ratio < 0.998f) {
        // Right is faster -> trim Right down
        trim_left = 1.0f;
        trim_right = avg_ratio;
    }

    // Clamp trim values between 0.60 and 1.00 for safety
    trim_left = constrain(trim_left, 0.60f, 1.0f);
    trim_right = constrain(trim_right, 0.60f, 1.0f);

    // Apply and persist to Flash NVS
    motors_.setTrim(trim_left, trim_right);
    motors_.saveToNVS();

    // Verification test: run at 50% duty with trims applied
    Serial.println("\n[CALIB] Running post-trim verification at 50% duty...");
    motors_.setEffort(0.50f, 0.50f);
    delay(400);

    uint32_t v_start = millis();
    EncoderState v_enc_start = encoders_.getState();
    int32_t v_ticks_l_start = v_enc_start.left_ticks_total;
    int32_t v_ticks_r_start = v_enc_start.right_ticks_total;

    delay(1000);

    uint32_t v_end = millis();
    EncoderState v_enc_end = encoders_.getState();
    float v_dt = (float)(v_end - v_start) / 1000.0f;
    int32_t v_delta_l = abs(v_enc_end.left_ticks_total - v_ticks_l_start);
    int32_t v_delta_r = abs(v_enc_end.right_ticks_total - v_ticks_r_start);

    motors_.coast();

    float v_rpm_l = ((float)v_delta_l / ENCODER_TOTAL_CPR) / v_dt * 60.0f;
    float v_rpm_r = ((float)v_delta_r / ENCODER_TOTAL_CPR) / v_dt * 60.0f;
    float diff_rpm = v_rpm_l - v_rpm_r;
    float diff_pct = (fabsf(diff_rpm) / ((v_rpm_l + v_rpm_r) * 0.5f)) * 100.0f;

    Serial.println("=======================================================");
    Serial.println("         CALIBRATION RESULTS & TRIM SAVED              ");
    Serial.println("=======================================================");
    Serial.printf(" Raw Speed Ratio (L/R) : %.4f\n", avg_ratio);
    Serial.printf(" Trim Multipliers      : Left = %.4f | Right = %.4f\n", trim_left, trim_right);
    Serial.printf(" Verified 50%% RPM      : Left = %6.1f | Right = %6.1f\n", v_rpm_l, v_rpm_r);
    Serial.printf(" RPM Discrepancy       : %+.1f RPM (%.2f%% difference)\n", diff_rpm, diff_pct);
    Serial.println("=======================================================\n");

    calibrating_motors_ = false;
    resetTracking();
    return true;
}

void MotionController::runTachometerBenchmark(float duty, uint16_t duration_ms) {
    calibrating_motors_ = true;
    duty = constrain(duty, 0.1f, 1.0f);

    float trim_l = 1.0f, trim_r = 1.0f;
    motors_.getTrim(trim_l, trim_r);

    Serial.println("\n-------------------------------------------------------");
    Serial.printf(" [TACH] RUNNING BENCHMARK @ %2.0f%% DUTY (Duration: %d ms)\n", duty * 100.0f, duration_ms);
    Serial.printf(" [TACH] Active Trims: Left=%.4f, Right=%.4f\n", trim_l, trim_r);
    Serial.println(" [TACH] Point your external optical RPM tachometer gun now!");
    Serial.println("-------------------------------------------------------");

    motors_.setEffort(duty, duty);
    delay(300); // Settle time

    uint32_t start_time = millis();
    uint32_t last_sample_time = millis();
    int32_t last_ticks_l = encoders_.getState().left_ticks_total;
    int32_t last_ticks_r = encoders_.getState().right_ticks_total;

    while (millis() - start_time < duration_ms) {
        delay(500);

        uint32_t now = millis();
        float dt_s = (float)(now - last_sample_time) / 1000.0f;
        EncoderState enc = encoders_.getState();
        int32_t delta_l = abs(enc.left_ticks_total - last_ticks_l);
        int32_t delta_r = abs(enc.right_ticks_total - last_ticks_r);

        last_sample_time = now;
        last_ticks_l = enc.left_ticks_total;
        last_ticks_r = enc.right_ticks_total;

        float rpm_l = ((float)delta_l / ENCODER_TOTAL_CPR) / dt_s * 60.0f;
        float rpm_r = ((float)delta_r / ENCODER_TOTAL_CPR) / dt_s * 60.0f;
        float diff = rpm_l - rpm_r;

        Serial.printf("[TACH +%4d ms] Left: %6.1f RPM | Right: %6.1f RPM | Diff: %+5.1f RPM\n",
                      (int)(now - start_time), rpm_l, rpm_r, diff);
    }

    motors_.coast();
    Serial.println("[TACH] Benchmark complete. Motors stopped.\n");

    calibrating_motors_ = false;
    resetTracking();
}

void MotionController::saveToNVS() {
    Preferences prefs;
    prefs.begin("motion_cal", false);
    prefs.putFloat("k_sync", k_wheel_sync_);
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    pid_linear_vel_.getGains(kp, ki, kd);
    prefs.putFloat("v_kp", kp);
    prefs.putFloat("v_ki", ki);
    prefs.putFloat("v_kd", kd);
    prefs.putBool("valid", true);
    prefs.end();
    Serial.printf("[MOTION] Calibration saved to NVS: Sync=%.6f, VelPID=[%.5f, %.5f, %.6f]\n",
                  k_wheel_sync_, kp, ki, kd);
}

bool MotionController::loadFromNVS() {
    Preferences prefs;
    prefs.begin("motion_cal", true);
    if (!prefs.getBool("valid", false)) {
        prefs.end();
        k_wheel_sync_ = 0.0004f;
        return false;
    }
    k_wheel_sync_ = prefs.getFloat("k_sync", 0.0004f);
    float kp = prefs.getFloat("v_kp", 0.0025f);
    float ki = prefs.getFloat("v_ki", 0.0005f);
    float kd = prefs.getFloat("v_kd", 0.00005f);
    pid_linear_vel_.setGains(kp, ki, kd);
    prefs.end();
    Serial.printf("[MOTION] Calibration loaded from NVS: Sync=%.6f, VelPID=[%.5f, %.5f, %.6f]\n",
                  k_wheel_sync_, kp, ki, kd);
    return true;
}

