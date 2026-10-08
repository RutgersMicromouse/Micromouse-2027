#include "control/motion_controller/motion_controller.h"

// ==============================================================================
// MOTION CONTROLLER: THE 500 Hz LOOP
// ==============================================================================

// Called once per control tick. Works out where the robot should be by now, compares that
// with the encoders, IMU and IR sensors, and sets the motor efforts.

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
            float forward_effort = pid_linear_vel_.update(vel_err, dt_seconds) +
                                   wheelFeedforward(active_cmd_.exit_speed_mm_s, 0.0f);
            motors_.setEffort(forward_effort, forward_effort);
            return;
        }
        // A move that was meant to be followed at speed, but nothing followed: the robot is no
        // longer where that move aimed to end. Likewise if it has simply been standing a while.
        if (active_cmd_.exit_speed_mm_s > 10.0f || idle_ticks_ > CHAIN_TIMEOUT_TICKS) {
            if (chain_valid_ && active_cmd_.exit_speed_mm_s > 10.0f) late_handovers_ = late_handovers_ + 1;
            chain_valid_ = false;
        } else {
            idle_ticks_++;
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

    // Note what the 45° sensors see of the cell ahead (used by the search run to turn early)
    samplePreview(current_distance - start_distance_mm_);

    float forward_effort = 0.0f;
    float rotational_effort = 0.0f;

    // 2. Linear Profile & Controller
    // While the speed profile is running the robot follows it. Once it has run out, the robot
    // holds the point the move was aiming for (for a turn on the spot: the point it started on).
    float dist_err = 0.0f;
    {
        const bool profile_running = !profile_linear_.isFinished();
        if (profile_running) profile_linear_.update(dt_seconds);

        float dist_traveled = current_distance - start_distance_mm_;

        // Search moves roll into a cell at speed in case the way ahead is open. If the front
        // sensors find a wall instead, re-plan the rest of this move to come to rest exactly on
        // its target (the cell centre), rather than arriving at speed with a wall ahead.
        //
        // How far is "the rest"? The wheel encoders say one thing, but the front sensors measure
        // the wall itself: the moment it comes into view the robot is a known distance short of
        // the cell centre (IRReadings::front_offset_mm, only once the front level has been
        // measured against a real wall). That is used when there is one, so the robot stops in
        // the middle of the cell instead of running on and having to step back. A move that was
        // going to stop at the centre anyway is corrected the same way.
        if (profile_running && active_cmd_.stop_at_front_wall && ir_.hasFrontWall()) {
            const float by_encoders = target_relative_dist_mm_ - profile_linear_.getTargetDistance();
            const float by_sensors  = ir_.getReadings().front_offset_mm;
            const bool  measured    = ENABLE_FRONT_WALL_STOP && by_sensors != 0.0f;
            float remaining = by_encoders;
            if (measured) { // Believe the wall, but not blindly: stay within FRONT_STOP_TRUST_MM of the encoders
                remaining = constrain(by_sensors, by_encoders - FRONT_STOP_TRUST_MM, by_encoders + FRONT_STOP_TRUST_MM);
            }
            if (remaining > 1.0f && (measured || active_cmd_.exit_speed_mm_s > 10.0f)) {
                float v_now = profile_linear_.getTargetVelocity();
                // Brake as hard as it takes to stop in that distance, if the usual rate is not enough
                float decel = fmaxf(active_cmd_.acceleration, (v_now * v_now) / (2.0f * remaining));
                start_distance_mm_ += profile_linear_.getTargetDistance();
                dist_traveled = current_distance - start_distance_mm_;
                target_relative_dist_mm_ = remaining;
                profile_linear_.start(remaining, v_now, decel, v_now, 0.0f);
                profile_linear_.update(dt_seconds);
            }
            active_cmd_.exit_speed_mm_s = 0.0f;
            active_cmd_.stop_at_front_wall = false; // Done once per move
        }

        const bool holding = profile_linear_.isFinished() && !profile_running;
        float target_dist = holding ? target_relative_dist_mm_ : profile_linear_.getTargetDistance();
        float target_vel  = holding ? 0.0f : profile_linear_.getTargetVelocity();

        // Distance error
        dist_err = target_dist - dist_traveled;
        float vel_setpoint = target_vel + pid_linear_dist_.update(dist_err, dt_seconds);

        // Velocity error
        float vel_err = vel_setpoint - enc.linear_speed_mm_s;
        forward_effort = pid_linear_vel_.update(vel_err, dt_seconds);

        // Feedforward: most of the effort comes straight from the planned speed and acceleration.
        // While holding a position, it supplies the push needed to creep the last millimetres.
        if (holding) {
            forward_effort += wheelFeedforward(vel_setpoint, 0.0f);
        } else {
            float target_accel = (target_vel - prev_target_speed_mm_s_) / dt_seconds;
            prev_target_speed_mm_s_ = target_vel;
            forward_effort += wheelFeedforward(target_vel, target_accel);
        }
    }

    // 3. Angular Profile & Controller (Continuous unwrapped heading)
    float heading_traveled = accumulated_heading_deg_ - start_heading_deg_;
    float target_heading_setpoint = 0.0f;

    float rotational_feedforward = 0.0f;

    if (curve_active_) {
        // Smooth turn: heading eases in and out along the path (zero yaw rate at both ends, so it
        // joins the straights before and after without a jerk): heading = angle * (3u² - 2u³)
        float u = constrain(profile_linear_.getTargetDistance() / curve_length_mm_, 0.0f, 1.0f);
        target_heading_setpoint = target_relative_angle_deg_ * u * u * (3.0f - 2.0f * u);

        // Feedforward the wheel speed difference the turn needs, so the PID only trims the error
        float yaw_rate_deg_s = target_relative_angle_deg_ * 6.0f * u * (1.0f - u) / curve_length_mm_ *
                               profile_linear_.getTargetVelocity();
        float wheel_delta_mm_s = yaw_rate_deg_s * (PI / 180.0f) * (WHEEL_BASE_MM * 0.5f);
        rotational_feedforward = tune_[TUNE_FF_KV] * wheel_delta_mm_s * tune_[TUNE_TURN_FF];
    } else if (!profile_angular_.isFinished()) {
        profile_angular_.update(dt_seconds);
        target_heading_setpoint = profile_angular_.getTargetDistance();

        // Turn on the spot: each wheel runs at (yaw rate x half the wheelbase), in opposite directions
        float wheel_mm_s = profile_angular_.getTargetVelocity() * (PI / 180.0f) * (WHEEL_BASE_MM * 0.5f);
        rotational_feedforward = wheelFeedforward(wheel_mm_s, 0.0f) * tune_[TUNE_TURN_FF];
    } else {
        target_heading_setpoint = target_relative_angle_deg_;

        // Diagonals: there are no side walls to follow, only posts passing close by on both sides
        if (active_cmd_.action == ACTION_MOVE_DIAGONAL_HALF) {
            IRReadings ir_now = ir_.getReadings();

            // 1. Centring. Remember the strongest reading of each row of posts as they go by,
            // letting it fade with distance travelled so an old post cannot steer the robot.
            float fade = 1.0f - (fabsf(enc.linear_speed_mm_s) * dt_seconds) / DIAG_PEAK_MEMORY_MM;
            if (fade < 0.0f) fade = 0.0f;
            float now_left  = (float)ir_now.left_45  / (float)ir_.getNominalL45();
            float now_right = (float)ir_now.right_45 / (float)ir_.getNominalR45();
            diag_peak_left_  = fmaxf(now_left,  diag_peak_left_  * fade);
            diag_peak_right_ = fmaxf(now_right, diag_peak_right_ * fade);

            float centre_err = 0.0f; // +ve = closer to the left row of posts
            if (diag_peak_left_ > DIAG_PEAK_MIN && diag_peak_right_ > DIAG_PEAK_MIN) {
                // Reading falls with distance squared, so distance goes as 1/sqrt(reading)
                float dist_left  = 1.0f / sqrtf(diag_peak_left_);
                float dist_right = 1.0f / sqrtf(diag_peak_right_);
                centre_err = (dist_right - dist_left) / (dist_right + dist_left);
            }

            // 2. Guard. If one post is closer than it should ever be, steer away from it hard.
            float guard_err = ir_.getDiagonalGuardError(); // +ve = too close on the left

            float trim_deg = -(centre_err * DIAG_CENTER_GAIN_DEG + guard_err * DIAG_GUARD_MAX_TRIM_DEG);
            target_heading_setpoint += constrain(trim_deg, -DIAG_GUARD_MAX_TRIM_DEG, DIAG_GUARD_MAX_TRIM_DEG);
        }

        // Wall Centering steering injection during straight runs
        if (wall_centering_enabled_ && active_cmd_.enable_wall_centering &&
            (active_cmd_.action == ACTION_MOVE_FORWARD_CELLS || active_cmd_.action == ACTION_MOVE_DISTANCE || active_cmd_.action == ACTION_MOVE_HALF_CELL)) {

            float centering_err = ir_.getCenteringError();
            if (fabsf(centering_err) > 0.001f) {
                // Negative sign: if mouse is too close to left wall, steer right (negative angle trim)
                float wall_trim_angle = -pid_wall_centering_.update(centering_err, dt_seconds);

                // Gyro yaw rate damping: prevents overshooting and snaking during aggressive centering
                float gyro_z = imu_.getGyroZ(); // deg/s (CCW positive)
                wall_trim_angle -= (gyro_z * tune_[TUNE_W_GYRO]);

                target_heading_setpoint += wall_trim_angle;
            } else {
                pid_wall_centering_.reset();
            }
        }
    }

    // Shortest-path angular error normalization in (-180, +180]
    float heading_err = shortestAngularDifference(target_heading_setpoint, heading_traveled);
    rotational_effort = pid_angular_heading_.update(heading_err, dt_seconds) + rotational_feedforward;

    // Settling after a turn on the spot: the planned turn has run out, so the feedforward is gone
    // and only the heading loop is left to steer out what the turn left over. With a gentle
    // heading gain that is less effort than it takes to make the wheels move at all, and the
    // robot just sits there a few degrees off until the settle wait gives up. So add the effort
    // that overcomes friction, plus a little, toward the target until it is inside the tolerance.
    // Only while it is not already swinging toward the target, though: pushing a robot that is
    // on its way makes it run past and rock back and forth before it settles.
    const bool turn_on_spot = (active_cmd_.action == ACTION_TURN_LEFT_90 || active_cmd_.action == ACTION_TURN_RIGHT_90 ||
                               active_cmd_.action == ACTION_TURN_LEFT_45 || active_cmd_.action == ACTION_TURN_RIGHT_45 ||
                               active_cmd_.action == ACTION_TURN_AROUND_180);
    const float rate_toward_target = copysignf(1.0f, heading_err) * imu_.getGyroZ(); // deg/s
    if (turn_on_spot && profile_angular_.isFinished() && fabsf(heading_err) > SETTLE_HEADING_DEG &&
        rate_toward_target < TURN_SETTLE_PUSH_MAX_RATE_DEG_S) {
        rotational_effort += copysignf(tune_[TUNE_FF_KS] + TURN_SETTLE_PUSH, heading_err);
    }

    // Differential Wheel Speed Lock: actively prevents wheel speed divergence during straight lines
    if (active_cmd_.action == ACTION_MOVE_FORWARD_CELLS || active_cmd_.action == ACTION_MOVE_DISTANCE ||
        active_cmd_.action == ACTION_MOVE_HALF_CELL || active_cmd_.action == ACTION_MOVE_DIAGONAL_HALF) {
        float speed_diff = enc.right_speed_mm_s - enc.left_speed_mm_s;
        rotational_effort -= (speed_diff * tune_[TUNE_K_SYNC]);
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
                snapHeadingToGrid();
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

        // Difference between the two front sensors, each measured against its own calibrated
        // reading: positive means the left one is closer (tilted CW -> pivot CCW/left).
        // 0 until the robot has been calibrated facing a wall, so it then does not pivot at all.
        float norm_diff = ir_.getFrontSkew();

        // Within 4% symmetry = perfectly perpendicular to front wall
        if (fabsf(norm_diff) < 0.04f) {
            wall_align_timer_++;
            motors_.brake();

            if (wall_align_timer_ >= 25) { // 25 ticks @ 500 Hz = 50 ms of stable symmetry
                command_finished_ = true;
                command_active_ = false;
                motors_.brake();

                snapHeadingToGrid();
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
            stall_count_ = 0; // Gave up squaring: keep the heading as it was rather than trust a bad alignment
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
            safety_stop_ = true;
            stall_count_ = 0;
            return;
        }
    } else {
        stall_count_ = 0;
    }

    // 5b. Encoder Fault Detection: on a straight, one wheel reading zero while the other is clearly
    // moving means a dead encoder channel (or a jammed wheel). Distance tracking is no longer
    // trustworthy, so stop instead of driving blind into a wall.
    bool is_straight = (active_cmd_.action == ACTION_MOVE_FORWARD_CELLS || active_cmd_.action == ACTION_MOVE_DISTANCE ||
                        active_cmd_.action == ACTION_MOVE_HALF_CELL || active_cmd_.action == ACTION_MOVE_DIAGONAL_HALF);
    float slow_wheel = fminf(fabsf(enc.left_speed_mm_s), fabsf(enc.right_speed_mm_s));
    float fast_wheel = fmaxf(fabsf(enc.left_speed_mm_s), fabsf(enc.right_speed_mm_s));
    if (is_straight && slow_wheel < 5.0f && fast_wheel > 80.0f) {
        encoder_fault_ticks_++;
        if (encoder_fault_ticks_ > ENCODER_FAULT_TICKS) {
            Serial.printf("\n[SAFETY ALERT] ENCODER FAULT! Left=%.0f mm/s, Right=%.0f mm/s. Emergency stop engaged.\n",
                          enc.left_speed_mm_s, enc.right_speed_mm_s);
            encoder_faults_++;
            emergencyStop();
            safety_stop_ = true;
            encoder_fault_ticks_ = 0;
            return;
        }
    } else {
        encoder_fault_ticks_ = 0;
    }

    // 5c. Lost-Heading Protection: the robot is pointing far from where this move wants it, which
    // only happens after a crash or when someone has picked it up. With no buttons on the robot,
    // this is also the hands-on way to stop a run: lift it and turn it sideways.
    if (fabsf(heading_err) > HEADING_FAULT_DEG) {
        heading_fault_ticks_++;
        if (heading_fault_ticks_ > HEADING_FAULT_TICKS) {
            Serial.printf("\n[SAFETY ALERT] HEADING LOST (%.0f deg off course). Emergency stop engaged.\n", heading_err);
            emergencyStop();
            safety_stop_ = true;
            heading_fault_ticks_ = 0;
            return;
        }
    } else {
        heading_fault_ticks_ = 0;
    }

    // 5d. Run Log: planned vs. actual, for tuning (read back with the console command "log")
    if (++log_tick_ >= RUN_LOG_DIVIDER) {
        log_tick_ = 0;
        IRReadings ir_log = ir_.getReadings();
        RunLogSample sample;
        sample.target_speed_mm_s = (int16_t)profile_linear_.getTargetVelocity();
        sample.speed_mm_s        = (int16_t)enc.linear_speed_mm_s;
        sample.heading_x10       = (int16_t)(normalizeAngle180(accumulated_heading_deg_) * 10.0f);
        sample.heading_err_x10   = (int16_t)(heading_err * 10.0f);
        sample.forward_pct       = (int8_t)constrain(forward_effort * 100.0f, -127.0f, 127.0f);
        sample.turn_pct          = (int8_t)constrain(rotational_effort * 100.0f, -127.0f, 127.0f);
        sample.action            = (uint8_t)active_cmd_.action;
        sample.reserved          = 0;
        sample.ir[0] = ir_log.left_90;     sample.ir[1] = ir_log.left_45;
        sample.ir[2] = ir_log.front_left;  sample.ir[3] = ir_log.front_right;
        sample.ir[4] = ir_log.right_45;    sample.ir[5] = ir_log.right_90;
        run_log_.record(sample);
    }

    // 6. Actuator Mixer
    float left_duty  = forward_effort - rotational_effort;
    float right_duty = forward_effort + rotational_effort;

    motors_.setEffort(left_duty, right_duty);

    // 7. Completion check
    if (profile_linear_.isFinished() && profile_angular_.isFinished()) {
        const bool stopping = (active_cmd_.exit_speed_mm_s <= 10.0f);

        // A move that ends in a stop is only finished once the robot has actually arrived: the
        // planned motion running out just means it should be there by now. Otherwise each stop
        // would leave it a few millimetres short, and those add up across a maze.
        if (stopping) {
            const bool arrived = fabsf(dist_err) < SETTLE_DISTANCE_MM &&
                                 fabsf(heading_err) < SETTLE_HEADING_DEG &&
                                 fabsf(enc.linear_speed_mm_s) < SETTLE_SPEED_MM_S &&
                                 fabsf(imu_.getGyroZ()) < SETTLE_YAW_RATE_DEG_S;
            // ...and has STAYED there for a moment. One good reading is not enough: if the
            // heading reading runs behind the real turn, or the robot is still swinging, the
            // error shows up a few ticks later and must be steered out before the motors brake.
            settle_good_ticks_ = arrived ? settle_good_ticks_ + 1 : 0;
            const uint16_t patience = turn_on_spot ? SETTLE_TIMEOUT_TURN_TICKS : SETTLE_TIMEOUT_TICKS;
            if (settle_good_ticks_ < SETTLE_DWELL_TICKS && settle_ticks_ < patience) {
                settle_ticks_++;
                return; // Keep holding the target
            }
        }

        command_finished_ = true;
        command_active_ = false;
        curve_active_ = false;
        chained_coast_timer_ = 0;
        idle_ticks_ = 0;

        // The next move starts from where this one aimed to end, not from wherever the robot
        // happened to be when it was declared finished
        chain_valid_ = true;
        chain_end_distance_mm_ = start_distance_mm_ + target_relative_dist_mm_;

        if (stopping) {
            motors_.brake();
        }
    }
}
