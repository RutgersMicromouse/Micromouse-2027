#include "control.h"

// ==============================================================================
// PID CONTROLLER
// ==============================================================================

PIDController::PIDController()
    : kp_(0.0f), ki_(0.0f), kd_(0.0f),
      max_output_(1.0f), max_integral_(0.5f),
      integral_(0.0f), prev_error_(0.0f),
      prev_derivative_(0.0f), first_run_(true) {}

PIDController::PIDController(float kp, float ki, float kd, float max_out, float max_integral)
    : kp_(kp), ki_(ki), kd_(kd),
      max_output_(max_out),
      max_integral_(max_integral > 0.0f ? max_integral : max_out * 0.5f),
      integral_(0.0f), prev_error_(0.0f),
      prev_derivative_(0.0f), first_run_(true) {}

void PIDController::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(float max_out, float max_integral) {
    max_output_ = max_out;
    max_integral_ = (max_integral > 0.0f) ? max_integral : (max_out * 0.5f);
}

void PIDController::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    prev_derivative_ = 0.0f;
    first_run_ = true;
}

float PIDController::update(float error, float dt_seconds) {
    if (dt_seconds <= 0.00001f) {
        dt_seconds = 0.001f;
    }

    // 1. Proportional term
    float p_term = kp_ * error;

    // 2. Integral term with anti-windup clamping
    integral_ += error * dt_seconds;
    if (integral_ > max_integral_)  integral_ = max_integral_;
    if (integral_ < -max_integral_) integral_ = -max_integral_;
    float i_term = ki_ * integral_;

    // 3. Derivative term with low-pass filtering
    float raw_derivative = 0.0f;
    if (!first_run_) {
        raw_derivative = (error - prev_error_) / dt_seconds;
    } else {
        first_run_ = false;
    }
    prev_error_ = error;

    // Filter derivative to suppress high frequency sensor noise (cutoff ~50Hz)
    const float alpha = 0.3f;
    float filtered_derivative = (alpha * raw_derivative) + ((1.0f - alpha) * prev_derivative_);
    prev_derivative_ = filtered_derivative;
    float d_term = kd_ * filtered_derivative;

    // 4. Sum terms and saturate output
    float output = p_term + i_term + d_term;
    if (output > max_output_)  output = max_output_;
    if (output < -max_output_) output = -max_output_;

    return output;
}

// ==============================================================================
// TRAPEZOIDAL PROFILE
// ==============================================================================

TrapezoidalProfile::TrapezoidalProfile()
    : target_total_dist_(0.0f), max_speed_(0.0f), accel_(0.0f),
      v_start_(0.0f), v_peak_(0.0f), v_end_(0.0f),
      current_dist_(0.0f), current_vel_(0.0f),
      d_accel_(0.0f), d_cruise_(0.0f), d_decel_(0.0f),
      t_accel_(0.0f), t_cruise_(0.0f), t_decel_(0.0f), t_total_(0.0f),
      elapsed_time_(0.0f), finished_(true) {}

void TrapezoidalProfile::start(float target_distance, float max_speed, float acceleration,
                               float start_speed, float end_speed) {
    target_total_dist_ = target_distance;
    max_speed_ = fabsf(max_speed);
    accel_ = fabsf(acceleration);
    v_start_ = fabsf(start_speed);
    v_end_ = fabsf(end_speed);

    elapsed_time_ = 0.0f;
    current_dist_ = 0.0f;
    current_vel_ = v_start_;
    finished_ = false;

    if (accel_ < 1.0f) accel_ = 1.0f;
    if (max_speed_ < 1.0f) max_speed_ = 1.0f;
    if (v_start_ > max_speed_) max_speed_ = v_start_;
    if (v_end_ > max_speed_) v_end_ = max_speed_;

    float dist_abs = fabsf(target_distance);
    if (dist_abs < 0.1f) {
        current_vel_ = v_end_;
        finished_ = true;
        return;
    }

    // Distance required to transition v_start -> max_speed and max_speed -> v_end
    float da = fabsf(max_speed_ * max_speed_ - v_start_ * v_start_) / (2.0f * accel_);
    float dd = fabsf(max_speed_ * max_speed_ - v_end_ * v_end_) / (2.0f * accel_);

    if ((da + dd) > dist_abs) {
        // Triangular profile (cannot reach full max_speed)
        float vp_sq = (2.0f * accel_ * dist_abs + v_start_ * v_start_ + v_end_ * v_end_) / 2.0f;
        v_peak_ = sqrtf(fmaxf(0.0f, vp_sq));
        if (v_peak_ > max_speed_) v_peak_ = max_speed_;

        d_accel_ = fabsf(v_peak_ * v_peak_ - v_start_ * v_start_) / (2.0f * accel_);
        d_decel_ = dist_abs - d_accel_;
        d_cruise_ = 0.0f;

        t_accel_ = fabsf(v_peak_ - v_start_) / accel_;
        t_cruise_ = 0.0f;
        t_decel_ = fabsf(v_peak_ - v_end_) / accel_;
        t_total_ = t_accel_ + t_decel_;
    } else {
        // Full trapezoidal profile
        v_peak_ = max_speed_;
        d_accel_ = da;
        d_decel_ = dd;
        d_cruise_ = dist_abs - (da + dd);

        t_accel_ = fabsf(v_peak_ - v_start_) / accel_;
        t_cruise_ = d_cruise_ / max_speed_;
        t_decel_ = fabsf(v_peak_ - v_end_) / accel_;
        t_total_ = t_accel_ + t_cruise_ + t_decel_;
    }
}

void TrapezoidalProfile::update(float dt_seconds) {
    if (finished_) return;

    elapsed_time_ += dt_seconds;
    float sign = (target_total_dist_ >= 0.0f) ? 1.0f : -1.0f;
    float t = elapsed_time_;

    if (t >= t_total_) {
        current_dist_ = target_total_dist_;
        current_vel_ = v_end_ * sign;
        finished_ = true;
        return;
    }

    if (t <= t_accel_) {
        // Phase 1: Acceleration / Initial ramp
        float s_acc = (v_peak_ >= v_start_) ? 1.0f : -1.0f;
        current_vel_ = v_start_ + (s_acc * accel_ * t);
        current_dist_ = (v_start_ * t) + (0.5f * s_acc * accel_ * t * t);
    } else if (t <= (t_accel_ + t_cruise_)) {
        // Phase 2: Cruising at v_peak
        float dt_c = t - t_accel_;
        current_vel_ = v_peak_;
        current_dist_ = d_accel_ + (v_peak_ * dt_c);
    } else {
        // Phase 3: Deceleration to v_end
        float dt_d = t - (t_accel_ + t_cruise_);
        float s_dec = (v_end_ >= v_peak_) ? 1.0f : -1.0f;
        current_vel_ = v_peak_ + (s_dec * accel_ * dt_d);
        current_dist_ = d_accel_ + d_cruise_ + (v_peak_ * dt_d) + (0.5f * s_dec * accel_ * dt_d * dt_d);
    }

    current_dist_ *= sign;
    current_vel_ *= sign;
}

float TrapezoidalProfile::getTargetDistance() const {
    return current_dist_;
}

float TrapezoidalProfile::getTargetVelocity() const {
    return current_vel_;
}

bool TrapezoidalProfile::isFinished() const {
    return finished_;
}

void TrapezoidalProfile::stop() {
    finished_ = true;
    current_vel_ = 0.0f;
}

// ==============================================================================
// MOTION CONTROLLER
// ==============================================================================

MotionController::MotionController(Encoders& encoders, Motors& motors, IRSensors& ir, IMU& imu)
    : encoders_(encoders), motors_(motors), ir_(ir), imu_(imu),
      command_active_(false), command_finished_(true),
      start_distance_mm_(0.0f), start_heading_deg_(0.0f),
      target_relative_dist_mm_(0.0f), target_relative_angle_deg_(0.0f),
      accumulated_heading_deg_(0.0f), prev_raw_heading_deg_(0.0f),
      stall_count_(0), encoder_fault_ticks_(0), encoder_faults_(0), heading_fault_ticks_(0),
      wall_align_timer_(0), chained_coast_timer_(0),
      curve_active_(false), curve_length_mm_(0.0f),
      chain_valid_(false), chain_end_distance_mm_(0.0f),
      preview_min_l_(0), preview_max_l_(0), preview_min_r_(0), preview_max_r_(0), preview_samples_(0),
      front_min_(0), front_max_(0), front_samples_(0), front_uses_left_sensor_(false),
      diag_peak_left_(0.0f), diag_peak_right_(0.0f),
      prev_target_speed_mm_s_(0.0f), log_tick_(0),
      wall_centering_enabled_(true), calibrating_motors_(false), safety_stop_(false),
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
    encoder_fault_ticks_ = 0;
    heading_fault_ticks_ = 0;
    wall_align_timer_ = 0;
    chained_coast_timer_ = 0;
    motors_.coast();
}

void MotionController::resetHeading() {
    imu_.resetHeading(0.0f);
    pid_angular_heading_.reset();
    pid_wall_centering_.reset();
    start_heading_deg_ = 0.0f;
    accumulated_heading_deg_ = 0.0f;
    prev_raw_heading_deg_ = imu_.getHeadingDeg();
}

void MotionController::executeCommand(const MotionCommand& cmd) {
    active_cmd_ = cmd;
    command_finished_ = false;
    command_active_ = true;
    stall_count_ = 0;
    encoder_fault_ticks_ = 0;
    heading_fault_ticks_ = 0;
    wall_align_timer_ = 0;
    chained_coast_timer_ = 0;

    // Where this move starts. When it follows straight on from a move that ended at speed, start
    // from exactly where that move was supposed to end, so small tracking errors are corrected by
    // this move instead of piling up along a chain of moves.
    EncoderState enc = encoders_.getState();
    if (chain_valid_) {
        start_distance_mm_ = chain_end_distance_mm_;
    } else {
        start_distance_mm_ = (enc.left_dist_mm + enc.right_dist_mm) * 0.5f;
    }
    chain_valid_ = false;
    curve_active_ = false;
    prev_target_speed_mm_s_ = cmd.entry_speed_mm_s;
    preview_samples_ = 0;
    front_samples_ = 0;
    diag_peak_left_ = 0.0f;
    diag_peak_right_ = 0.0f;

    // Grid Axis Snapping: every move starts from an exact maze heading (a multiple of 45°), so a
    // few degrees of error left over from the previous turn is steered out instead of carried on.
    // Straights hold a cardinal heading, diagonals and smooth curves any multiple of 45°.
    bool is_cardinal_move = (cmd.action == ACTION_MOVE_FORWARD_CELLS || cmd.action == ACTION_MOVE_DISTANCE ||
                             cmd.action == ACTION_MOVE_HALF_CELL);
    bool is_grid_move = (cmd.action == ACTION_MOVE_DIAGONAL_HALF ||
                         cmd.action == ACTION_CURVE_LEFT_90 || cmd.action == ACTION_CURVE_RIGHT_90 ||
                         cmd.action == ACTION_CURVE_LEFT_45 || cmd.action == ACTION_CURVE_RIGHT_45);
    start_heading_deg_ = accumulated_heading_deg_;
    if (is_cardinal_move) {
        float nearest_cardinal = roundf(accumulated_heading_deg_ / 90.0f) * 90.0f;
        if (fabsf(accumulated_heading_deg_ - nearest_cardinal) < 25.0f) start_heading_deg_ = nearest_cardinal;
    } else if (is_grid_move) {
        float nearest_grid = roundf(accumulated_heading_deg_ / 45.0f) * 45.0f;
        if (fabsf(accumulated_heading_deg_ - nearest_grid) < 18.0f) start_heading_deg_ = nearest_grid;
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

        case ACTION_CURVE_LEFT_90:  startCurve(cmd,  90.0f, CURVE_90_LENGTH_MM); break;
        case ACTION_CURVE_RIGHT_90: startCurve(cmd, -90.0f, CURVE_90_LENGTH_MM); break;
        case ACTION_CURVE_LEFT_45:  startCurve(cmd,  45.0f, CURVE_45_LENGTH_MM); break;
        case ACTION_CURVE_RIGHT_45: startCurve(cmd, -45.0f, CURVE_45_LENGTH_MM); break;

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

// Effort a wheel needs to hold `speed` while accelerating at `accel`, from the motor model in config.h
static float wheelFeedforward(float speed_mm_s, float accel_mm_s2) {
    float effort = FF_KV * speed_mm_s + FF_KA * accel_mm_s2;
    if (speed_mm_s > 5.0f)  effort += FF_KS;
    if (speed_mm_s < -5.0f) effort -= FF_KS;
    return effort;
}

void MotionController::startCurve(const MotionCommand& cmd, float angle_deg, float default_length_mm) {
    // A smooth turn is driven like a straight of `curve_length_mm_` along the path, while the
    // heading target follows the distance covered (see update()). Tying heading to distance
    // rather than to time keeps the robot on the planned curve even if it runs slow or fast.
    curve_length_mm_ = (cmd.param_value > 10.0f) ? cmd.param_value : default_length_mm;
    curve_active_ = true;
    target_relative_dist_mm_ = curve_length_mm_;
    target_relative_angle_deg_ = angle_deg;

    float speed = (cmd.max_speed_mm_s > 0.0f) ? cmd.max_speed_mm_s : SEARCH_CURVE_SPEED_MM_S;
    profile_linear_.start(curve_length_mm_, speed, cmd.acceleration, cmd.entry_speed_mm_s, cmd.exit_speed_mm_s);
    profile_angular_.stop();
}

void MotionController::samplePreview(float dist_in_move_mm) {
    IRReadings ir = ir_.getReadings();

    if (curve_active_) {
        // Half-way round a 90° curve the outer 45° sensor faces the front wall of the cell squarely
        float u = dist_in_move_mm / curve_length_mm_;
        if (fabsf(target_relative_angle_deg_) > 60.0f && u >= SEARCH_FRONT_SAMPLE_FROM && u <= SEARCH_FRONT_SAMPLE_TO) {
            front_uses_left_sensor_ = (target_relative_angle_deg_ < 0.0f); // Right turn -> left sensor is outside
            uint16_t reading = front_uses_left_sensor_ ? ir.left_45 : ir.right_45;
            if (front_samples_ == 0 || reading < front_min_) front_min_ = reading;
            if (front_samples_ == 0 || reading > front_max_) front_max_ = reading;
            front_samples_++;
        }
        return;
    }

    bool is_straight = (active_cmd_.action == ACTION_MOVE_FORWARD_CELLS || active_cmd_.action == ACTION_MOVE_DISTANCE ||
                        active_cmd_.action == ACTION_MOVE_HALF_CELL);
    if (is_straight && dist_in_move_mm >= SEARCH_LOOKAHEAD_START_MM && dist_in_move_mm <= SEARCH_LOOKAHEAD_END_MM) {
        if (preview_samples_ == 0 || ir.left_45  < preview_min_l_) preview_min_l_ = ir.left_45;
        if (preview_samples_ == 0 || ir.left_45  > preview_max_l_) preview_max_l_ = ir.left_45;
        if (preview_samples_ == 0 || ir.right_45 < preview_min_r_) preview_min_r_ = ir.right_45;
        if (preview_samples_ == 0 || ir.right_45 > preview_max_r_) preview_max_r_ = ir.right_45;
        preview_samples_++;
    }
}

WallPreview MotionController::getWallPreview() const {
    // A wall is only reported if EVERY sample in the window was above the wall threshold, and an
    // opening only if every sample was well below it. Anything in between is "not sure".
    WallPreview preview = {};
    const float wall_l = (float)ir_.getThresholdL45();
    const float wall_r = (float)ir_.getThresholdR45();

    if (preview_samples_ >= PREVIEW_MIN_SAMPLES) {
        preview.left_wall  = (float)preview_min_l_ > wall_l;
        preview.left_open  = (float)preview_max_l_ < wall_l * SEARCH_OPEN_RATIO;
        preview.right_wall = (float)preview_min_r_ > wall_r;
        preview.right_open = (float)preview_max_r_ < wall_r * SEARCH_OPEN_RATIO;
    }
    if (front_samples_ >= PREVIEW_MIN_SAMPLES) {
        const float wall = front_uses_left_sensor_ ? wall_l : wall_r;
        preview.front_wall = (float)front_min_ > wall;
        preview.front_open = (float)front_max_ < wall * SEARCH_OPEN_RATIO;
    }
    return preview;
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

    // Walls start and end at posts, which sit at the same place in every cell. Compare where
    // the encoders think we are within the cell against where the post says we are.
    float phase = fmodf(dist_in_seg + active_cmd_.start_offset_mm, MAZE_CELL_SIZE_MM);
    float drift = phase - POST_EDGE_PHASE_MM;

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
            float forward_effort = pid_linear_vel_.update(vel_err, dt_seconds) +
                                   wheelFeedforward(active_cmd_.exit_speed_mm_s, 0.0f);
            motors_.setEffort(forward_effort, forward_effort);
            return;
        }
        chain_valid_ = false;
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

        // Feedforward: most of the effort comes straight from the planned speed and acceleration
        float target_accel = (target_vel - prev_target_speed_mm_s_) / dt_seconds;
        prev_target_speed_mm_s_ = target_vel;
        forward_effort += wheelFeedforward(target_vel, target_accel);
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
        rotational_feedforward = FF_KV * wheel_delta_mm_s;
    } else if (!profile_angular_.isFinished()) {
        profile_angular_.update(dt_seconds);
        target_heading_setpoint = profile_angular_.getTargetDistance();

        // Turn on the spot: each wheel runs at (yaw rate x half the wheelbase), in opposite directions
        float wheel_mm_s = profile_angular_.getTargetVelocity() * (PI / 180.0f) * (WHEEL_BASE_MM * 0.5f);
        rotational_feedforward = wheelFeedforward(wheel_mm_s, 0.0f);
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
                wall_trim_angle -= (gyro_z * 0.035f);

                target_heading_setpoint += wall_trim_angle;
            } else {
                pid_wall_centering_.reset();
            }
        }
    }

    // Shortest-path angular error normalization in (-180, +180]
    float heading_err = shortestAngularDifference(target_heading_setpoint, heading_traveled);
    rotational_effort = pid_angular_heading_.update(heading_err, dt_seconds) + rotational_feedforward;

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
        command_finished_ = true;
        command_active_ = false;
        curve_active_ = false;
        chained_coast_timer_ = 0;
        // Brake if stopping, or maintain velocity if exit speed was requested
        if (active_cmd_.exit_speed_mm_s <= 10.0f) {
            motors_.brake();
        } else {
            chain_valid_ = true;
            chain_end_distance_mm_ = start_distance_mm_ + target_relative_dist_mm_;
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
    curve_active_ = false;
    chain_valid_ = false;
    command_active_ = false;
    command_finished_ = true;
    chained_coast_timer_ = 255;
    motors_.brake();
}

void MotionController::setWallCenteringEnabled(bool enabled) {
    wall_centering_enabled_ = enabled;
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
