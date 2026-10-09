#include "control/motion_controller/motion_controller.h"

// Setting up, starting a move, and the small helpers. The 500 Hz loop itself
// (MotionController::update) is in motion_update.cpp.

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
      preview_reported_(false), preview_ready_(false),
      front_min_(0), front_max_(0), front_samples_(0), front_uses_left_sensor_(false),
      diag_peak_left_(0.0f), diag_peak_right_(0.0f),
      settle_ticks_(0), settle_good_ticks_(0), idle_ticks_(0),
      prev_target_speed_mm_s_(0.0f), log_tick_(0),
      wall_centering_enabled_(true), calibrating_motors_(false), safety_stop_(false), late_handovers_(0),
      tune_{} {

    // Fixed limits. Every gain, and the heading and wall-centring limits, come from the tuning
    // table (kTune, further down), so they can all be changed from the phone app.
    pid_linear_dist_.setOutputLimits(SPEEDRUN_DIAG_SPEED_MM_S, 200.0f); // Distance loop: outputs a speed (mm/s)
    pid_linear_vel_.setOutputLimits(1.0f, 0.3f);                        // Speed loop: outputs motor effort (-1 to 1)
    for (int i = 0; i < TUNE_COUNT; ++i) tune_[i] = tuneDefault(i);
    applyTuning();
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
    chain_valid_ = false;
    curve_active_ = false;
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

// The robot has just been squared up against a wall, so it is pointing exactly along the maze
// grid. Snap the heading reference to the nearest grid direction, removing whatever error had
// built up. (It must stay the SAME direction it was tracking: every later move is measured
// from it.)
void MotionController::snapHeadingToGrid() {
    float grid_heading = roundf(accumulated_heading_deg_ / 90.0f) * 90.0f;
    imu_.resetHeading(normalizeAngle180(grid_heading));
    accumulated_heading_deg_ = grid_heading;
    start_heading_deg_ = grid_heading;
    prev_raw_heading_deg_ = imu_.getHeadingDeg();
    pid_angular_heading_.reset();
    pid_wall_centering_.reset();
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

    // Where this move starts. When it follows on from another move, start from exactly where
    // that move was supposed to end, so small tracking errors are corrected by this move instead
    // of piling up along a chain of moves.
    EncoderState enc = encoders_.getState();
    if (chain_valid_) {
        start_distance_mm_ = chain_end_distance_mm_;
    } else {
        start_distance_mm_ = (enc.left_dist_mm + enc.right_dist_mm) * 0.5f;
    }
    chain_valid_ = false;
    curve_active_ = false;
    prev_target_speed_mm_s_ = cmd.entry_speed_mm_s;
    settle_ticks_ = 0;
    settle_good_ticks_ = 0;
    idle_ticks_ = 0;
    preview_samples_ = 0;
    preview_reported_ = false;
    front_samples_ = 0;
    diag_peak_left_ = 0.0f;
    diag_peak_right_ = 0.0f;

    // Grid Axis Snapping: every move starts from an exact maze heading (a multiple of 45°), so a
    // few degrees of error left over from the previous turn is steered out instead of carried on.
    // Straights hold a cardinal heading; diagonals, smooth curves and turns on the spot start
    // from any multiple of 45°.
    bool is_cardinal_move = (cmd.action == ACTION_MOVE_FORWARD_CELLS || cmd.action == ACTION_MOVE_DISTANCE ||
                             cmd.action == ACTION_MOVE_HALF_CELL);
    bool is_grid_move = (cmd.action == ACTION_MOVE_DIAGONAL_HALF ||
                         cmd.action == ACTION_CURVE_LEFT_90 || cmd.action == ACTION_CURVE_RIGHT_90 ||
                         cmd.action == ACTION_CURVE_LEFT_45 || cmd.action == ACTION_CURVE_RIGHT_45 ||
                         cmd.action == ACTION_TURN_LEFT_45  || cmd.action == ACTION_TURN_RIGHT_45 ||
                         // Turns on the spot too. A straight can end a few degrees off the grid
                         // (wall centring steers it by up to 25 degrees), and a turn measured from
                         // there would end the same amount off: 90 right from 8 right of straight
                         // is 98. Measured from the grid heading it ends square.
                         cmd.action == ACTION_TURN_LEFT_90  || cmd.action == ACTION_TURN_RIGHT_90 ||
                         cmd.action == ACTION_TURN_AROUND_180);
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

// Effort a wheel needs to hold `speed` while accelerating at `accel`, from the motor model
// (ff_ks, ff_kv, ff_ka in the tuning table; they start at FF_KS, FF_KV, FF_KA from config.h)
float MotionController::wheelFeedforward(float speed_mm_s, float accel_mm_s2) const {
    float effort = tune_[TUNE_FF_KV] * speed_mm_s + tune_[TUNE_FF_KA] * accel_mm_s2;
    if (speed_mm_s > 5.0f)  effort += tune_[TUNE_FF_KS];
    if (speed_mm_s < -5.0f) effort -= tune_[TUNE_FF_KS];
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
        // At the end of the curve, nearly square to the next cell: its side walls (see
        // SEARCH_CURVE_SIDE_SAMPLE_DEG). Judged by heading, not distance, so it does not depend
        // on the encoders.
        const float heading_left_deg = target_relative_angle_deg_ - (accumulated_heading_deg_ - start_heading_deg_);
        if (fabsf(target_relative_angle_deg_) > 60.0f && u > SEARCH_FRONT_SAMPLE_TO &&
            fabsf(heading_left_deg) < SEARCH_CURVE_SIDE_SAMPLE_DEG) {
            if (preview_samples_ == 0 || ir.left_45  < preview_min_l_) preview_min_l_ = ir.left_45;
            if (preview_samples_ == 0 || ir.left_45  > preview_max_l_) preview_max_l_ = ir.left_45;
            if (preview_samples_ == 0 || ir.right_45 < preview_min_r_) preview_min_r_ = ir.right_45;
            if (preview_samples_ == 0 || ir.right_45 > preview_max_r_) preview_max_r_ = ir.right_45;
            preview_samples_++;
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
    if (is_straight && dist_in_move_mm > SEARCH_LOOKAHEAD_END_MM && preview_samples_ > 0 && !preview_reported_) {
        preview_reported_ = true;
        preview_ready_ = true; // The window has closed: the verdict is final for this move
    }
}

void MotionController::printPreviewReport() const {
    const WallPreview preview = getWallPreview();
    const char* left  = preview.left_wall  ? "WALL" : (preview.left_open  ? "open" : "not sure");
    const char* right = preview.right_wall ? "WALL" : (preview.right_open ? "open" : "not sure");
    Serial.printf("[LOOK] 45° sensors decided %d-%d mm past the cell centre. Next cell: left %s (read %u-%u, wall above %u), right %s (read %u-%u, wall above %u)%s\n",
                  (int)SEARCH_LOOKAHEAD_START_MM, (int)SEARCH_LOOKAHEAD_END_MM,
                  left,  (unsigned)preview_min_l_, (unsigned)preview_max_l_, (unsigned)ir_.getThresholdL45(),
                  right, (unsigned)preview_min_r_, (unsigned)preview_max_r_, (unsigned)ir_.getThresholdR45(),
                  (preview.left_wall && preview.right_wall) ? " -> walls on BOTH sides" : "");
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

// ---------------------------------------------------------------- Live tuning
// Every gain that can be changed from the phone app, with the value it has in the code and the
// range a typed value must fall inside (a slipped decimal point should not reach the motors).
// To change a default for good, edit `value` here (or the config.h constant it names).
struct TuneItem {
    const char* name;         // Typed in the `tune` command; also the key it is saved under
    const char* description;
    float value;
    float min, max;
};
static const TuneItem kTune[MotionController::TUNE_COUNT] = {
    // --- Speed loop: wheel speed error -> motor effort
    { "v_kp",   "Speed loop P",                         0.0025f,  0.0f,  0.02f  },
    { "v_ki",   "Speed loop I",                         0.0005f,  0.0f,  0.01f  },
    { "v_kd",   "Speed loop D",                         0.00005f, 0.0f,  0.001f },
    // --- Heading loop: degrees off course -> turning effort. This is what keeps the robot
    //     straight where there are no walls, and what finishes every turn.
    //     Heading I = 0.025 neutralizes motor/gearbox friction asymmetry within 150 ms
    { "h_kp",   "Heading loop P",                       0.045f,   0.0f,  0.5f   },
    { "h_ki",   "Heading loop I",                       0.025f,   0.0f,  0.3f   },
    { "h_kd",   "Heading loop D",                       0.0012f,  0.0f,  0.02f  },
    { "k_sync", "Wheel sync (holds both wheels equal)", 0.0004f,  0.0f,  0.005f },
    { "enc_a",  "Wheel-speed smoothing (1 = none)",     ENCODER_SPEED_FILTER_ALPHA, 0.02f, 1.0f },
    { "imu_a",  "Heading smoothing (1 = none)",         IMU_FILTER_ALPHA,           0.1f,  1.0f },
    // New value = old value x 180 / the distance the robot really drove for one cell
    { "dist_k", "Distance scale (raise if it stops short)", 1.0f,                     0.5f,  2.0f },
    // --- Distance loop: mm short of where it should be by now -> extra speed (mm/s)
    { "d_kp",   "Distance loop P",                      3.5f,     0.0f,  20.0f  },
    { "d_ki",   "Distance loop I",                      0.0f,     0.0f,  5.0f   },
    { "d_kd",   "Distance loop D",                      0.1f,     0.0f,  2.0f   },
    // --- Wall centring: IR centring error -> degrees added to the heading target
    { "w_kp",   "Wall centring P",                      35.0f,    0.0f,  200.0f },
    { "w_ki",   "Wall centring I",                      0.0f,     0.0f,  50.0f  },
    { "w_kd",   "Wall centring D",                      3.5f,     0.0f,  50.0f  },
    // --- Feedforward: the effort sent straight from the planned motion, before any loop acts
    { "ff_ks",  "Feedforward: effort to overcome friction",   FF_KS, 0.0f, 0.3f   },
    { "ff_kv",  "Feedforward: effort per mm/s of wheel speed", FF_KV, 0.0f, 0.01f  },
    { "ff_ka",  "Feedforward: effort per mm/s2",               FF_KA, 0.0f, 0.001f },
    { "turn_ff", "Turn feedforward scale (lower if turns overshoot)", 1.0f, 0.0f, 3.0f },
    // --- Limits
    { "h_max",  "Heading loop: most effort it may use",       0.6f,   0.05f, 1.0f },
    { "h_imax", "Heading loop: most of that from I",          0.12f,  0.0f,  1.0f },
    { "w_max",  "Wall centring: most steering, degrees",      25.0f,  0.0f,  45.0f },
    { "w_gyro", "Wall centring: turn-rate damping",           0.035f, 0.0f,  0.2f },
    // --- Turn-rate damping and the nudge after a turn on the spot (they start at the values in config.h)
    { "s_damp", "Straights: turn-rate damping (0 = off)",     STRAIGHT_YAW_DAMPING, 0.0f, 0.005f },
    { "t_damp", "After a turn: turn-rate damping (0 = off)",  TURN_SETTLE_DAMPING,  0.0f, 0.005f },
    { "t_push", "After a turn: nudge onto the heading (0 = off)", TURN_SETTLE_PUSH, 0.0f, 0.3f },
    // --- Extra heading loop P during smooth curves only, on top of h_kp (0 = curves use h_kp alone)
    { "c_kp",   "Curves: extra heading P (0 = off)",          0.0f,   0.0f,  0.2f },
};

const char* MotionController::tuneName(int index)        { return kTune[index].name; }
const char* MotionController::tuneDescription(int index) { return kTune[index].description; }
float MotionController::tuneDefault(int index)           { return kTune[index].value; }

float MotionController::getTune(int index) const {
    return (index >= 0 && index < TUNE_COUNT) ? tune_[index] : 0.0f;
}

bool MotionController::setTune(int index, float value) {
    if (index < 0 || index >= TUNE_COUNT) return false;
    if (!(value >= kTune[index].min && value <= kTune[index].max)) return false; // Also rejects NaN
    tune_[index] = value;
    applyTuning();
    return true;
}

// Hands every tuning value to the part of the controller that uses it. (The feedforward, turn
// scale and gyro damping are read from tune_ directly where they are used.)
void MotionController::applyTuning() {
    pid_linear_vel_.setGains(tune_[TUNE_V_KP], tune_[TUNE_V_KI], tune_[TUNE_V_KD]);
    pid_angular_heading_.setGains(tune_[TUNE_H_KP], tune_[TUNE_H_KI], tune_[TUNE_H_KD]);
    pid_angular_heading_.setOutputLimits(tune_[TUNE_H_MAX], tune_[TUNE_H_IMAX]);
    pid_linear_dist_.setGains(tune_[TUNE_D_KP], tune_[TUNE_D_KI], tune_[TUNE_D_KD]);
    pid_wall_centering_.setGains(tune_[TUNE_W_KP], tune_[TUNE_W_KI], tune_[TUNE_W_KD]);
    pid_wall_centering_.setOutputLimits(tune_[TUNE_W_MAX], 5.0f);
    encoders_.setSpeedFilterAlpha(tune_[TUNE_ENC_A]);
    imu_.setFilterAlpha(tune_[TUNE_IMU_A]);
    encoders_.setDistanceScale(tune_[TUNE_DIST_K]);
}

void MotionController::forgetSavedTuning() {
    Preferences prefs;
    prefs.begin("motion_cal", false);
    prefs.clear();
    prefs.end();
    for (int i = 0; i < TUNE_COUNT; ++i) setTune(i, tuneDefault(i));
    Serial.println("[MOTION] Saved tuning wiped: back to the values in the code.");
}

void MotionController::saveToNVS() {
    Preferences prefs;
    prefs.begin("motion_cal", false);
    for (int i = 0; i < TUNE_COUNT; ++i) prefs.putFloat(kTune[i].name, getTune(i));
    prefs.putBool("valid", true);
    prefs.end();
    Serial.println("[MOTION] Tuning saved: it will be used again after power-off.");
}

bool MotionController::loadFromNVS() {
    Preferences prefs;
    prefs.begin("motion_cal", true);
    if (!prefs.getBool("valid", false)) {
        prefs.end();
        return false;
    }
    // A value that was never saved, or is out of range, keeps the value in the code
    for (int i = 0; i < TUNE_COUNT; ++i) setTune(i, prefs.getFloat(kTune[i].name, getTune(i)));
    prefs.end();
    Serial.println("[MOTION] Using the tuning SAVED ON THE ROBOT, not the values in the code ('tune' lists it, 'tune reset' wipes it).");
    return true;
}
