#include "hardware.h"

// ==============================================================================
// ENCODERS
// ==============================================================================

Encoders::Encoders()
    : invert_left_(INVERT_LEFT_ENCODER),
      invert_right_(INVERT_RIGHT_ENCODER),
      left_dist_acc_mm_(0.0f),
      right_dist_acc_mm_(0.0f) {
    memset(&state_, 0, sizeof(state_));
}

void Encoders::begin() {
    // Unit 0 for Left motor encoder
    initPcntUnit(PCNT_UNIT_0, PIN_ENC_L_A, PIN_ENC_L_B);

    // Unit 1 for Right motor encoder
    initPcntUnit(PCNT_UNIT_1, PIN_ENC_R_A, PIN_ENC_R_B);

    reset();
}

void Encoders::initPcntUnit(pcnt_unit_t unit, int pin_a, int pin_b) {
    // Enable internal pullups for N20 magnetic Hall effect encoder sensors
    pinMode(pin_a, INPUT_PULLUP);
    pinMode(pin_b, INPUT_PULLUP);

    // Channel 0: Pulse on Pin A, Level on Pin B
    pcnt_config_t config_ch0 = {
        .pulse_gpio_num = pin_a,
        .ctrl_gpio_num = pin_b,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_REVERSE,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = unit,
        .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&config_ch0);

    // Channel 1: Pulse on Pin B, Level on Pin A (Completes 4x quadrature decoding)
    pcnt_config_t config_ch1 = {
        .pulse_gpio_num = pin_b,
        .ctrl_gpio_num = pin_a,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .counter_h_lim = 32767,
        .counter_l_lim = -32768,
        .unit = unit,
        .channel = PCNT_CHANNEL_1,
    };
    pcnt_unit_config(&config_ch1);

    // Glitch filter: ignore pulses shorter than ~100 APB clock cycles (~1.25 µs)
    pcnt_set_filter_value(unit, 100);
    pcnt_filter_enable(unit);

    // Clear and resume counting
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void Encoders::update(float dt_seconds) {
    if (dt_seconds <= 0.0f) {
        dt_seconds = CONTROL_DT_S;
    }

    int16_t raw_left = 0;
    int16_t raw_right = 0;

    // Read counter hardware registers and immediately clear for the next delta
    pcnt_get_counter_value(PCNT_UNIT_0, &raw_left);
    pcnt_counter_clear(PCNT_UNIT_0);

    pcnt_get_counter_value(PCNT_UNIT_1, &raw_right);
    pcnt_counter_clear(PCNT_UNIT_1);

    if (invert_left_)  raw_left  = -raw_left;
    if (invert_right_) raw_right = -raw_right;

    state_.left_delta_ticks = raw_left;
    state_.right_delta_ticks = raw_right;

    state_.left_ticks_total += raw_left;
    state_.right_ticks_total += raw_right;

    // Convert ticks to millimeters
    float d_left_mm = (float)raw_left * MM_PER_TICK;
    float d_right_mm = (float)raw_right * MM_PER_TICK;

    left_dist_acc_mm_ += d_left_mm;
    right_dist_acc_mm_ += d_right_mm;

    state_.left_dist_mm = left_dist_acc_mm_;
    state_.right_dist_mm = right_dist_acc_mm_;

    // Instantaneous wheel speeds (mm/s)
    float raw_speed_l = d_left_mm / dt_seconds;
    float raw_speed_r = d_right_mm / dt_seconds;

    // First-order low-pass filter on speeds (alpha = 0.6) for smoother PID derivative
    const float alpha = 0.6f;
    state_.left_speed_mm_s  = (alpha * raw_speed_l) + ((1.0f - alpha) * state_.left_speed_mm_s);
    state_.right_speed_mm_s = (alpha * raw_speed_r) + ((1.0f - alpha) * state_.right_speed_mm_s);

    state_.linear_speed_mm_s = (state_.left_speed_mm_s + state_.right_speed_mm_s) * 0.5f;
}

EncoderState Encoders::getState() const {
    return state_;
}

void Encoders::reset() {
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_1);
    left_dist_acc_mm_ = 0.0f;
    right_dist_acc_mm_ = 0.0f;
    memset(&state_, 0, sizeof(state_));
}

void Encoders::setInverted(bool invert_left, bool invert_right) {
    invert_left_ = invert_left;
    invert_right_ = invert_right;
}

void Encoders::getInverted(bool& invert_left, bool& invert_right) const {
    invert_left = invert_left_;
    invert_right = invert_right_;
}

// ==============================================================================
// MOTORS
// ==============================================================================

#include <Preferences.h>

Motors::Motors()
    : invert_left_(INVERT_LEFT_MOTOR),
      invert_right_(INVERT_RIGHT_MOTOR),
      power_enabled_(false),
      trim_left_(1.0f),
      trim_right_(1.0f),
      deadband_left_(0.02f),
      deadband_right_(0.02f),
      is_braking_(false),
      last_left_cmd_(-9999),
      last_right_cmd_(-9999),
      last_cmd_time_ms_(0),
      connected_(false),
      status_flags_(0),
      last_health_ms_(0),
      reset_recoveries_(0),
      bus_faults_(0),
      supply_volts_(0.0f) {}

void Motors::begin() {
    // 1. Configure Hardware Reset and Boost Converter Enable pins
    pinMode(PIN_MOTOR_BAT_CTRL, OUTPUT);
    digitalWrite(PIN_MOTOR_BAT_CTRL, LOW); // Start with 12V boost disabled

    pinMode(PIN_MOTOR_RST, OUTPUT);
    digitalWrite(PIN_MOTOR_RST, HIGH);

    // 2. Enable 12V motor battery boost converter
    setMotorPowerEnabled(true);
    delay(300); // Allow 12V rail to stabilize (same delay as the proven bench tests)

    // 3. Bring up the Motoron, retrying with a longer post-reset wait if it does not answer
    mc_.setAddress(MOTORON_I2C_ADDR);
    connected_ = false;
    for (int attempt = 0; attempt < 3 && !connected_; ++attempt) {
        digitalWrite(PIN_MOTOR_RST, LOW);
        delay(5);
        digitalWrite(PIN_MOTOR_RST, HIGH);
        delay(10 + attempt * 50);
        connected_ = configureMotoron();
    }
    if (!connected_) {
        Serial.println("[MOTORS] ERROR: Motoron M2T256 not responding on I2C! Will keep retrying in the background.");
    }

    // 4. Load calibration trims from Flash NVS
    loadFromNVS();

    is_braking_ = false;
    last_left_cmd_ = -9999;
    last_right_cmd_ = -9999;
    last_cmd_time_ms_ = 0;
    last_health_ms_ = millis();

    coast();
}

bool Motors::configureMotoron() {
    mc_.reinitialize();
    mc_.clearResetFlag();

    // Set Motoron internal acceleration to 0 (unlimited) so our software PID
    // and trapezoidal trajectory generator have direct instantaneous control
    mc_.setMaxAcceleration(1, 0);
    mc_.setMaxDeceleration(1, 0);
    mc_.setMaxAcceleration(2, 0);
    mc_.setMaxDeceleration(2, 0);

    // A successful read-back proves two-way communication and that the reset flag really cleared
    status_flags_ = mc_.getStatusFlags();
    return mc_.getLastError() == 0 && !(status_flags_ & (1 << MOTORON_STATUS_FLAG_RESET));
}

void Motors::maintain(uint32_t now) {
    if (now - last_health_ms_ < MOTORON_HEALTH_PERIOD_MS) return;
    last_health_ms_ = now;

    uint16_t flags = mc_.getStatusFlags();
    bool needs_config = false;

    if (mc_.getLastError() != 0) {
        // I2C bus fault or Motoron unpowered
        if (connected_) bus_faults_++;
        connected_ = false;
        needs_config = true;
    } else {
        status_flags_ = flags;
        if (flags & (1 << MOTORON_STATUS_FLAG_RESET)) {
            // The Motoron rebooted (e.g. 12V rail sag) and refuses to drive until it is reconfigured
            reset_recoveries_++;
            needs_config = true;
        } else {
            connected_ = true;

            // Motor supply voltage, lightly smoothed, for effort compensation in sendEfforts()
            float volts = (float)mc_.getVinVoltageMv(3300) / 1000.0f;
            if (mc_.getLastError() == 0 && volts > 1.0f) {
                supply_volts_ = (supply_volts_ < 1.0f) ? volts : (0.8f * supply_volts_ + 0.2f * volts);
            }
        }
    }

    if (needs_config) {
        connected_ = configureMotoron();
        last_cmd_time_ms_ = now - MOTORON_KEEPALIVE_MS; // Force the next command to be re-sent
    }
}

void Motors::setMotorPowerEnabled(bool enabled) {
    power_enabled_ = enabled;
    // MotorBatControl drives NPN base HIGH, which pulls PMOS gate LOW and switches battery to 12V boost
    digitalWrite(PIN_MOTOR_BAT_CTRL, enabled ? HIGH : LOW);
}

void Motors::setEffort(float left_effort, float right_effort) {
    // Apply motor balance calibration trims
    sendEfforts(left_effort * trim_left_, right_effort * trim_right_);
}

void Motors::setRawEffort(float left_effort, float right_effort) {
    // Direct effort bypassing trim multipliers (used for calibration benchmarking)
    sendEfforts(left_effort, right_effort);
}

void Motors::sendEfforts(float left_effort, float right_effort) {
    if (!power_enabled_) return;

    if (invert_left_)  left_effort  = -left_effort;
    if (invert_right_) right_effort = -right_effort;

    // Supply compensation: "effort" means a fraction of MOTOR_NOMINAL_VOLTS. If the motor supply
    // has sagged (or is running high), scale the duty so the motors still get the voltage asked for.
    if (supply_volts_ > 6.0f) {
        float scale = constrain(MOTOR_NOMINAL_VOLTS / supply_volts_, 0.8f, 1.3f);
        left_effort  *= scale;
        right_effort *= scale;
    }

    // Clamp to range [-1.0, 1.0]
    left_effort  = constrain(left_effort,  -1.0f, 1.0f);
    right_effort = constrain(right_effort, -1.0f, 1.0f);

    // Deadband check
    int16_t left_cmd = 0;
    int16_t right_cmd = 0;

    if (fabsf(left_effort) >= deadband_left_) {
        left_cmd = (int16_t)(left_effort * (float)MOTORON_MAX_SPEED);
    }
    if (fabsf(right_effort) >= deadband_right_) {
        right_cmd = (int16_t)(right_effort * (float)MOTORON_MAX_SPEED);
    }

    // Schematic: M1 = Right Motor (R_MOTOR), M2 = Left Motor (L_MOTOR)
    // Send both motor speeds in a single I2C transaction
    // Only transmit if speed changed, transitioning from braking, or periodic keep-alive
    uint32_t now = millis();
    if (is_braking_ || left_cmd != last_left_cmd_ || right_cmd != last_right_cmd_ ||
        (now - last_cmd_time_ms_ >= MOTORON_KEEPALIVE_MS)) {
        mc_.setAllSpeedsNow(right_cmd, left_cmd);
        last_left_cmd_ = left_cmd;
        last_right_cmd_ = right_cmd;
        is_braking_ = false;
        last_cmd_time_ms_ = now;
    }

    maintain(now);
}

void Motors::setTrim(float trim_left, float trim_right) {
    trim_left_ = constrain(trim_left, 0.5f, 1.0f);
    trim_right_ = constrain(trim_right, 0.5f, 1.0f);
}

void Motors::getTrim(float& trim_left, float& trim_right) const {
    trim_left = trim_left_;
    trim_right = trim_right_;
}

void Motors::setDeadband(float deadband_left, float deadband_right) {
    deadband_left_ = constrain(deadband_left, 0.005f, 0.20f);
    deadband_right_ = constrain(deadband_right, 0.005f, 0.20f);
}

void Motors::getDeadband(float& deadband_left, float& deadband_right) const {
    deadband_left = deadband_left_;
    deadband_right = deadband_right_;
}

void Motors::saveToNVS() {
    Preferences prefs;
    prefs.begin("motor_cal", false);
    prefs.putFloat("trim_l", trim_left_);
    prefs.putFloat("trim_r", trim_right_);
    prefs.putFloat("dead_l", deadband_left_);
    prefs.putFloat("dead_r", deadband_right_);
    prefs.putBool("valid", true);
    prefs.end();
    Serial.printf("[MOTORS] Calibration saved to NVS: Trim=[%.4f, %.4f], Deadband=[%.4f, %.4f]\n",
                  trim_left_, trim_right_, deadband_left_, deadband_right_);
}

bool Motors::loadFromNVS() {
    Preferences prefs;
    prefs.begin("motor_cal", true);
    if (!prefs.getBool("valid", false)) {
        prefs.end();
        trim_left_ = 1.0f;
        trim_right_ = 1.0f;
        deadband_left_ = 0.02f;
        deadband_right_ = 0.02f;
        return false;
    }
    trim_left_ = prefs.getFloat("trim_l", 1.0f);
    trim_right_ = prefs.getFloat("trim_r", 1.0f);
    deadband_left_ = prefs.getFloat("dead_l", 0.02f);
    deadband_right_ = prefs.getFloat("dead_r", 0.02f);
    prefs.end();
    Serial.printf("[MOTORS] Calibration loaded from NVS: Trim=[%.4f, %.4f], Deadband=[%.4f, %.4f]\n",
                  trim_left_, trim_right_, deadband_left_, deadband_right_);
    return true;
}

void Motors::brake() {
    if (!power_enabled_) return;

    // Filter redundant brake calls at 500 Hz, but keep re-sending so the Motoron's
    // command timeout never releases the brake while the robot is holding position
    uint32_t now = millis();
    if (!is_braking_ || (now - last_cmd_time_ms_ >= MOTORON_KEEPALIVE_MS)) {
        mc_.setBrakingNow(1, MOTORON_MAX_SPEED);
        mc_.setBrakingNow(2, MOTORON_MAX_SPEED);
        is_braking_ = true;
        last_left_cmd_ = -9999;
        last_right_cmd_ = -9999;
        last_cmd_time_ms_ = now;
    }

    maintain(now);
}

void Motors::coast() {
    if (!power_enabled_) return;
    mc_.setAllSpeedsNow(0, 0);
    is_braking_ = false;
    last_left_cmd_ = 0;
    last_right_cmd_ = 0;
    last_cmd_time_ms_ = millis();
}

void Motors::setInverted(bool invert_left, bool invert_right) {
    invert_left_ = invert_left;
    invert_right_ = invert_right;
}

void Motors::getInverted(bool& invert_left, bool& invert_right) const {
    invert_left = invert_left_;
    invert_right = invert_right_;
}

// ==============================================================================
// IR WALL SENSORS
// ==============================================================================

IRSensors::IRSensors()
    : filtered_{0, 0, 0, 0, 0, 0},
      history_{},
      update_count_(0),
      prev_l90_(0),
      prev_r90_(0),
      thresh_l90_(WALL_THRESH_L90),
      thresh_l45_(WALL_THRESH_L45),
      thresh_front_(WALL_THRESH_FRONT),
      thresh_r45_(WALL_THRESH_R45),
      thresh_r90_(WALL_THRESH_R90),
      nominal_center_l45_(NOMINAL_CENTER_L45),
      nominal_center_r45_(NOMINAL_CENTER_R45) {
    memset(&readings_, 0, sizeof(readings_));
}

// Channel order everywhere in this file: CH1..CH6 = L90, L45, FL, FR, R45, R90
static const uint8_t kEmitterPins[6]  = { PIN_IR_E1, PIN_IR_E2, PIN_IR_E3, PIN_IR_E4, PIN_IR_E5, PIN_IR_E6 };
static const uint8_t kReceiverPins[6] = { PIN_IR_R1, PIN_IR_R2, PIN_IR_R3, PIN_IR_R4, PIN_IR_R5, PIN_IR_R6 };

// Interleaved firing groups (channel indices). The two front sensors and each side's 90°/45° pair
// are split across groups so sensors aimed at the same wall never fire together.
static const uint8_t kGroups[2][3] = {
    { 0, 2, 5 }, // Group A: L90, FL, R90
    { 1, 3, 4 }  // Group B: L45, FR, R45
};

void IRSensors::begin() {
    for (int i = 0; i < 6; ++i) {
        pinMode(kEmitterPins[i], OUTPUT);
        digitalWrite(kEmitterPins[i], LOW);
        pinMode(kReceiverPins[i], INPUT);
    }

    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // Try loading previously saved calibration from NVS Flash
    if (!loadFromNVS()) {
        Serial.println("[IR] No stored calibration found in Flash; using defaults from config.h.");
    }
}

void IRSensors::sampleGroup(const uint8_t group[3]) {
    uint16_t ambient[3];

    // 1. Measure ambient light with the group's emitters OFF
    for (int i = 0; i < 3; ++i) {
        ambient[i] = analogRead(kReceiverPins[group[i]]);
    }

    // 2. Pulse the group's emitters ON together and let the phototransistors settle
    for (int i = 0; i < 3; ++i) {
        digitalWrite(kEmitterPins[group[i]], HIGH);
    }
    delayMicroseconds(IR_PULSE_SETTLE_US);

    // 3. Sample reflected signal, ambient-subtracted and low-pass filtered
    for (int i = 0; i < 3; ++i) {
        const uint8_t ch = group[i];
        uint16_t lit = analogRead(kReceiverPins[ch]);
        uint16_t raw = (lit > ambient[i]) ? (lit - ambient[i]) : 0;

        // Median of the last three samples throws away a single-sample spike (sunlight flicker,
        // a camera flash, electrical noise) without smearing real edges the way averaging would
        uint16_t a = raw, b = history_[ch][0], c = history_[ch][1];
        uint16_t median = max(min(a, b), min(max(a, b), c));
        history_[ch][1] = history_[ch][0];
        history_[ch][0] = raw;

        filtered_[ch] = (uint16_t)(IR_FILTER_ALPHA * median + (1.0f - IR_FILTER_ALPHA) * filtered_[ch]);
    }

    // 4. Emitters OFF
    for (int i = 0; i < 3; ++i) {
        digitalWrite(kEmitterPins[group[i]], LOW);
    }
}

// How far away a wall is, as a fraction of its distance when the robot is centred (1.0 = centred,
// below 1 = closer, above 1 = further). Reflected IR falls off with the square of distance, so
// distance goes as 1/sqrt(reading). Working in distance makes the steering error proportional to
// how far off-centre the robot actually is, instead of exploding as it nears a wall.
static float relativeDistance(uint16_t reading, uint16_t centred_reading) {
    float r = (reading > 1) ? (float)reading : 1.0f;
    return sqrtf((float)centred_reading / r);
}

void IRSensors::update() {
    sampleGroup(kGroups[update_count_ & 1]);
    update_count_ = update_count_ + 1;

    readings_.left_90      = filtered_[0];
    readings_.left_45      = filtered_[1];
    readings_.front_left   = filtered_[2];
    readings_.front_right  = filtered_[3];
    readings_.right_45     = filtered_[4];
    readings_.right_90     = filtered_[5];
    readings_.front_center = (readings_.front_left > readings_.front_right) ? readings_.front_left : readings_.front_right;

    // Pillar / post edge detection (falling edge when passing a wall opening)
    readings_.post_edge_left  = (prev_l90_ > thresh_l90_) && (readings_.left_90 < thresh_l90_) && ((int32_t)prev_l90_ - readings_.left_90 > 60);
    readings_.post_edge_right = (prev_r90_ > thresh_r90_) && (readings_.right_90 < thresh_r90_) && ((int32_t)prev_r90_ - readings_.right_90 > 60);

    // Pillar / post rising edge (entering a wall from an opening)
    readings_.post_rising_left  = (prev_l90_ < thresh_l90_) && (readings_.left_90 > thresh_l90_) && ((int32_t)readings_.left_90 - prev_l90_ > 60);
    readings_.post_rising_right = (prev_r90_ < thresh_r90_) && (readings_.right_90 > thresh_r90_) && ((int32_t)readings_.right_90 - prev_r90_ > 60);

    prev_l90_ = readings_.left_90;
    prev_r90_ = readings_.right_90;

    // 90° sensors directly inspect current cell walls
    readings_.wall_left  = (readings_.left_90 > thresh_l90_);
    readings_.wall_right = (readings_.right_90 > thresh_r90_);
    readings_.wall_front = (readings_.front_center > thresh_front_);

    // Opening anticipation: 90° sensor detects wall present beside the robot,
    // but 45° lookahead sensor drops significantly below nominal, indicating the wall terminates ahead.
    const uint16_t opening_cutoff_l = (uint16_t)(nominal_center_l45_ * 0.65f);
    const uint16_t opening_cutoff_r = (uint16_t)(nominal_center_r45_ * 0.65f);

    readings_.opening_left  = readings_.wall_left  && (readings_.left_45  < opening_cutoff_l);
    readings_.opening_right = readings_.wall_right && (readings_.right_45 < opening_cutoff_r);

    // Determine reliable wall guides for steering
    bool valid_left_guide  = readings_.wall_left  && !readings_.opening_left  && (readings_.left_45  > thresh_l45_);
    bool valid_right_guide = readings_.wall_right && !readings_.opening_right && (readings_.right_45 > thresh_r45_);
    bool both_diagonals_funnel = (readings_.left_45 > thresh_l45_) && (readings_.right_45 > thresh_r45_);

    // Centering error calculation with opening anticipation & dynamic front-wall approach squaring
    if (readings_.wall_front) {
        // Approaching a front wall: dynamically null angular tilt using FL vs FR disparity
        float fl = (float)readings_.front_left;
        float fr = (float)readings_.front_right;
        float avg_f = (fl + fr) * 0.5f;
        if (avg_f > 100.0f) {
            // FL > FR: left side closer to front wall -> tilted CCW -> steer CW (positive error)
            readings_.centering_error = 1.5f * ((fl - fr) / avg_f);
        } else {
            readings_.centering_error = 0.0f;
        }
    } else if ((valid_left_guide && valid_right_guide) ||
               (both_diagonals_funnel && !readings_.wall_left && !readings_.wall_right)) {
        // Both walls present (or re-entering a corridor with walls ahead on both sides):
        // steer toward the side whose wall is further away
        float dist_left  = relativeDistance(readings_.left_45,  nominal_center_l45_);
        float dist_right = relativeDistance(readings_.right_45, nominal_center_r45_);
        readings_.centering_error = 2.0f * (dist_right - dist_left);
    } else if (valid_left_guide) {
        // Right wall is opening or absent -> hold the centred distance from the left wall only
        readings_.centering_error = 4.0f * (1.0f - relativeDistance(readings_.left_45, nominal_center_l45_));
    } else if (valid_right_guide) {
        // Left wall is opening or absent -> hold the centred distance from the right wall only
        readings_.centering_error = 4.0f * (relativeDistance(readings_.right_45, nominal_center_r45_) - 1.0f);
    } else {
        // Both walls open / open intersection -> maintain heading via IMU, zero steering bias
        readings_.centering_error = 0.0f;
    }

    // Clamp centering error to prevent extreme spikes from saturating actuators
    if (readings_.centering_error > 1.5f)  readings_.centering_error = 1.5f;
    if (readings_.centering_error < -1.5f) readings_.centering_error = -1.5f;
}

bool IRSensors::calibrateInCell(uint16_t sample_count) {
    Serial.println("\n[CALIB] Starting In-Cell IR Auto-Calibration...");
    Serial.println("[CALIB] Ensure the mouse is placed squarely in the center of a cell with Left, Right, and Front walls!");

    uint32_t sum_l90 = 0, sum_l45 = 0, sum_fl = 0, sum_fr = 0, sum_r45 = 0, sum_r90 = 0;

    for (uint16_t i = 0; i < sample_count; ++i) {
        // The 500 Hz control loop owns the emitters; just average what it produces. If it is not
        // running (bench tests), sample both groups here instead.
        uint32_t seen = update_count_;
        delay(5);
        if (update_count_ == seen) {
            update();
            update();
        }
        sum_l90 += filtered_[0];
        sum_l45 += filtered_[1];
        sum_fl  += filtered_[2];
        sum_fr  += filtered_[3];
        sum_r45 += filtered_[4];
        sum_r90 += filtered_[5];
    }

    uint16_t avg_l90   = sum_l90 / sample_count;
    uint16_t avg_l45   = sum_l45 / sample_count;
    uint16_t avg_fl    = sum_fl  / sample_count;
    uint16_t avg_fr    = sum_fr  / sample_count;
    uint16_t avg_front = (avg_fl > avg_fr) ? avg_fl : avg_fr;
    uint16_t avg_r45   = sum_r45 / sample_count;
    uint16_t avg_r90   = sum_r90 / sample_count;

    Serial.printf("[CALIB] Measured Cell Averages: L90=%d, L45=%d, FL=%d, FR=%d, R45=%d, R90=%d\n",
                  avg_l90, avg_l45, avg_fl, avg_fr, avg_r45, avg_r90);

    // The 45° sensors point forward as well as sideways, so depending on where they are mounted
    // they may be looking at the side wall of the NEXT cell, which can have a gap in it. If one
    // of them sees nothing while the other does, borrow from the good side, scaled by how the two
    // 90° sensors compare (which both do see this cell's walls).
    const bool left_ok  = avg_l45 >= 100;
    const bool right_ok = avg_r45 >= 100;
    if (!left_ok && !right_ok) {
        Serial.println("[CALIB] ERROR: Side sensor readings too low! Ensure robot is centered between left/right walls.");
        return false;
    }
    if (!left_ok || !right_ok) {
        float left_to_right = (avg_l90 > 100 && avg_r90 > 100) ? (float)avg_r90 / (float)avg_l90 : 1.0f;
        if (!right_ok) avg_r45 = (uint16_t)((float)avg_l45 * left_to_right);
        if (!left_ok)  avg_l45 = (uint16_t)((float)avg_r45 / left_to_right);
        Serial.println("[CALIB] Note: one 45° sensor saw no wall (gap ahead on that side). Estimated it from the other side.");
    }

    // Set nominal center values for steering
    nominal_center_l45_ = avg_l45;
    nominal_center_r45_ = avg_r45;

    // Detection thresholds set to 40% of nominal wall reflection
    thresh_l90_ = (avg_l90 > 100) ? (uint16_t)(avg_l90 * 0.40f) : WALL_THRESH_L90;
    thresh_l45_ = (uint16_t)(avg_l45 * 0.40f);
    thresh_r45_ = (uint16_t)(avg_r45 * 0.40f);
    thresh_r90_ = (avg_r90 > 100) ? (uint16_t)(avg_r90 * 0.40f) : WALL_THRESH_R90;

    // If front wall is present, calibrate from measurement; otherwise estimate from side 45° sensors
    if (avg_front >= 100) {
        thresh_front_ = (uint16_t)(avg_front * 0.40f);
    } else {
        Serial.println("[CALIB] Note: Front wall absent (open start corridor). Estimating front threshold from side sensors.");
        thresh_front_ = (uint16_t)(((float)avg_l45 + (float)avg_r45) * 0.5f * 0.45f);
    }

    // Save permanently to NVS Flash
    saveToNVS();
    Serial.println("[CALIB] Calibration successfully saved to Flash NVS!\n");
    return true;
}

void IRSensors::saveToNVS() {
    prefs_.begin("ir_calib", false); // Namespace "ir_calib", read/write
    prefs_.putUShort("l90_t", thresh_l90_);
    prefs_.putUShort("l45_t", thresh_l45_);
    prefs_.putUShort("f_t",   thresh_front_);
    prefs_.putUShort("r45_t", thresh_r45_);
    prefs_.putUShort("r90_t", thresh_r90_);
    prefs_.putUShort("l45_c", nominal_center_l45_);
    prefs_.putUShort("r45_c", nominal_center_r45_);
    prefs_.putBool("valid", true);
    prefs_.end();
}

bool IRSensors::loadFromNVS() {
    prefs_.begin("ir_calib", true); // Read-only
    if (!prefs_.getBool("valid", false)) {
        prefs_.end();
        return false;
    }

    thresh_l90_         = prefs_.getUShort("l90_t", WALL_THRESH_L90);
    thresh_l45_         = prefs_.getUShort("l45_t", WALL_THRESH_L45);
    thresh_front_       = prefs_.getUShort("f_t",   WALL_THRESH_FRONT);
    thresh_r45_         = prefs_.getUShort("r45_t", WALL_THRESH_R45);
    thresh_r90_         = prefs_.getUShort("r90_t", WALL_THRESH_R90);
    nominal_center_l45_ = prefs_.getUShort("l45_c", NOMINAL_CENTER_L45);
    nominal_center_r45_ = prefs_.getUShort("r45_c", NOMINAL_CENTER_R45);
    prefs_.end();

    Serial.printf("[IR] Loaded from Flash: CenterL45=%d, CenterR45=%d, FrontThresh=%d\n",
                  nominal_center_l45_, nominal_center_r45_, thresh_front_);
    return true;
}

IRReadings IRSensors::getReadings() const {
    return readings_;
}

bool IRSensors::hasLeftWall() const {
    return readings_.wall_left;
}

bool IRSensors::hasRightWall() const {
    return readings_.wall_right;
}

bool IRSensors::hasFrontWall() const {
    return readings_.wall_front;
}

bool IRSensors::hasLeftPostEdge() const {
    return readings_.post_edge_left;
}

bool IRSensors::hasRightPostEdge() const {
    return readings_.post_edge_right;
}

bool IRSensors::hasLeftPostRising() const {
    return readings_.post_rising_left;
}

bool IRSensors::hasRightPostRising() const {
    return readings_.post_rising_right;
}

bool IRSensors::hasLeftOpening() const {
    return readings_.opening_left;
}

bool IRSensors::hasRightOpening() const {
    return readings_.opening_right;
}

float IRSensors::getCenteringError() const {
    return readings_.centering_error;
}

float IRSensors::getDiagonalGuardError() const {
    // Each 45° sensor starts to complain once it reads DIAG_GUARD_RATIO times its normal
    // "centred in a corridor" level, and is fully alarmed at twice that
    float guard_l = (float)nominal_center_l45_ * DIAG_GUARD_RATIO;
    float guard_r = (float)nominal_center_r45_ * DIAG_GUARD_RATIO;
    float excess_l = ((float)readings_.left_45  - guard_l) / guard_l;
    float excess_r = ((float)readings_.right_45 - guard_r) / guard_r;
    if (excess_l < 0.0f) excess_l = 0.0f;
    if (excess_r < 0.0f) excess_r = 0.0f;
    return constrain(excess_l - excess_r, -1.0f, 1.0f);
}

void IRSensors::setThresholds(uint16_t thresh_l90, uint16_t thresh_l45,
                              uint16_t thresh_front,
                              uint16_t thresh_r45, uint16_t thresh_r90) {
    thresh_l90_   = thresh_l90;
    thresh_l45_   = thresh_l45;
    thresh_front_ = thresh_front;
    thresh_r45_   = thresh_r45;
    thresh_r90_   = thresh_r90;
}

void IRSensors::setNominalCenters(uint16_t center_l45, uint16_t center_r45) {
    nominal_center_l45_ = center_l45;
    nominal_center_r45_ = center_r45;
}

// ==============================================================================
// IMU
// ==============================================================================

// BNO055 register map
#define BNO055_CHIP_ID_ADDR        0x00
#define BNO055_PAGE_ID_ADDR        0x07
#define BNO055_GYRO_DATA_Z_LSB     0x18
#define BNO055_OPR_MODE_ADDR       0x3D
#define BNO055_PWR_MODE_ADDR       0x3E

#define BNO055_CHIP_ID             0xA0
#define BNO055_ALT_I2C_ADDR        0x29   // Address when the module's ADR pin is pulled high

#define OPERATION_MODE_CONFIG      0x00
#define OPERATION_MODE_IMUPLUS     0x08   // Gyro + accelerometer fusion (no magnetometer)

// The BNO055 is read every 5th control tick (100 Hz, its fusion output rate)
#define IMU_POLL_DIVIDER           5
#define IMU_MODE_CHECK_DIVIDER     500    // Verify fusion mode once per second
#define IMU_SETTLE_READS           5      // Reads discarded after re-entering fusion mode
#define IMU_FROZEN_READS           10     // Unchanged readings during a turn that trigger a mode check

IMU::IMU()
    : address_(BNO055_I2C_ADDR),
      hardware_detected_(false),
      heading_offset_deg_(0.0f),
      last_bno_heading_deg_(0.0f),
      heading_rate_deg_s_(0.0f),
      time_since_good_s_(0.0f),
      gyro_bias_z_(0.0f),
      gyro_sign_(1),
      gyro_agreement_(0),
      poll_counter_(0),
      bad_reads_(0),
      settle_reads_(0),
      frozen_reads_(0),
      reanchor_pending_(false),
      fault_count_(0) {
    memset(&state_, 0, sizeof(state_));
}

void IMU::begin() {
    // The BNO055 needs ~650 ms after power-up before it answers, so retry on both possible addresses
    hardware_detected_ = false;
    for (int attempt = 0; attempt < 4 && !hardware_detected_; ++attempt) {
        if (attempt > 0) delay(200);
        hardware_detected_ = initBNO055(BNO055_I2C_ADDR) || initBNO055(BNO055_ALT_I2C_ADDR);
    }

    if (hardware_detected_) {
        Serial.printf("[IMU] Bosch BNO055 initialized in IMU/Fusion mode at 0x%02X.\n", address_);

        // Measure static gyro bias while mouse is stationary
        float sum_gz = 0.0f;
        int valid_samples = 0;
        for (int i = 0; i < 50; ++i) {
            float h = 0.0f, gz = 0.0f;
            if (readBNO055Data(h, gz)) {
                sum_gz += gz;
                last_bno_heading_deg_ = h;
                valid_samples++;
            }
            delay(5);
        }
        if (valid_samples > 0) {
            gyro_bias_z_ = sum_gz / (float)valid_samples;
            Serial.printf("[IMU] Calibrated Z-Gyro Static Bias: %5.3f deg/s\n", gyro_bias_z_);
        }
    } else {
        Serial.println("[IMU] BNO055 not detected on I2C. Falling back to differential encoder odometry.");
    }

    state_.is_calibrated = hardware_detected_;
    resetHeading(0.0f);
}

bool IMU::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool IMU::readRegister(uint8_t reg, uint8_t& value) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;

    Wire.requestFrom(address_, (uint8_t)1);
    if (!Wire.available()) return false;
    value = Wire.read();
    return true;
}

bool IMU::initBNO055(uint8_t address) {
    address_ = address;

    // 1. Check chip ID
    uint8_t id = 0;
    if (!readRegister(BNO055_CHIP_ID_ADDR, id) || id != BNO055_CHIP_ID) return false;

    // 2. Config mode
    writeRegister(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    delay(25);

    // 3. Normal power mode
    writeRegister(BNO055_PWR_MODE_ADDR, 0x00);
    delay(10);

    // 4. Page 0
    writeRegister(BNO055_PAGE_ID_ADDR, 0x00);

    // 5. IMUPLUS mode (gyro + accelerometer fusion)
    bool ok = writeRegister(BNO055_OPR_MODE_ADDR, OPERATION_MODE_IMUPLUS);
    delay(20);
    return ok;
}

bool IMU::readBNO055Data(float& heading_deg, float& gyro_z) {
    // Burst read 4 bytes beginning at 0x18:
    //   0x18/0x19 = Gyro Z LSB/MSB, 0x1A/0x1B = Euler Heading LSB/MSB
    Wire.beginTransmission(address_);
    Wire.write(BNO055_GYRO_DATA_Z_LSB);
    if (Wire.endTransmission() != 0) return false;

    Wire.requestFrom(address_, (uint8_t)4);
    if (Wire.available() < 4) return false;

    uint8_t gz_lsb = Wire.read();
    uint8_t gz_msb = Wire.read();
    uint8_t h_lsb  = Wire.read();
    uint8_t h_msb  = Wire.read();

    int16_t raw_gz = (int16_t)(((uint16_t)gz_msb << 8) | gz_lsb);
    int16_t raw_h  = (int16_t)(((uint16_t)h_msb << 8) | h_lsb);

    // Euler heading is 16 LSB per degree in [0, 360); anything else is a corrupted transfer
    if (raw_h < 0 || raw_h > 360 * 16) return false;

    // Gyroscope: 16 LSB = 1 degree/second
    gyro_z = (float)raw_gz / 16.0f;

    // Native BNO055 heading increases clockwise. Convert to the robot convention:
    // LEFT / CCW = positive, RIGHT / CW = negative.
    float raw_deg = (float)raw_h / 16.0f;
    heading_deg = (raw_deg == 0.0f) ? 0.0f : (360.0f - raw_deg);
    return true;
}

void IMU::update(float dt_seconds, float encoder_yaw_rate, float linear_speed_mm_s) {
    if (dt_seconds <= 0.0f) {
        dt_seconds = CONTROL_DT_S;
    }

    poll_counter_++;
    time_since_good_s_ += dt_seconds;

    if (hardware_detected_ && (poll_counter_ % IMU_POLL_DIVIDER) == 0) {
        float raw_h = 0.0f;
        float raw_gz = 0.0f;
        bool ok = readBNO055Data(raw_h, raw_gz);

        // A brownout drops the BNO055 back into config mode, where it answers but its heading
        // freezes. Check once per second, or right away if the heading has not moved at all while
        // the wheels say we are turning, and put it back into fusion mode if that happened.
        if (ok && fabsf(encoder_yaw_rate) > 60.0f && raw_h == last_bno_heading_deg_) {
            frozen_reads_++;
        } else {
            frozen_reads_ = 0;
        }
        if (ok && ((poll_counter_ % IMU_MODE_CHECK_DIVIDER) == 0 || frozen_reads_ >= IMU_FROZEN_READS)) {
            frozen_reads_ = 0;
            uint8_t mode = 0;
            if (readRegister(BNO055_OPR_MODE_ADDR, mode) && (mode & 0x0F) != OPERATION_MODE_IMUPLUS) {
                writeRegister(BNO055_OPR_MODE_ADDR, OPERATION_MODE_IMUPLUS);
                settle_reads_ = IMU_SETTLE_READS;
                bad_reads_ = IMU_FAULT_READS; // Run on encoder odometry until fusion is back
                reanchor_pending_ = true;     // Its heading restarted from an unrelated reference
                fault_count_++;
            }
        }
        if (ok && settle_reads_ > 0) {
            settle_reads_--;
            ok = false;
        }

        if (ok) {
            // Compare against the heading we have been tracking since the last good read
            float jump = normalizeAngle180(raw_h - heading_offset_deg_ - state_.heading_deg);
            bool rate_valid = true;
            if (reanchor_pending_ || fabsf(jump) > IMU_MAX_STEP_DEG) {
                if (!reanchor_pending_ && bad_reads_ < IMU_FAULT_READS) {
                    ok = false; // Physically impossible step: reject as a glitch
                } else {
                    // Sensor came back with a different reference after an outage:
                    // keep our tracked heading and re-anchor the BNO055 to it
                    heading_offset_deg_ = normalizeAngle180(raw_h - state_.heading_deg);
                    reanchor_pending_ = false;
                    rate_valid = false;
                }
            }

            if (ok) {
                acceptReading(raw_h, raw_gz, rate_valid, encoder_yaw_rate, linear_speed_mm_s);
                return;
            }
        }

        if (bad_reads_ < IMU_FAULT_READS) {
            bad_reads_++;
            if (bad_reads_ == IMU_FAULT_READS) fault_count_++;
        }
    }

    // Between BNO055 reads: extrapolate with the rate measured from its last two headings so the
    // heading is smooth at 500 Hz. If the sensor is missing or a read is overdue, dead-reckon on
    // differential encoder odometry instead.
    bool imu_fresh = hardware_detected_ && bad_reads_ == 0 &&
                     time_since_good_s_ < (IMU_POLL_DIVIDER * CONTROL_DT_S * 1.5f);
    float rate = imu_fresh ? heading_rate_deg_s_ : encoder_yaw_rate;

    if (isUsingFallback()) {
        state_.gyro_z_deg_s = encoder_yaw_rate;
    }

    state_.heading_deg = normalizeAngle180(state_.heading_deg + rate * dt_seconds);
    state_.heading_rad = state_.heading_deg * (PI / 180.0f);
}

void IMU::acceptReading(float raw_h, float raw_gz, bool rate_valid,
                        float encoder_yaw_rate, float linear_speed_mm_s) {
    // Heading rate from two consecutive BNO055 headings (always sign-consistent with the heading)
    if (rate_valid && bad_reads_ == 0 && time_since_good_s_ > 0.0f && time_since_good_s_ < 0.05f) {
        heading_rate_deg_s_ = normalizeAngle180(raw_h - last_bno_heading_deg_) / time_since_good_s_;
    } else {
        heading_rate_deg_s_ = 0.0f;
    }

    // Update gyro bias while stationary
    bool is_stationary = (fabsf(linear_speed_mm_s) < 8.0f) && (fabsf(encoder_yaw_rate) < 1.0f);
    if (is_stationary) {
        gyro_bias_z_ = 0.98f * gyro_bias_z_ + 0.02f * raw_gz;
    }
    float gyro = raw_gz - gyro_bias_z_;

    // Cross-check the gyro axis polarity against the heading during real turns. If the module is
    // mounted so they disagree, flip the gyro so yaw damping can never become positive feedback.
    if (fabsf(heading_rate_deg_s_) > 45.0f && fabsf(gyro) > 45.0f) {
        bool agree = (heading_rate_deg_s_ > 0.0f) == (gyro > 0.0f);
        gyro_agreement_ = constrain(gyro_agreement_ + (agree ? 1 : -1), -50, 50);
        if (gyro_agreement_ <= -20) gyro_sign_ = -1;
        if (gyro_agreement_ >= 20)  gyro_sign_ = 1;
    }

    state_.gyro_z_deg_s = (float)gyro_sign_ * gyro;

    // Use the BNO055 fused heading directly, relative to where resetHeading() was last called
    state_.heading_deg = normalizeAngle180(raw_h - heading_offset_deg_);
    state_.heading_rad = state_.heading_deg * (PI / 180.0f);

    last_bno_heading_deg_ = raw_h;
    time_since_good_s_ = 0.0f;
    bad_reads_ = 0;
}

IMUState IMU::getState() const {
    return state_;
}

float IMU::getHeadingDeg() const {
    return state_.heading_deg;
}

float IMU::getGyroZ() const {
    return state_.gyro_z_deg_s;
}

void IMU::resetHeading(float initial_heading_deg) {
    if (hardware_detected_) {
        // Store the BNO055's current physical direction as the new reference. If this read fails,
        // fall back to where the tracked heading says the BNO055 currently is.
        float bno_now = state_.heading_deg + heading_offset_deg_;
        float current_raw = 0.0f, gz = 0.0f;
        if (settle_reads_ == 0 && readBNO055Data(current_raw, gz)) {
            bno_now = current_raw;
            last_bno_heading_deg_ = current_raw;
            time_since_good_s_ = 0.0f;
        }
        heading_offset_deg_ = normalizeAngle180(bno_now - initial_heading_deg);
    }

    state_.heading_deg = initial_heading_deg;
    state_.heading_rad = initial_heading_deg * (PI / 180.0f);
    state_.gyro_z_deg_s = 0.0f;
}

bool IMU::isHardwareConnected() const {
    return hardware_detected_;
}

bool IMU::isUsingFallback() const {
    return !hardware_detected_ || bad_reads_ >= IMU_FAULT_READS;
}
