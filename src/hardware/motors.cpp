#include "motors.h"
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
      last_cmd_time_ms_(0) {}

void Motors::begin() {
    // 1. Configure Hardware Reset and Boost Converter Enable pins
    pinMode(PIN_MOTOR_BAT_CTRL, OUTPUT);
    digitalWrite(PIN_MOTOR_BAT_CTRL, LOW); // Start with 12V boost disabled

    pinMode(PIN_MOTOR_RST, OUTPUT);
    digitalWrite(PIN_MOTOR_RST, HIGH);

    // 2. Enable 12V motor battery boost converter
    setMotorPowerEnabled(true);
    delay(50); // Allow 12V rail to stabilize

    // 3. Cycle Motoron hardware reset line
    digitalWrite(PIN_MOTOR_RST, LOW);
    delay(5);
    digitalWrite(PIN_MOTOR_RST, HIGH);
    delay(10);

    // 4. Initialize I2C communication with Motoron M2T256
    mc_.setAddress(MOTORON_I2C_ADDR);
    mc_.reinitialize();
    mc_.clearResetFlag();

    // Set Motoron internal acceleration to 0 (unlimited) so our 1kHz software PID
    // and trapezoidal trajectory generator have direct instantaneous control
    mc_.setMaxAcceleration(1, 0);
    mc_.setMaxDeceleration(1, 0);
    mc_.setMaxAcceleration(2, 0);
    mc_.setMaxDeceleration(2, 0);

    // 5. Load calibration trims from Flash NVS
    loadFromNVS();

    is_braking_ = false;
    last_left_cmd_ = -9999;
    last_right_cmd_ = -9999;
    last_cmd_time_ms_ = 0;

    coast();
}

void Motors::setMotorPowerEnabled(bool enabled) {
    power_enabled_ = enabled;
    // MotorBatControl drives NPN base HIGH, which pulls PMOS gate LOW and switches battery to 12V boost
    digitalWrite(PIN_MOTOR_BAT_CTRL, enabled ? HIGH : LOW);
}

void Motors::setEffort(float left_effort, float right_effort) {
    if (!power_enabled_) return;

    // Apply motor balance calibration trims
    left_effort *= trim_left_;
    right_effort *= trim_right_;

    if (invert_left_)  left_effort  = -left_effort;
    if (invert_right_) right_effort = -right_effort;

    // Clamp to range [-1.0, 1.0]
    if (left_effort > 1.0f) left_effort = 1.0f;
    if (left_effort < -1.0f) left_effort = -1.0f;
    if (right_effort > 1.0f) right_effort = 1.0f;
    if (right_effort < -1.0f) right_effort = -1.0f;

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
    // Only transmit if speed changed, transitioning from braking, or periodic 100ms keep-alive
    uint32_t now = millis();
    if (is_braking_ || left_cmd != last_left_cmd_ || right_cmd != last_right_cmd_ || (now - last_cmd_time_ms_ >= 100)) {
        mc_.setAllSpeedsNow(right_cmd, left_cmd);
        last_left_cmd_ = left_cmd;
        last_right_cmd_ = right_cmd;
        is_braking_ = false;
        last_cmd_time_ms_ = now;
    }
}

void Motors::setRawEffort(float left_effort, float right_effort) {
    if (!power_enabled_) return;

    // Direct effort bypassing trim multipliers (used for calibration benchmarking)
    if (invert_left_)  left_effort  = -left_effort;
    if (invert_right_) right_effort = -right_effort;

    if (left_effort > 1.0f) left_effort = 1.0f;
    if (left_effort < -1.0f) left_effort = -1.0f;
    if (right_effort > 1.0f) right_effort = 1.0f;
    if (right_effort < -1.0f) right_effort = -1.0f;

    int16_t left_cmd = 0;
    int16_t right_cmd = 0;

    if (fabsf(left_effort) >= deadband_left_) {
        left_cmd = (int16_t)(left_effort * (float)MOTORON_MAX_SPEED);
    }
    if (fabsf(right_effort) >= deadband_right_) {
        right_cmd = (int16_t)(right_effort * (float)MOTORON_MAX_SPEED);
    }

    uint32_t now = millis();
    if (is_braking_ || left_cmd != last_left_cmd_ || right_cmd != last_right_cmd_ || (now - last_cmd_time_ms_ >= 100)) {
        mc_.setAllSpeedsNow(right_cmd, left_cmd);
        last_left_cmd_ = left_cmd;
        last_right_cmd_ = right_cmd;
        is_braking_ = false;
        last_cmd_time_ms_ = now;
    }
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
    if (is_braking_) return; // Filter redundant brake calls at 500 Hz
    mc_.setBrakingNow(1, MOTORON_MAX_SPEED);
    mc_.setBrakingNow(2, MOTORON_MAX_SPEED);
    is_braking_ = true;
    last_left_cmd_ = -9999;
    last_right_cmd_ = -9999;
    last_cmd_time_ms_ = millis();
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

