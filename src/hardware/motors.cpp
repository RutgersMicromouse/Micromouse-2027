#include "motors.h"

Motors::Motors()
    : invert_left_(false),
      invert_right_(false),
      power_enabled_(false) {}

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

    coast();
}

void Motors::setMotorPowerEnabled(bool enabled) {
    power_enabled_ = enabled;
    // MotorBatControl drives NPN base HIGH, which pulls PMOS gate LOW and switches battery to 12V boost
    digitalWrite(PIN_MOTOR_BAT_CTRL, enabled ? HIGH : LOW);
}

void Motors::setEffort(float left_effort, float right_effort) {
    if (!power_enabled_) return;

    if (invert_left_)  left_effort  = -left_effort;
    if (invert_right_) right_effort = -right_effort;

    // Clamp to range [-1.0, 1.0]
    if (left_effort > 1.0f) left_effort = 1.0f;
    if (left_effort < -1.0f) left_effort = -1.0f;
    if (right_effort > 1.0f) right_effort = 1.0f;
    if (right_effort < -1.0f) right_effort = -1.0f;

    // Deadband check
    const float deadband = 0.02f;
    int16_t left_cmd = 0;
    int16_t right_cmd = 0;

    if (fabsf(left_effort) >= deadband) {
        left_cmd = (int16_t)(left_effort * (float)MOTORON_MAX_SPEED);
    }
    if (fabsf(right_effort) >= deadband) {
        right_cmd = (int16_t)(right_effort * (float)MOTORON_MAX_SPEED);
    }

    // Schematic: M1 = Right Motor (R_MOTOR), M2 = Left Motor (L_MOTOR)
    // Send both motor speeds in a single I2C transaction to halve bus traffic at 500 Hz
    mc_.setAllSpeedsNow(right_cmd, left_cmd);
}

void Motors::brake() {
    mc_.setBrakingNow(1, MOTORON_MAX_SPEED);
    mc_.setBrakingNow(2, MOTORON_MAX_SPEED);
}

void Motors::coast() {
    mc_.setAllSpeedsNow(0, 0);
}

void Motors::setInverted(bool invert_left, bool invert_right) {
    invert_left_ = invert_left;
    invert_right_ = invert_right;
}
