#include "motors.h"

MotorController motors;

MotorController::MotorController()
    : motoron_(MOTORON_I2C_ADDR),
      is_initialized_(false),
      last_command_time_(0) {
}

bool MotorController::begin() {
    Wire.begin();
    Wire.setClock(I2C_BUS_SPEED);

    // Reinitialize Motoron driver
    motoron_.reinitialize();
    motoron_.disableCrc();
    motoron_.clearResetFlag();

    // Verify communication by reading firmware version and product ID
    uint16_t product_id = 0;
    uint16_t firmware_version = 0;
    motoron_.getFirmwareVersion(&product_id, &firmware_version);
    if (product_id == 0 || product_id == 0xFFFF) {
        Serial.printf("[MOTORS] ERROR: Motoron M2T256 not detected at I2C address 0x%02X!\n", MOTORON_I2C_ADDR);
        is_initialized_ = false;
        return false;
    }

    Serial.printf("[MOTORS] Motoron M2T256 detected! Product ID: 0x%04X\n", product_id);

    // Configure motor settings:
    // Configure max acceleration & deceleration to ensure smooth current draw from 12V regulator
    motoron_.setMaxAcceleration(MOTOR_LEFT_CHANNEL, 400);
    motoron_.setMaxDeceleration(MOTOR_LEFT_CHANNEL, 400);
    motoron_.setMaxAcceleration(MOTOR_RIGHT_CHANNEL, 400);
    motoron_.setMaxDeceleration(MOTOR_RIGHT_CHANNEL, 400);

    // Clear any pending error flags
    motoron_.clearMotorFault();

    is_initialized_ = true;
    last_command_time_ = millis();
    stop(true);
    return true;
}

void MotorController::setSpeeds(int16_t left_speed, int16_t right_speed) {
    if (!is_initialized_) return;

    // Clamp speed limits
    left_speed  = constrain(left_speed,  -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    right_speed = constrain(right_speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

    // Right motor is mounted symmetrically opposing left motor, so invert right direction
    motoron_.setSpeed(MOTOR_LEFT_CHANNEL, left_speed);
    motoron_.setSpeed(MOTOR_RIGHT_CHANNEL, -right_speed);

    last_command_time_ = millis();
}

void MotorController::setLeftSpeed(int16_t speed) {
    if (!is_initialized_) return;
    speed = constrain(speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    motoron_.setSpeed(MOTOR_LEFT_CHANNEL, speed);
}

void MotorController::setRightSpeed(int16_t speed) {
    if (!is_initialized_) return;
    speed = constrain(speed, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    motoron_.setSpeed(MOTOR_RIGHT_CHANNEL, -speed);
}

void MotorController::stop(bool brake) {
    if (!is_initialized_) return;

    if (brake) {
        motoron_.setBraking(MOTOR_LEFT_CHANNEL, 100);
        motoron_.setBraking(MOTOR_RIGHT_CHANNEL, 100);
    } else {
        motoron_.setSpeed(MOTOR_LEFT_CHANNEL, 0);
        motoron_.setSpeed(MOTOR_RIGHT_CHANNEL, 0);
    }
}

bool MotorController::isConnected() {
    return is_initialized_;
}

void MotorController::clearErrors() {
    if (!is_initialized_) return;
    motoron_.clearResetFlag();
    motoron_.clearMotorFault();
}
