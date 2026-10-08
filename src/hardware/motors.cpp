#include "motors.h"

MotorController motors;

static int16_t scaledMotorCommand(int16_t command, float motor_compensation) {
    command = constrain(command, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    float scaled = command * MOTOR_COMMAND_SCALE * motor_compensation;
    return (int16_t)constrain(scaled, (float)-MOTOR_MAX_SPEED, (float)MOTOR_MAX_SPEED);
}

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

    left_speed = scaledMotorCommand(left_speed, 1.0f);
    right_speed = scaledMotorCommand(right_speed, MOTOR_RIGHT_COMPENSATION);

    motoron_.setSpeed(MOTOR_LEFT_CHANNEL, MOTOR_LEFT_DIRECTION * left_speed);
    motoron_.setSpeed(MOTOR_RIGHT_CHANNEL, MOTOR_RIGHT_DIRECTION * right_speed);

    last_command_time_ = millis();
}

void MotorController::setLeftSpeed(int16_t speed) {
    if (!is_initialized_) return;
    speed = scaledMotorCommand(speed, 1.0f);
    motoron_.setSpeed(MOTOR_LEFT_CHANNEL, MOTOR_LEFT_DIRECTION * speed);
}

void MotorController::setRightSpeed(int16_t speed) {
    if (!is_initialized_) return;
    speed = scaledMotorCommand(speed, MOTOR_RIGHT_COMPENSATION);
    motoron_.setSpeed(MOTOR_RIGHT_CHANNEL, MOTOR_RIGHT_DIRECTION * speed);
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

uint16_t MotorController::getStatusFlags() {
    if (!is_initialized_) return 0;
    return motoron_.getStatusFlags();
}

int16_t MotorController::getLeftCurrentSpeed() {
    if (!is_initialized_) return 0;
    return motoron_.getCurrentSpeed(MOTOR_LEFT_CHANNEL);
}

int16_t MotorController::getRightCurrentSpeed() {
    if (!is_initialized_) return 0;
    return motoron_.getCurrentSpeed(MOTOR_RIGHT_CHANNEL);
}

void MotorController::clearErrors() {
    if (!is_initialized_) return;
    motoron_.clearResetFlag();
    motoron_.clearMotorFault();
}
