#include "motors.h"

MotorController motors;

MotorController::MotorController()
    : motoron_(MOTORON_I2C_ADDR),
      is_initialized_(false),
      last_command_time_(0),
      prev_left_cmd_(0),
      prev_right_cmd_(0),
      left_kick_end_ms_(0),
      right_kick_end_ms_(0),
      left_speed_compensation_(MOTOR_LEFT_COMPENSATION),
      right_speed_compensation_(MOTOR_RIGHT_COMPENSATION) {
}

bool MotorController::begin() {
    Wire.begin();
    Wire.setClock(I2C_BUS_SPEED);

    // Reinitialize Motoron driver
    motoron_.reinitialize();
    motoron_.disableCrc();
    motoron_.clearResetFlag();
    motoron_.disableCommandTimeout();

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
    // Disable Motoron internal acceleration limits (0 = immediate) so software PID has direct control
    motoron_.setMaxAcceleration(MOTOR_LEFT_CHANNEL, 0);
    motoron_.setMaxDeceleration(MOTOR_LEFT_CHANNEL, 0);
    motoron_.setMaxAcceleration(MOTOR_RIGHT_CHANNEL, 0);
    motoron_.setMaxDeceleration(MOTOR_RIGHT_CHANNEL, 0);

    // Clear any pending error flags
    motoron_.clearMotorFault();

    is_initialized_ = true;
    last_command_time_ = millis();
    stop(true);
    return true;
}

int16_t MotorController::processChannelCommand(int16_t command, float compensation,
                                               int16_t& prev_cmd, uint32_t& kick_end_ms,
                                               int16_t startup_offset, int16_t max_kick)
{
    if (command == 0) {
        prev_cmd = 0;
        kick_end_ms = 0;
        return 0;
    }

    uint32_t now = millis();
    // Only trigger breakaway kickstart when starting from a complete standstill.
    // Crucial: NEVER retrigger on direction reversals during active closed-loop control,
    // which would cause violent limit-cycle oscillations.
    bool starting_from_stop = (prev_cmd == 0);
    if (starting_from_stop) {
        kick_end_ms = now + MOTOR_KICKSTART_DURATION_MS;
    }
    prev_cmd = command;

    int16_t sign = (command > 0) ? 1 : -1;
    int16_t magnitude = abs(command);

    // Apply channel scaling / compensation
    magnitude = (int16_t)(magnitude * MOTOR_COMMAND_SCALE * compensation);

    int16_t kick_pwm = (int16_t)((MOTOR_KICKSTART_PWM + startup_offset) * compensation);
    if (kick_pwm > max_kick) {
        kick_pwm = max_kick;
    }

    if (now < kick_end_ms) {
        if (magnitude < kick_pwm) {
            magnitude = kick_pwm;
        }
    }

    magnitude = constrain(magnitude, 0, MOTOR_MAX_SPEED);
    return sign * magnitude;
}

bool MotorController::calibrateWheelSpeedBalance(float left_ticks_per_second,
                                                 float right_ticks_per_second) {
    if (left_ticks_per_second <= 0.0f || right_ticks_per_second <= 0.0f) {
        return false;
    }

    left_speed_compensation_ = MOTOR_LEFT_COMPENSATION;
    right_speed_compensation_ = MOTOR_RIGHT_COMPENSATION;

    if (left_ticks_per_second < right_ticks_per_second) {
        const float ratio = right_ticks_per_second / left_ticks_per_second;
        left_speed_compensation_ *= fminf(ratio, MOTOR_BALANCE_MAX_COMPENSATION);
    } else if (right_ticks_per_second < left_ticks_per_second) {
        const float ratio = left_ticks_per_second / right_ticks_per_second;
        right_speed_compensation_ *= fminf(ratio, MOTOR_BALANCE_MAX_COMPENSATION);
    }

    Serial.printf("[MOTORS] Runtime speed balance: left=%.3fx right=%.3fx\n",
                  left_speed_compensation_,
                  right_speed_compensation_);
    return true;
}

void MotorController::setSpeeds(int16_t left_speed, int16_t right_speed) {
    if (!is_initialized_) return;

    // Detect if this is an in-place turn (differential counter-rotation)
    bool is_turn = (left_speed > 0 && right_speed < 0) || (left_speed < 0 && right_speed > 0);
    int16_t max_kick = is_turn ? (int16_t)TURN_MAX_MOTOR_COMMAND : (int16_t)MOTOR_KICKSTART_PWM;

    int16_t processed_left = processChannelCommand(left_speed, left_speed_compensation_,
                                                   prev_left_cmd_, left_kick_end_ms_, 0, max_kick);
    int16_t processed_right = processChannelCommand(right_speed, right_speed_compensation_,
                                                    prev_right_cmd_, right_kick_end_ms_,
                                                    MOTOR_RIGHT_STARTUP_OFFSET, max_kick);

    // Send both speeds in a single efficient I2C packet
    motoron_.setAllSpeedsNow(MOTOR_LEFT_DIRECTION * processed_left,
                             MOTOR_RIGHT_DIRECTION * processed_right);

    last_command_time_ = millis();
}

void MotorController::setLeftSpeed(int16_t speed) {
    if (!is_initialized_) return;
    int16_t processed = processChannelCommand(speed, left_speed_compensation_,
                                             prev_left_cmd_, left_kick_end_ms_, 0);
    motoron_.setSpeedNow(MOTOR_LEFT_CHANNEL, MOTOR_LEFT_DIRECTION * processed);
}

void MotorController::setRightSpeed(int16_t speed) {
    if (!is_initialized_) return;
    int16_t processed = processChannelCommand(speed, right_speed_compensation_,
                                             prev_right_cmd_, right_kick_end_ms_,
                                             MOTOR_RIGHT_STARTUP_OFFSET);
    motoron_.setSpeedNow(MOTOR_RIGHT_CHANNEL, MOTOR_RIGHT_DIRECTION * processed);
}

void MotorController::stop(bool brake) {
    if (!is_initialized_) return;

    prev_left_cmd_ = 0;
    prev_right_cmd_ = 0;
    left_kick_end_ms_ = 0;
    right_kick_end_ms_ = 0;

    if (brake) {
        motoron_.setBraking(MOTOR_LEFT_CHANNEL, 100);
        motoron_.setBraking(MOTOR_RIGHT_CHANNEL, 100);
    } else {
        motoron_.setAllSpeedsNow(0, 0);
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
