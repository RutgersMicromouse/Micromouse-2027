#include "motion_controller.h"

MotionController motion;

MotionController::MotionController()
    : target_heading_deg_(0.0f),
      start_distance_mm_(0.0f),
      centering_enabled_(true),
      last_tick_micros_(0) {
    // Linear Velocity PID: (Kp, Ki, Kd, Kf, min, max)
    // Motoron takes speed commands up to ±800
    pid_linear_vel_.setGains(1.2f, 0.08f, 0.02f, 0.75f, -MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);

    // Angular Heading PID: Translates heading error (degrees) into differential motor speed
    pid_angular_heading_.setGains(4.5f, 0.02f, 0.25f, 0.0f, -MOTOR_MAX_SPEED * 0.7f, MOTOR_MAX_SPEED * 0.7f);
}

bool MotionController::begin() {
    Serial.println("[MOTION] Initializing Subsystems...");

    // Initialize I2C Bus & Devices
    Wire.begin();
    Wire.setClock(I2C_BUS_SPEED);

    // 1. Motors
    if (!motors.begin()) {
        Serial.println("[MOTION] WARNING: Motor driver failed to initialize!");
    }

    // 2. Encoders
    encoders.begin();

    // 3. IMU
    imu.begin();

    // 4. IR Distance Sensors
    ir_sensors.begin();

    // 5. Battery Monitor
    battery.begin();

    target_heading_deg_ = imu.getHeadingDeg();
    last_tick_micros_ = micros();
    Serial.println("[MOTION] Motion Controller Ready!");
    return true;
}

void MotionController::setTargetHeading(float deg) {
    target_heading_deg_ = deg;
    while (target_heading_deg_ > 180.0f)  target_heading_deg_ -= 360.0f;
    while (target_heading_deg_ <= -180.0f) target_heading_deg_ += 360.0f;
}

void MotionController::update(float dt_seconds) {
    if (dt_seconds <= 0.0f) {
        dt_seconds = CONTROL_DT_S;
    }

    // 1. Update Hardware Readings
    encoders.update(dt_seconds);
    imu.update(dt_seconds, encoders.getEncoderYawRateDeg_S(), encoders.getForwardSpeedMM_S());
    ir_sensors.update();
    battery.update();

    // Failsafe: Battery Protection
    if (battery.isCritical()) {
        motors.stop(false);
        return;
    }

    // 2. Update Motion Profiles
    linear_profile_.update(dt_seconds);
    angular_profile_.update(dt_seconds);

    // 3. Linear Speed Control
    float desired_vel = linear_profile_.getCurrentVelocity();
    float current_vel = encoders.getForwardSpeedMM_S();
    float linear_cmd  = pid_linear_vel_.update(desired_vel, current_vel, dt_seconds);

    // 4. Angular Heading & Wall Centering Control
    float heading_error = target_heading_deg_ - imu.getHeadingDeg();
    while (heading_error > 180.0f)  heading_error -= 360.0f;
    while (heading_error <= -180.0f) heading_error += 360.0f;

    // Inject wall centering error when moving forward and centering is active
    if (centering_enabled_ && fabsf(desired_vel) > 30.0f && !ir_sensors.hasFrontWall()) {
        float centering_bias_deg = ir_sensors.getCenteringError() * -12.0f; // Up to 12° steering injection
        heading_error += centering_bias_deg;
    }

    float angular_cmd = pid_angular_heading_.updateError(heading_error, dt_seconds);

    // 5. Differential Motor Mixing
    int16_t left_motor_pwm  = (int16_t)(linear_cmd - angular_cmd);
    int16_t right_motor_pwm = (int16_t)(linear_cmd + angular_cmd);

    // If profile is finished and zero velocity requested, stop active driving
    if (linear_profile_.isFinished() && angular_profile_.isFinished() && fabsf(desired_vel) < 1.0f) {
        motors.stop(true);
    } else {
        motors.setSpeeds(left_motor_pwm, right_motor_pwm);
    }
}

bool MotionController::moveForward(float distance_mm, float max_speed, float end_speed, bool allow_centering) {
    centering_enabled_ = allow_centering;
    encoders.resetDistance();
    pid_linear_vel_.reset();

    float start_speed = encoders.getForwardSpeedMM_S();
    linear_profile_.start(distance_mm, start_speed, end_speed, max_speed, SEARCH_ACCEL_MM_S2, DECEL_MM_S2);

    uint32_t start_time = millis();
    // Maximum timeout based on distance + margin
    uint32_t timeout_ms = (uint32_t)((fabsf(distance_mm) / (max_speed * 0.4f)) * 1000.0f) + 1200;

    while (!linear_profile_.isFinished()) {
        uint32_t now = micros();
        float dt = (now - last_tick_micros_) * 1e-6f;
        if (dt >= CONTROL_DT_S) {
            last_tick_micros_ = now;
            update(dt);
        }

        // Emergency front-wall collision avoidance check
        if (ir_sensors.hasFrontWall() && ir_sensors.getFront() > (IR_FRONT_STOP_DIST + 150)) {
            Serial.println("[MOTION] Early Front Wall Stop Triggered!");
            break;
        }

        if (millis() - start_time > timeout_ms) {
            Serial.println("[MOTION] moveForward Timeout!");
            break;
        }
    }

    if (end_speed <= 0.0f) {
        motors.stop(true);
        delay(20); // Settle
    }
    return true;
}

bool MotionController::moveForwardCells(int num_cells, float max_speed, float end_speed, bool allow_centering) {
    float distance = (float)num_cells * CELL_DIMENSION_MM;
    return moveForward(distance, max_speed, end_speed, allow_centering);
}

bool MotionController::turnInPlace(float angle_deg, float turn_speed) {
    centering_enabled_ = false;
    motors.stop(true);
    delay(30);

    float new_target = target_heading_deg_ + angle_deg;
    setTargetHeading(new_target);

    pid_angular_heading_.reset();

    uint32_t start_time = millis();
    uint32_t timeout_ms = (uint32_t)((fabsf(angle_deg) / turn_speed) * 1000.0f) + 1000;

    while (millis() - start_time < timeout_ms) {
        uint32_t now = micros();
        float dt = (now - last_tick_micros_) * 1e-6f;
        if (dt >= CONTROL_DT_S) {
            last_tick_micros_ = now;

            // Enforce zero linear velocity during in-place turn
            linear_profile_.stopNow();
            update(dt);
        }

        float err = target_heading_deg_ - imu.getHeadingDeg();
        while (err > 180.0f)  err -= 360.0f;
        while (err <= -180.0f) err += 360.0f;

        // Settling condition: angle error < 1.0 deg and yaw rate < 5 dps
        if (fabsf(err) < 1.0f && fabsf(imu.getYawRateDeg_S()) < 5.0f && (millis() - start_time > 150)) {
            break;
        }
    }

    motors.stop(true);
    delay(30);
    return true;
}

bool MotionController::alignFrontWall(float approach_speed, uint16_t timeout_ms) {
    if (!ir_sensors.hasFrontWall()) return false;

    Serial.println("[MOTION] Aligning to Front Wall...");
    uint32_t start_time = millis();

    while (millis() - start_time < timeout_ms) {
        uint32_t now = micros();
        float dt = (now - last_tick_micros_) * 1e-6f;
        if (dt >= CONTROL_DT_S) {
            last_tick_micros_ = now;
            encoders.update(dt);
            imu.update(dt, encoders.getEncoderYawRateDeg_S(), encoders.getForwardSpeedMM_S());
            ir_sensors.update();

            // Drive slowly forward until front distance threshold reached
            if (ir_sensors.getFront() >= IR_FRONT_STOP_DIST) {
                break;
            }

            motors.setSpeeds((int16_t)approach_speed, (int16_t)approach_speed);
        }
    }

    motors.stop(true);
    delay(40);
    return true;
}

void MotionController::emergencyStop() {
    linear_profile_.stopNow();
    angular_profile_.stopNow();
    motors.stop(true);
}
