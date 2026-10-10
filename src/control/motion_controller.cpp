#include "motion_controller.h"

MotionController motion;

MotionController::MotionController()
    : target_heading_deg_(0.0f),
      start_distance_mm_(0.0f),
      wall_alignment_gain_(0.5f),
      centering_enabled_(true),
      linear_motion_active_(false),
      search_motion_active_(false),
      linear_motion_start_ms_(0),
      last_tick_micros_(0)
{
    // Linear Velocity PID: (Kp, Ki, Kd, Kf)
    pid_linear_vel_.setGains(2.20f, 0.015f, 0.05f, 0.82f);
    pid_linear_vel_.setOutputLimits(-MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
    pid_linear_vel_.setIntegratorLimits(-20.0f, 20.0f);

    pid_angular_heading_.setGains(TURN_KP, TURN_KI, TURN_KD, 0.0f);
    pid_angular_heading_.setOutputLimits(-TURN_MAX_MOTOR_COMMAND, TURN_MAX_MOTOR_COMMAND);
    pid_angular_heading_.setIntegratorLimits(-50.0f, 50.0f);
}

bool MotionController::begin()
{
    Serial.println("[MOTION] Initializing Subsystems...");

    // Initialize I2C Bus & Devices
    Wire.begin();
    Wire.setClock(I2C_BUS_SPEED);

    // 1. Motors
    if (!motors.begin())
    {
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

void MotionController::setTargetHeading(float deg)
{
    target_heading_deg_ = deg;
    while (target_heading_deg_ > 180.0f)
        target_heading_deg_ -= 360.0f;
    while (target_heading_deg_ <= -180.0f)
        target_heading_deg_ += 360.0f;
}

void MotionController::update(float dt_seconds)
{
    if (dt_seconds <= 0.0f)
    {
        dt_seconds = CONTROL_DT_S;
    }

    // 1. Update Hardware Readings
    encoders.update(dt_seconds);
    imu.update(dt_seconds, encoders.getEncoderYawRateDeg_S(), encoders.getForwardSpeedMM_S());
    ir_sensors.update();
    battery.update();

#ifdef DEBUG_IMU_STREAM
    static uint32_t last_imu_debug_time = 0;
    if (millis() - last_imu_debug_time >= 250) {
        last_imu_debug_time = millis();
        IMUReading reading = imu.getReading();
        Serial.printf("[IMU DEBUG] connected=%s heading=%+.1fdeg yaw=%+.1fdps accelX=%+.3fg accelY=%+.3fg\n",
                      reading.is_ready ? "YES" : "NO",
                      reading.heading_deg,
                      reading.gyro_z_dps,
                      reading.accel_x_g,
                      reading.accel_y_g);
    }
#endif

    // 2. Update Motion Profiles
    if (linear_motion_active_) {
        linear_profile_.updateWithFeedback(dt_seconds, encoders.getAverageDistanceMM());
    } else {
        linear_profile_.update(dt_seconds);
    }
    angular_profile_.update(dt_seconds);

    // 3. Linear Speed Control
    float desired_vel = linear_profile_.getCurrentVelocity();
    float current_vel = encoders.getForwardSpeedMM_S();

    // Bleed off forward integrator windup during deceleration so it cannot resist stopping
    if (linear_profile_.isDecelerating() || (desired_vel < 25.0f && pid_linear_vel_.getIntegrator() > 0.0f)) {
        pid_linear_vel_.resetIntegrator();
    }

    float linear_cmd = pid_linear_vel_.update(desired_vel, current_vel, dt_seconds);
    // Sustain crawl speed only during accel/cruise, but allow complete taper down to 0 during deceleration
    if (!linear_profile_.isDecelerating() && desired_vel > 0.0f && linear_cmd < MOTOR_MIN_PWM) {
        linear_cmd = MOTOR_MIN_PWM;
    }
    if (search_motion_active_ && desired_vel > 0.0f && !linear_profile_.isDecelerating() &&
        millis() - linear_motion_start_ms_ < SEARCH_BREAKAWAY_BOOST_MS) {
        linear_cmd += SEARCH_BREAKAWAY_BOOST_COMMAND;
    }

    // 4. Angular Heading & Wall Centering Control
    float heading_error = target_heading_deg_ - imu.getHeadingDeg();
    while (heading_error > 180.0f)
        heading_error -= 360.0f;
    while (heading_error <= -180.0f)
        heading_error += 360.0f;

    // Combine side-wall centering with front/rear alignment correction.
#ifdef ENABLE_IR_WALL_CENTERING
    // Keep wall guidance active during exploration crawl (even when approaching front wall)
    if (centering_enabled_ && fabsf(desired_vel) > 8.0f)
    {
        // Centering injection: up to +/- 10.0 degrees max to firmly steer back to centerline
        float centering_bias_deg = constrain(ir_sensors.getCenteringError() * -10.0f, -10.0f, 10.0f);
        heading_error += centering_bias_deg;

        // Wall alignment: keep robot chassis parallel to the corridor walls
        float wall_yaw = ir_sensors.getWallAlignmentErrorDeg();
        if (fabsf(wall_yaw) > 0.5f && wall_alignment_gain_ > 0.0f) {
            heading_error += constrain(wall_yaw * wall_alignment_gain_, -6.0f, 6.0f);
        }
    }
#endif

    float angular_cmd = pid_angular_heading_.updateError(heading_error, dt_seconds);

    // A pivot has no requested translation. Cap differential steering while idle.
    if (!linear_motion_active_) {
        angular_cmd = constrain(angular_cmd, -TURN_MAX_MOTOR_COMMAND, TURN_MAX_MOTOR_COMMAND);
    } else if (desired_vel > 0.0f) {
        angular_cmd *= MOVING_TURN_STEERING_BOOST;
    }

    // A forward move must retain adequate steering authority even at low start/stop speeds
    if (desired_vel > 0.0f) {
        float max_steering_cmd = fmaxf(40.0f, 0.45f * fabsf(linear_cmd));
        angular_cmd = constrain(angular_cmd, -max_steering_cmd, max_steering_cmd);
    }

    // 5. Differential Motor Mixing
    int16_t left_motor_pwm = (int16_t)(linear_cmd - angular_cmd);
    int16_t right_motor_pwm = (int16_t)(linear_cmd + angular_cmd);

#ifdef DEBUG_MOTOR_COMMAND_STREAM
    static uint32_t last_motor_debug_time = 0;
    if (millis() - last_motor_debug_time >= 250) {
        last_motor_debug_time = millis();
        Serial.printf("[MOTOR DEBUG] heading target=%+.1f current=%+.1f error=%+.1fdeg | target=%+.1fmm/s measured L=%+.1f R=%+.1fmm/s | cmd linear=%+.1f steer=%+.1f -> L=%d R=%d\n",
                      target_heading_deg_,
                      imu.getHeadingDeg(),
                      heading_error,
                      desired_vel,
                      encoders.getLeftSpeedMM_S(),
                      encoders.getRightSpeedMM_S(),
                      linear_cmd,
                      angular_cmd,
                      left_motor_pwm,
                      right_motor_pwm);
    }
#endif

    // If profile is finished and zero velocity requested, stop active driving
    if (linear_profile_.isFinished() && angular_profile_.isFinished() &&
        fabsf(desired_vel) < 1.0f)
    {
        motors.stop(true);
    }
    else
    {
        motors.setSpeeds(left_motor_pwm, right_motor_pwm);
    }
}

bool MotionController::moveForward(float distance_mm, float max_speed, float end_speed,
                                   bool allow_centering, float wall_alignment_gain)
{
    centering_enabled_ = allow_centering;
    wall_alignment_gain_ = constrain(wall_alignment_gain, 0.0f, 1.0f);
    if (distance_mm > 0.0f) {
        // Snap target heading to the nearest cardinal grid angle (0, -90, 180, 90)
        // so forward movements track strictly along corridor axes without drifting
        float cardinal = roundf(target_heading_deg_ / 90.0f) * 90.0f;
        while (cardinal > 180.0f)  cardinal -= 360.0f;
        while (cardinal <= -180.0f) cardinal += 360.0f;
        setTargetHeading(cardinal);
        pid_angular_heading_.reset();
    }
    encoders.resetDistance();
    pid_linear_vel_.reset();

    float start_speed = encoders.getForwardSpeedMM_S();
    if (start_speed < MIN_SPEED_MM_S) {
        start_speed = MIN_SPEED_MM_S;
    }
    linear_profile_.start(distance_mm, start_speed, end_speed, max_speed, SEARCH_ACCEL_MM_S2, DECEL_MM_S2);
    search_motion_active_ = max_speed <= SEARCH_SPEED_MM_S;
    linear_motion_start_ms_ = millis();
    linear_motion_active_ = true;
    last_tick_micros_ = micros();

    uint32_t start_time = millis();
    // Maximum timeout based on expected move duration + safety buffer
    uint32_t timeout_ms = (uint32_t)((fabsf(distance_mm) / (max_speed * 0.6f)) * 1000.0f) + 1200;

    bool reached_cell_front_wall = false;
    bool emergency_stopped = false;

    while (!linear_profile_.isFinished())
    {
        uint32_t now = micros();
        float dt = (now - last_tick_micros_) * 1e-6f;
        if (dt >= CONTROL_DT_S)
        {
            last_tick_micros_ = now;
            update(dt);
        }

        float traveled_mm = fabsf(encoders.getAverageDistanceMM());
        uint16_t front_adc = ir_sensors.getFront();

        // 1. Target distance completion: break once measured encoder distance reaches target
        // (accounts for ~1.0 mm dynamic motor brake settling transition)
        if (linear_profile_.isFinished() || traveled_mm >= (fabsf(distance_mm) - 1.0f)) {
            break;
        }

        // 2. Front wall arrival stop:
        // When approaching a front wall, stop at cell center standoff (~50 mm from wall).
        if (front_adc >= IR_FRONT_STOP_DIST || front_adc >= IR_FRONT_CRITICAL_DIST) {
            if (traveled_mm >= CELL_REACHED_THRESHOLD_MM) {
                Serial.printf("[MOTION] Front wall reached (raw=%u, %.1f mm at traveled=%.1f mm). Cell arrival confirmed!\n",
                              front_adc, ir_sensors.getFrontMM(), traveled_mm);
                reached_cell_front_wall = true;
                break;
            }
        }

        if (millis() - start_time > timeout_ms)
        {
            Serial.println("[MOTION] moveForward Timeout!");
            emergency_stopped = true;
            break;
        }
    }

    linear_profile_.stopNow();
    motors.stop(true);
    delay(60); // Settle
    ir_sensors.flushFilter(4); // Refresh IR sensors at stationary standstill

    linear_motion_active_ = false;
    search_motion_active_ = false;

    float final_traveled_mm = fabsf(encoders.getAverageDistanceMM());
    uint16_t final_front_adc = ir_sensors.getFront();

    bool dist_ok = linear_profile_.isFinished() ||
                   (final_traveled_mm >= (fabsf(distance_mm) - MOTION_DISTANCE_TOLERANCE_MM));
    bool wall_ok = reached_cell_front_wall || 
                   (final_front_adc >= IR_WALL_DETECT_FRONT && final_traveled_mm >= CELL_REACHED_THRESHOLD_MM);

    Serial.printf("[MOTION] moveForward done: req=%.1fmm, enc_avg=%.1fmm (L=%.1f, R=%.1f, ticks L=%ld R=%ld), time=%lums, dist_ok=%s, wall_ok=%s\n",
                  distance_mm, final_traveled_mm,
                  encoders.getLeftDistanceMM(), encoders.getRightDistanceMM(),
                  (long)encoders.getLeftTicks(), (long)encoders.getRightTicks(),
                  millis() - start_time,
                  dist_ok ? "YES" : "NO", wall_ok ? "YES" : "NO");

    if (wall_ok || dist_ok) {
        return true;
    }

    if (emergency_stopped) {
        Serial.printf("[MOTION] Move aborted due to timeout (traveled=%.1f mm).\n", final_traveled_mm);
        return false;
    }

    return false;
}

bool MotionController::moveForwardCells(int num_cells, float max_speed, float end_speed, bool allow_centering)
{
    float distance = (float)num_cells * CELL_DIMENSION_MM;
    return moveForward(distance, max_speed, end_speed, allow_centering);
}

bool MotionController::turnInPlace(float angle_deg)
{
    if (fabsf(angle_deg) < 0.2f || !motors.isConnected()) {
        return false;
    }

    centering_enabled_ = false;
    linear_motion_active_ = false;
    linear_profile_.stopNow();
    angular_profile_.stopNow();
    motors.stop(true);
    delay(20);

    const float start_heading = imu.getHeadingDeg();
    float prev_heading = start_heading;
    float imu_accumulated_rotation = 0.0f; // Continuous unwrapped degrees from IMU gyro

    pid_angular_heading_.reset();
    encoders.reset();
    last_tick_micros_ = micros();
    const float target_abs_angle = fabsf(angle_deg);
    const float MAX_TURN_RATE_DPS = TURN_SPEED_DPS; // Controlled rotational rate (90 deg/s)
    const float DECEL_K = 18.0f;                    // Smooth sqrt braking curve gain

    // Calculate maximum duration based on 90 dps: 90° takes ~1.5s, 360° takes ~4.5s
    uint32_t start_time = millis();
    uint32_t timeout_ms = (uint32_t)(target_abs_angle * 10.0f) + 600;
    uint32_t within_tolerance_since = 0;

    float enc_accumulated_rotation = 0.0f;

    while (millis() - start_time < timeout_ms)
    {
        uint32_t now = micros();
        float dt = (now - last_tick_micros_) * 1e-6f;
        if (dt >= CONTROL_DT_S)
        {
            last_tick_micros_ = now;
            encoders.update(dt);
            imu.update(dt, encoders.getEncoderYawRateDeg_S(), 0.0f);

            // 1. IMU Integrated Rotation (unwrapped)
            float current_h = imu.getHeadingDeg();
            float delta_h = current_h - prev_heading;
            while (delta_h > 180.0f)  delta_h -= 360.0f;
            while (delta_h <= -180.0f) delta_h += 360.0f;
            imu_accumulated_rotation += delta_h;
            prev_heading = current_h;

            // 2. Wheel Encoder Differential Rotation:
            // Arc length difference between right and left wheels converted to chassis degrees
            float enc_diff_mm = encoders.getRightDistanceMM() - encoders.getLeftDistanceMM();
            enc_accumulated_rotation = (enc_diff_mm / TRACK_WIDTH_MM) * (180.0f / 3.1415926535f);

            // 3. Fused Rotation Determination:
            // If IMU is active and tracks in reasonable agreement (within 25%) of physical wheel travel,
            // use high-resolution IMU; otherwise, wheel encoder odometry takes precedence.
            float effective_rotation;
            if (imu.isConnected() && fabsf(imu_accumulated_rotation) >= 0.75f * fabsf(enc_accumulated_rotation)) {
                effective_rotation = imu_accumulated_rotation;
            } else {
                effective_rotation = enc_accumulated_rotation;
            }

            // 4. Hard Safety Bounding:
            // Physical wheel travel CANNOT exceed target angle by more than 1.5 degrees.
            if (fabsf(enc_accumulated_rotation) >= target_abs_angle + 1.5f) {
                break;
            }

            float remaining_angle = angle_deg - effective_rotation;
            float abs_err = fabsf(remaining_angle);
            float current_yaw_rate = imu.getYawRateDeg_S();
            if (!imu.isConnected() || fabsf(current_yaw_rate) < 0.5f) {
                current_yaw_rate = encoders.getEncoderYawRateDeg_S();
            }

            // Convergence check:
            // Condition A: within 2.5 degrees of target rotation and settled yaw rate for 15 ms
            if (abs_err <= 2.5f && fabsf(current_yaw_rate) < 14.0f) {
                if (within_tolerance_since == 0) {
                    within_tolerance_since = millis();
                } else if (millis() - within_tolerance_since >= 15) {
                    break; // Target confirmed reached and settled
                }
            } else {
                within_tolerance_since = 0;
            }

            // Condition B: Chassis has completed turning and come to physical rest (yaw rate < 4 dps)
            if (millis() - start_time > 200 && abs_err <= 4.0f && fabsf(current_yaw_rate) < 4.0f) {
                break; // Target achieved and robot has come to rest; proceed immediately
            }

            // Condition C: Physical wheel travel reached target angle and rotation has decelerated
            if (millis() - start_time > 200 && fabsf(enc_accumulated_rotation) >= target_abs_angle && fabsf(current_yaw_rate) < 15.0f) {
                break;
            }

            // Closed-loop rotational velocity profile:
            float current_turn_sign = (remaining_angle > 0.0f) ? 1.0f : -1.0f;
            float target_rate_magnitude;
            if (abs_err > 20.0f) {
                target_rate_magnitude = MAX_TURN_RATE_DPS;
            } else {
                target_rate_magnitude = fmaxf(10.0f, DECEL_K * sqrtf(abs_err));
            }
            float target_yaw_rate = current_turn_sign * target_rate_magnitude;

            // Closed-loop yaw rate error
            float rate_error = target_yaw_rate - current_yaw_rate;

            // Feedforward + Breakaway friction assist + Rate damping + Position trim
            float ff = target_yaw_rate * 0.30f;
            // Disable breakaway friction PWM once within 4.5 degrees so it cannot overshoot
            float friction_pwm = (fabsf(current_yaw_rate) < 18.0f && abs_err > 4.5f)
                ? (current_turn_sign * TURN_BREAKAWAY_PWM) : 0.0f;
            float rate_correction = 0.26f * rate_error;
            float pos_trim = constrain(current_turn_sign * fminf(abs_err, 35.0f) * 0.50f, -25.0f, 25.0f);

            float steer_cmd = ff + friction_pwm + rate_correction + pos_trim;

            // Anti-stall guarantee: when not yet moving and angle error is significant, sustain breakaway command
            if (abs_err > 5.0f && fabsf(current_yaw_rate) < 12.0f) {
                if (fabsf(steer_cmd) < TURN_BREAKAWAY_PWM) {
                    steer_cmd = current_turn_sign * TURN_BREAKAWAY_PWM;
                }
            }

            steer_cmd = constrain(steer_cmd, -TURN_MAX_MOTOR_COMMAND, TURN_MAX_MOTOR_COMMAND);

            // Positive angular command turns left (left wheel reverse, right wheel forward)
            int16_t left_pwm = (int16_t)(-steer_cmd);
            int16_t right_pwm = (int16_t)(steer_cmd);

            // Left turn motor balance:
            // When turning left (angle_deg > 0), the left motor runs in reverse against gearbox/floor
            // resistance while the stronger right motor pushes forward. Boost the left motor
            // so it turns enough to help and maintain symmetry.
            if (angle_deg > 0.0f) {
                left_pwm = (int16_t)(left_pwm * LEFT_TURN_LEFT_WHEEL_BOOST);
                right_pwm = (int16_t)(right_pwm * LEFT_TURN_RIGHT_WHEEL_TRIM);
            }

            // Closed-loop wheel synchronization during in-place turn:
            float left_spd = encoders.getLeftSpeedMM_S();
            float right_spd = encoders.getRightSpeedMM_S();
            float wheel_speed_diff = fabsf(left_spd) - fabsf(right_spd);
            if (wheel_speed_diff > 4.0f) {
                // Right wheel is lagging (higher friction) -> boost right wheel
                right_pwm += (int16_t)(current_turn_sign * constrain(wheel_speed_diff * 0.3f, 2.0f, 15.0f));
            } else if (wheel_speed_diff < -4.0f) {
                // Left wheel is lagging -> boost left wheel
                left_pwm -= (int16_t)(current_turn_sign * constrain(-wheel_speed_diff * 0.3f, 2.0f, 15.0f));
            }

            left_pwm = constrain(left_pwm, (int16_t)-MOTOR_MAX_SPEED, (int16_t)MOTOR_MAX_SPEED);
            right_pwm = constrain(right_pwm, (int16_t)-MOTOR_MAX_SPEED, (int16_t)MOTOR_MAX_SPEED);

            motors.setSpeeds(left_pwm, right_pwm);
        }
    }

    motors.stop(true);
    delay(25);

    // Snap target heading to cardinal grid angle based on starting target + angle
    float new_cardinal;
    if (fabsf(fmodf(target_abs_angle, 90.0f)) < 0.1f) {
        new_cardinal = roundf((target_heading_deg_ + angle_deg) / 90.0f) * 90.0f;
    } else {
        new_cardinal = target_heading_deg_ + angle_deg;
    }
    while (new_cardinal > 180.0f)  new_cardinal -= 360.0f;
    while (new_cardinal <= -180.0f) new_cardinal += 360.0f;
    setTargetHeading(new_cardinal);

    // Resynchronize IMU heading to match the confirmed cardinal target heading
    imu.resetHeading(new_cardinal);

    Serial.printf("[MOTION] Turn in place complete: req=%+.1f deg | imu_rot=%+.1f deg, enc_rot=%+.1f deg | snapped_target=%+.1f deg\n",
                  angle_deg, imu_accumulated_rotation, enc_accumulated_rotation, target_heading_deg_);
    return true;
}

bool MotionController::alignFrontWall(float approach_speed, uint16_t timeout_ms)
{
    if (!ir_sensors.hasFrontWall())
        return false;

    Serial.println("[MOTION] Aligning to Front Wall...");
    uint32_t start_time = millis();

    while (millis() - start_time < timeout_ms)
    {
        uint32_t now = micros();
        float dt = (now - last_tick_micros_) * 1e-6f;
        if (dt >= CONTROL_DT_S)
        {
            last_tick_micros_ = now;
            encoders.update(dt);
            imu.update(dt, encoders.getEncoderYawRateDeg_S(), encoders.getForwardSpeedMM_S());
            ir_sensors.update();

            // Drive slowly forward until front distance threshold reached
            if (ir_sensors.getFront() >= IR_FRONT_STOP_DIST)
            {
                break;
            }

            motors.setSpeeds((int16_t)approach_speed, (int16_t)approach_speed);
        }
    }

    motors.stop(true);
    delay(40);
    return true;
}

void MotionController::emergencyStop()
{
    linear_motion_active_ = false;
    linear_profile_.stopNow();
    angular_profile_.stopNow();
    motors.stop(true);
}
