#include "hardware/imu/imu.h"

// ==============================================================================
// IMU
// ==============================================================================

// BNO055 register map
#define BNO055_CHIP_ID_ADDR        0x00
#define BNO055_PAGE_ID_ADDR        0x07
#define BNO055_GYRO_DATA_Z_LSB     0x18
#define BNO055_EULER_ROLL_LSB      0x1C   // Euler roll, then pitch (0x1E), 16 LSB = 1 degree
#define BNO055_LINEAR_ACCEL_X_LSB  0x28   // Acceleration with gravity removed: X, Y, Z, 100 LSB = 1 m/s2
#define BNO055_OPR_MODE_ADDR       0x3D
#define BNO055_PWR_MODE_ADDR       0x3E

#define BNO055_CHIP_ID             0xA0
#define BNO055_ALT_I2C_ADDR        0x29   // Address when the module's ADR pin is pulled high

#define OPERATION_MODE_CONFIG      0x00
#define OPERATION_MODE_IMUPLUS     0x08   // Gyro + accelerometer fusion (no magnetometer)

// The BNO055 is read every 5th control tick (100 Hz, its fusion output rate)
#define IMU_POLL_DIVIDER           5
// The acceleration (display only) is read every 25th tick, two ticks after a heading read so the
// two never share a tick, nor the tick a failed heading read is retried on
#define IMU_ACCEL_DIVIDER          25
#define IMU_ACCEL_TICK             2
#define IMU_TILT_DIVIDER           25   // Roll and pitch every 25th control tick (20 Hz), for the tilt stop
#define IMU_TILT_TICK              3    // ...three ticks after a heading read, so they never share a tick
#define IMU_MODE_CHECK_DIVIDER     500    // Verify fusion mode once per second
#define IMU_SETTLE_READS           5      // Reads discarded after re-entering fusion mode
#define IMU_FROZEN_READS           10     // Unchanged readings during a turn that trigger a mode check

IMU::IMU()
    : address_(BNO055_I2C_ADDR),
      hardware_detected_(false),
      heading_offset_deg_(0.0f),
      last_bno_heading_deg_(0.0f),
      filter_alpha_(IMU_FILTER_ALPHA),
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
      fault_count_(0),
      bad_read_total_(0),
      retry_read_(false) {
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
    // (Roll and pitch were once read in the same transfer, 8 bytes: on the robot that took the
    // share of failed reads from about half to three quarters and 180 degree turns overshot by
    // 20-55 degrees, 2026-10-10. They have their own, slower read now: readTilt.)
    const uint32_t started_us = micros();
    Wire.beginTransmission(address_);
    Wire.write(BNO055_GYRO_DATA_Z_LSB);
    const bool answered = (Wire.endTransmission() == 0);
    const bool complete = answered && Wire.requestFrom(address_, (uint8_t)4) == 4 && Wire.available() >= 4;
    const uint32_t took_us = micros() - started_us;
    if (took_us > slowest_read_us_) slowest_read_us_ = took_us;
    if (!answered) { fail_no_answer_++; return false; }
    if (!complete) { fail_short_++;     return false; }

    uint8_t gz_lsb = Wire.read();
    uint8_t gz_msb = Wire.read();
    uint8_t h_lsb  = Wire.read();
    uint8_t h_msb  = Wire.read();

    int16_t raw_gz = (int16_t)(((uint16_t)gz_msb << 8) | gz_lsb);
    int16_t raw_h  = (int16_t)(((uint16_t)h_msb << 8) | h_lsb);

    // Euler heading is 16 LSB per degree in [0, 360); anything else is a corrupted transfer
    if (raw_h < 0 || raw_h > 360 * 16) { fail_bad_value_++; return false; }
    reads_ok_++;

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

    // Read on the normal 100 Hz slot. If that read fails, try once more on the very next tick
    // instead of flying blind for another 10 ms (one retry only: a timed-out read is slow).
    const bool poll_now = (poll_counter_ % IMU_POLL_DIVIDER) == 0;
    if (hardware_detected_ && (poll_now || retry_read_)) {
        float raw_h = 0.0f;
        float raw_gz = 0.0f;
        bool ok = readBNO055Data(raw_h, raw_gz);
        retry_read_ = !ok && poll_now;

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
            if (reanchor_pending_) {
                // The BNO055 itself restarted (it had dropped out of fusion mode), so its heading
                // now counts from an unrelated direction: keep our tracked heading and re-anchor
                // the BNO055 to it
                heading_offset_deg_ = normalizeAngle180(raw_h - state_.heading_deg);
                reanchor_pending_ = false;
                rate_valid = false;
            } else if (fabsf(jump) > IMU_MAX_STEP_DEG && bad_reads_ < IMU_GLITCH_READS) {
                ok = false; // Physically impossible step: reject as a glitch
            }
            // Otherwise believe it, even if it is far from the tracked heading. The same far-off
            // answer several reads running is not a glitch: reads were failing, the heading was
            // being tracked on the wheel encoders meanwhile, and they lost count of the turn. The
            // BNO055 kept integrating its gyro all along, so its heading is the true one.

            if (ok) {
                // Bring the prediction up to this tick, so the reading is compared with where the
                // filter expects the heading to be now
                if (bad_reads_ == 0) {
                    state_.heading_deg = normalizeAngle180(state_.heading_deg + heading_rate_deg_s_ * dt_seconds);
                }
                acceptReading(raw_h, raw_gz, rate_valid, encoder_yaw_rate, linear_speed_mm_s);
                return;
            }
        }

        bad_read_total_++;
        if (poll_now && bad_reads_ < IMU_FAULT_READS) { // (the retry does not count twice)
            bad_reads_++;
            if (bad_reads_ == IMU_FAULT_READS) fault_count_++;
        }
    }

    if (hardware_detected_ && !retry_read_ && (poll_counter_ % IMU_TILT_DIVIDER) == IMU_TILT_TICK) {
        readTilt();
    }

#if ENABLE_ACCEL_DISPLAY
    if (hardware_detected_ && !retry_read_ && (poll_counter_ % IMU_ACCEL_DIVIDER) == IMU_ACCEL_TICK) {
        readAcceleration();
    }
#endif

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
    // Alpha-beta filter on the BNO055 heading. Between reads update() has already predicted
    // heading = heading + rate * dt. Each new reading then corrects that prediction by only part of
    // the difference, and nudges the rate by the same difference:
    //     heading += alpha * (measured - predicted)
    //     rate    += beta  * (measured - predicted) / time since the last reading
    // alpha starts at IMU_FILTER_ALPHA; beta = alpha^2 / (2 - alpha), which makes the filter
    // settle without ringing whatever alpha is set to.
    // The BNO055 reports in 1/16° steps at 100 Hz; taken raw, every step is a small jolt in the
    // heading and a 6°/s jump in the rate, which the 500 Hz heading loop passes on to the motors.
    // The rate still comes from the heading alone, so it is always sign-consistent with it.
    const float measured = normalizeAngle180(raw_h - heading_offset_deg_);
    if (rate_valid && bad_reads_ == 0 && time_since_good_s_ > 0.0f && time_since_good_s_ < 0.05f) {
        const float residual = normalizeAngle180(measured - state_.heading_deg);
        const float beta = filter_alpha_ * filter_alpha_ / (2.0f - filter_alpha_);
        state_.heading_deg = normalizeAngle180(state_.heading_deg + filter_alpha_ * residual);
        heading_rate_deg_s_ += beta * residual / time_since_good_s_;
    } else {
        // First reading, or the first after an outage: nothing to predict from, so take it as it
        // is. After a short outage the turn rate is simply the change since the last good reading.
        state_.heading_deg = measured;
        heading_rate_deg_s_ = (rate_valid && time_since_good_s_ > 0.0f && time_since_good_s_ < 0.2f)
                              ? normalizeAngle180(raw_h - last_bno_heading_deg_) / time_since_good_s_ : 0.0f;
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

    state_.heading_rad = state_.heading_deg * (PI / 180.0f);

    last_bno_heading_deg_ = raw_h;
    time_since_good_s_ = 0.0f;
    bad_reads_ = 0;
}

// Roll and pitch (0x1C / 0x1E, 16 LSB per degree): how far the robot is tipped out of level. A
// failed read, or values that cannot be real (roll beyond 90 degrees, pitch beyond 180), leave
// the last ones in place and are not counted with the heading reads.
void IMU::readTilt() {
    Wire.beginTransmission(address_);
    Wire.write(BNO055_EULER_ROLL_LSB);
    if (Wire.endTransmission() != 0) return;
    if (Wire.requestFrom(address_, (uint8_t)4) != 4 || Wire.available() < 4) return;
    const uint8_t roll_lsb  = Wire.read();
    const uint8_t roll_msb  = Wire.read();
    const uint8_t pitch_lsb = Wire.read();
    const uint8_t pitch_msb = Wire.read();
    const int16_t raw_roll  = (int16_t)(((uint16_t)roll_msb << 8) | roll_lsb);
    const int16_t raw_pitch = (int16_t)(((uint16_t)pitch_msb << 8) | pitch_lsb);
    if (abs(raw_roll) > 90 * 16 || abs(raw_pitch) > 180 * 16) return;
    if (raw_roll == -1 && raw_pitch == -1) return; // All ones: the sensor was not driving the bus
    roll_deg_  = (float)raw_roll / 16.0f;
    pitch_deg_ = (float)raw_pitch / 16.0f;
}

// Display only. A failed read just leaves the last values in place; it is not counted with the
// heading reads, so the health numbers keep meaning what they did.
void IMU::readAcceleration() {
    Wire.beginTransmission(address_);
    Wire.write(BNO055_LINEAR_ACCEL_X_LSB);
    if (Wire.endTransmission() != 0) return;
    if (Wire.requestFrom(address_, (uint8_t)6) != 6 || Wire.available() < 6) return;
    for (int axis = 0; axis < 3; ++axis) {
        const uint8_t lsb = Wire.read();
        const uint8_t msb = Wire.read();
        const float value = (float)(int16_t)(((uint16_t)msb << 8) | lsb) / 100.0f;
        accel_now_[axis] = value;
        if (fabsf(value) > accel_peak_[axis]) accel_peak_[axis] = fabsf(value);
    }
}

void IMU::getAcceleration(float now[3], float peak[3], bool clear_peaks) {
    for (int axis = 0; axis < 3; ++axis) {
        now[axis] = accel_now_[axis];
        peak[axis] = accel_peak_[axis];
        if (clear_peaks) accel_peak_[axis] = 0.0f;
    }
}

IMUState IMU::getState() const {
    return state_;
}

float IMU::getHeadingDeg() const {
    return state_.heading_deg;
}

float IMU::getTiltDeg() const {
    if (!hardware_detected_ || isUsingFallback()) return 0.0f; // No trustworthy reading: never "tilted"
    return fmaxf(fabsf(normalizeAngle180(roll_deg_ - level_roll_deg_)),
                 fabsf(normalizeAngle180(pitch_deg_ - level_pitch_deg_)));
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
        // The robot is standing on the floor whenever its heading is set: this is "level"
        level_roll_deg_  = roll_deg_;
        level_pitch_deg_ = pitch_deg_;
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
