#include "imu.h"

IMUDriver imu;

IMUDriver::IMUDriver()
    : is_initialized_(false),
      gyro_scale_dps_(0.0350f * IMU_GYRO_SCALE_FACTOR),  // Calibrated for physical turn geometry
      accel_scale_g_(0.000061f), // 0.061 mg/LSB for ±2g full scale
      gyro_bias_z_(0.0f),
      raw_yaw_rate_dps_(0.0f),
      heading_deg_(0.0f),
      yaw_rate_dps_(0.0f),
      accel_x_g_(0.0f),
      accel_y_g_(0.0f),
      yaw_axis_(IMU_YAW_AXIS),
      yaw_sign_(IMU_YAW_SIGN),
      poll_tick_(0) {
}

bool IMUDriver::begin() {
    Wire.begin();
    Wire.setClock(I2C_BUS_SPEED);

    if (!lsm6_.init()) {
        Serial.println("[IMU] WARNING: Pololu MinIMU-9 (LSM6DS33) not detected at 0x6B or 0x6A!");
        Serial.println("[IMU] Robot will fallback to differential wheel encoder odometry.");
        is_initialized_ = false;
        return false;
    }

    lsm6_.enableDefault();
    delay(10);

    // Configure Gyroscope: 1.66 kHz ODR, ±1000 dps full-scale (0x88)
    // Bits: ODR=1000 (1.66 kHz), FS_G=10 (1000 dps), FS_125=0
    uint8_t ctrl2_readback = 0;
    for (int retry = 0; retry < 3; ++retry) {
        lsm6_.writeReg(LSM6::CTRL2_G, 0x88);
        delay(5);
        ctrl2_readback = lsm6_.readReg(LSM6::CTRL2_G);
        if (ctrl2_readback == 0x88) break;
    }
    Serial.printf("[IMU] CTRL2_G register set: 0x88, readback: 0x%02X\n", ctrl2_readback);

    // Configure Accelerometer: CTRL1_XL = 0x80 (1.66 kHz ODR, ±2g)
    lsm6_.writeReg(LSM6::CTRL1_XL, 0x80);
    delay(5);
    accel_scale_g_ = 0.000061f; // 0.061 mg/LSB

    // Set gyro sensitivity scale strictly matching configuration, calibrated by IMU_GYRO_SCALE_FACTOR
    // Default to ±1000 dps (0.0350 * 1.5000 = 0.0525 dps/LSB) unless readback explicitly confirms ±245 dps (0x80)
    if (ctrl2_readback == 0x80) {
        gyro_scale_dps_ = 0.00875f * IMU_GYRO_SCALE_FACTOR;  // (±245 dps)
        Serial.println("[IMU] Gyro Scale: ±245 dps calibrated");
    } else if (ctrl2_readback == 0x84) {
        gyro_scale_dps_ = 0.0175f * IMU_GYRO_SCALE_FACTOR;   // (±500 dps)
        Serial.println("[IMU] Gyro Scale: ±500 dps calibrated");
    } else if (ctrl2_readback == 0x8C) {
        gyro_scale_dps_ = 0.0700f * IMU_GYRO_SCALE_FACTOR;   // (±2000 dps)
        Serial.println("[IMU] Gyro Scale: ±2000 dps calibrated");
    } else {
        gyro_scale_dps_ = 0.0350f * IMU_GYRO_SCALE_FACTOR;   // ±1000 dps calibrated (0.0525 dps/LSB)
        Serial.printf("[IMU] Gyro Scale: ±1000 dps calibrated (%.5f dps/LSB)\n", gyro_scale_dps_);
    }

    is_initialized_ = true;

    // Hardcode yaw axis and sign from config:
    // Ratatouieee MinIMU-9 v5 is mounted horizontally on the PCB.
    // Vertical yaw axis is strictly Z (axis=2) with positive sign (+1.0f).
    yaw_axis_ = IMU_YAW_AXIS;
    yaw_sign_ = IMU_YAW_SIGN;

    // Read accelerometer once for telemetry log
    lsm6_.read();
    float ax = (float)lsm6_.a.x * accel_scale_g_;
    float ay = (float)lsm6_.a.y * accel_scale_g_;
    float az = (float)lsm6_.a.z * accel_scale_g_;
    Serial.printf("[IMU] Accelerometer at boot: Ax=%+.2fg, Ay=%+.2fg, Az=%+.2fg | Yaw Axis: Z (%d), Sign: %+.1f\n",
                  ax, ay, az, yaw_axis_, yaw_sign_);

    // Allow sensor filter to settle before zero-rate calibration
    delay(100);

    // Calibrate stationary gyro bias
    calibrateStaticBias(300);

    resetHeading(0.0f);
    return true;
}

void IMUDriver::calibrateStaticBias(uint16_t sample_count) {
    if (!is_initialized_) return;

    Serial.println("[IMU] Calibrating static gyro bias... Keep robot stationary.");
    float sum_yaw = 0.0f;
    uint16_t valid_samples = 0;

    for (uint16_t i = 0; i < sample_count; ++i) {
        lsm6_.read();
        int16_t raw_counts = (yaw_axis_ == 0) ? lsm6_.g.x :
                             (yaw_axis_ == 1) ? lsm6_.g.y :
                                                lsm6_.g.z;
        sum_yaw += (float)raw_counts * gyro_scale_dps_ * yaw_sign_;
        valid_samples++;
        delay(3);
    }

    if (valid_samples > 0) {
        float measured_bias = sum_yaw / (float)valid_samples;
        // Sanity check: stationary bias on LSM6DS33 is typically within +/- 15 dps.
        // If the robot was handled or placed down during calibration, reject outliers.
        if (fabsf(measured_bias) < 30.0f) {
            gyro_bias_z_ = measured_bias;
            Serial.printf("[IMU] Static Gyro Bias calibrated: %6.3f dps (from %d samples, axis=%d)\n",
                          gyro_bias_z_, valid_samples, yaw_axis_);
        } else {
            Serial.printf("[IMU] WARNING: Measured bias %6.3f dps exceeds stationary threshold; keeping previous bias: %6.3f dps\n",
                          measured_bias, gyro_bias_z_);
        }
    }
}

void IMUDriver::update(float dt_seconds, float encoder_yaw_rate, float linear_speed_mm_s) {
    if (dt_seconds <= 0.0f) {
        dt_seconds = CONTROL_DT_S;
    }

    poll_tick_++;

    if (is_initialized_) {
        // Read raw sensor registers from LSM6DS33
        lsm6_.read();

        // Convert raw gyro readings into calibrated degrees per second on the configured yaw axis
        int16_t raw_counts = (yaw_axis_ == 0) ? lsm6_.g.x :
                             (yaw_axis_ == 1) ? lsm6_.g.y :
                                                lsm6_.g.z;
        raw_yaw_rate_dps_ = (float)raw_counts * gyro_scale_dps_ * yaw_sign_;
        yaw_rate_dps_ = raw_yaw_rate_dps_ - gyro_bias_z_;

        // Accelerometer readings in g
        accel_x_g_ = (float)lsm6_.a.x * accel_scale_g_;
        accel_y_g_ = (float)lsm6_.a.y * accel_scale_g_;

        // Zero-Velocity Update (ZUPT):
        // When encoders show robot is stationary, lock yaw rate to 0 to prevent stationary heading drift
        bool is_stopped = (fabsf(linear_speed_mm_s) < 2.5f) && (fabsf(encoder_yaw_rate) < 0.8f);
        if (is_stopped) {
            gyro_bias_z_ = 0.998f * gyro_bias_z_ + 0.002f * raw_yaw_rate_dps_;
            yaw_rate_dps_ = 0.0f;
        }

        // Integrate yaw rate into heading using standard trapezoidal integration
        heading_deg_ += yaw_rate_dps_ * dt_seconds;
    } else {
        // Fallback: integrate differential wheel odometry directly
        yaw_rate_dps_ = encoder_yaw_rate;
        heading_deg_ += encoder_yaw_rate * dt_seconds;
    }

    // Wrap heading to [-180.0, +180.0]
    while (heading_deg_ > 180.0f)  heading_deg_ -= 360.0f;
    while (heading_deg_ <= -180.0f) heading_deg_ += 360.0f;
}

float IMUDriver::getHeadingDeg() const {
    return heading_deg_;
}

float IMUDriver::getHeadingRad() const {
    return heading_deg_ * (3.1415926535f / 180.0f);
}

float IMUDriver::getYawRateDeg_S() const {
    return yaw_rate_dps_;
}

float IMUDriver::getRawYawRateDeg_S() const {
    return raw_yaw_rate_dps_;
}

float IMUDriver::getGyroBiasDeg_S() const {
    return gyro_bias_z_;
}

void IMUDriver::resetHeading(float new_heading_deg) {
    heading_deg_ = new_heading_deg;
    yaw_rate_dps_ = 0.0f;
}

bool IMUDriver::isConnected() const {
    return is_initialized_;
}

IMUReading IMUDriver::getReading() const {
    IMUReading r;
    r.gyro_z_dps  = yaw_rate_dps_;
    r.accel_x_g   = accel_x_g_;
    r.accel_y_g   = accel_y_g_;
    r.heading_deg = heading_deg_;
    r.is_ready    = is_initialized_;
    return r;
}
