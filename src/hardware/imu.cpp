#include "imu.h"

IMUDriver imu;

IMUDriver::IMUDriver()
    : is_initialized_(false),
      gyro_scale_dps_(0.035f),  // 35 mdps/LSB for ±1000 dps full scale
      accel_scale_g_(0.000061f), // 0.061 mg/LSB for ±2g full scale
      gyro_bias_z_(0.0f),
      heading_deg_(0.0f),
      yaw_rate_dps_(0.0f),
      accel_x_g_(0.0f),
      accel_y_g_(0.0f),
      poll_tick_(0) {
}

bool IMUDriver::begin() {
    Wire.begin();
    Wire.setClock(I2C_BUS_SPEED);

    if (!lsm6_.init()) {
        Serial.println("[IMU] WARNING: Pololu MinIMU-9 (LSM6DS33) not detected at 0x6B or 0x6A.");
        Serial.println("[IMU] Robot will fallback to differential wheel encoder odometry.");
        is_initialized_ = false;
        return false;
    }

    lsm6_.enableDefault();

    // Configure Gyroscope: CTRL2_G = 0x88 (1.66 kHz ODR, ±1000 dps)
    // 1000 dps gives high dynamic range for fast turns while maintaining fine resolution
    lsm6_.writeReg(LSM6::CTRL2_G, 0x88);
    gyro_scale_dps_ = 0.035f;

    // Configure Accelerometer: CTRL1_XL = 0x80 (1.66 kHz ODR, ±2g)
    lsm6_.writeReg(LSM6::CTRL1_XL, 0x80);
    accel_scale_g_ = 0.000061f;

    is_initialized_ = true;
    Serial.println("[IMU] LSM6DS33 initialized (ODR: 1.66 kHz, Gyro: ±1000 dps, Accel: ±2g).");

    // Calibrate stationary gyro bias
    calibrateStaticBias(300);

    resetHeading(0.0f);
    return true;
}

void IMUDriver::calibrateStaticBias(uint16_t sample_count) {
    if (!is_initialized_) return;

    Serial.println("[IMU] Calibrating static gyro bias... Keep robot stationary.");
    float sum_z = 0.0f;
    uint16_t valid_samples = 0;

    for (uint16_t i = 0; i < sample_count; ++i) {
        lsm6_.read();
        sum_z += (float)lsm6_.g.z * gyro_scale_dps_;
        valid_samples++;
        delay(3);
    }

    if (valid_samples > 0) {
        gyro_bias_z_ = sum_z / (float)valid_samples;
        Serial.printf("[IMU] Static Z-Gyro Bias: %6.3f dps (from %d samples)\n", gyro_bias_z_, valid_samples);
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

        // Convert raw gyro readings into degrees per second
        float raw_gz_dps = (float)lsm6_.g.z * gyro_scale_dps_;
        yaw_rate_dps_    = raw_gz_dps - gyro_bias_z_;

        // Accelerometer readings in g
        accel_x_g_ = (float)lsm6_.a.x * accel_scale_g_;
        accel_y_g_ = (float)lsm6_.a.y * accel_scale_g_;

        // Zero-Velocity Update (ZUPT):
        // When robot is physically stopped (linear velocity < 3 mm/s and encoder rate < 0.5 deg/s),
        // slowly adapt bias to cancel any thermal drift
        bool is_stopped = (fabsf(linear_speed_mm_s) < 3.0f) && (fabsf(encoder_yaw_rate) < 0.5f);
        if (is_stopped) {
            gyro_bias_z_ = 0.995f * gyro_bias_z_ + 0.005f * raw_gz_dps;
        }

        // Integrate yaw rate into heading
        heading_deg_ += yaw_rate_dps_ * dt_seconds;

        // Complementary drift correction using differential wheel odometry
        // When turning steadily, gyro is 99% trusted, but encoder odometry provides long-term baseline
        float heading_error = encoder_yaw_rate - yaw_rate_dps_;
        heading_deg_ += 0.002f * heading_error * dt_seconds;
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
