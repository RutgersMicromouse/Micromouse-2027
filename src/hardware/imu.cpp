#include "imu.h"

// BNO055 Register Map
#define BNO055_PAGE_ID_ADDR        0x07
#define BNO055_CHIP_ID_ADDR        0x00
#define BNO055_OPR_MODE_ADDR       0x3D
#define BNO055_PWR_MODE_ADDR       0x3E
#define BNO055_SYS_TRIGGER_ADDR    0x3F
#define BNO055_UNIT_SEL_ADDR       0x3B
#define BNO055_EULER_H_LSB_ADDR    0x1A
#define BNO055_GYRO_DATA_Z_LSB     0x18

#define OPERATION_MODE_CONFIG      0x00
#define OPERATION_MODE_IMUPLUS     0x08
#define OPERATION_MODE_NDOF        0x0C

IMU::IMU()
    : hardware_detected_(false),
      heading_offset_deg_(0.0f),
      gyro_bias_z_(0.0f),
      poll_counter_(0) {
    memset(&state_, 0, sizeof(state_));
}

void IMU::begin() {
    hardware_detected_ = initBNO055();
    if (hardware_detected_) {
        Serial.println("[IMU] Bosch BNO055 initialized in IMU/Fusion mode.");

        // Measure static gyro bias at rest in starting cell (Zero-Velocity Calibration)
        float sum_gz = 0.0f;
        int valid_samples = 0;
        for (int i = 0; i < 50; ++i) {
            float h = 0.0f, gz = 0.0f;
            if (readBNO055Data(h, gz)) {
                sum_gz += gz;
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
    resetHeading(0.0f);
}

bool IMU::initBNO055() {
    // 1. Check Chip ID (Expected: 0xA0)
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(BNO055_CHIP_ID_ADDR);
    if (Wire.endTransmission() != 0) return false;

    Wire.requestFrom((uint8_t)BNO055_I2C_ADDR, (uint8_t)1);
    if (!Wire.available()) return false;
    uint8_t id = Wire.read();
    if (id != 0xA0) return false;

    // 2. Set to CONFIG mode before configuring registers
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(BNO055_OPR_MODE_ADDR);
    Wire.write(OPERATION_MODE_CONFIG);
    Wire.endTransmission();
    delay(25);

    // 3. Set normal power mode
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(BNO055_PWR_MODE_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(10);

    // 4. Set page 0
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(BNO055_PAGE_ID_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();

    // 5. Switch to IMU mode (combines Gyro + Accel for fast 100Hz fused orientation)
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(BNO055_OPR_MODE_ADDR);
    Wire.write(OPERATION_MODE_IMUPLUS);
    Wire.endTransmission();
    delay(20);

    state_.is_calibrated = true;
    return true;
}

bool IMU::readBNO055Data(float& heading_deg, float& gyro_z) {
    // Burst read 4 bytes starting at 0x18 (GYRO_DATA_Z_LSB):
    // 0x18: GYRO_Z_LSB, 0x19: GYRO_Z_MSB
    // 0x1A: EULER_H_LSB, 0x1B: EULER_H_MSB
    // Single transaction ensures shadow register latching
    Wire.beginTransmission(BNO055_I2C_ADDR);
    Wire.write(BNO055_GYRO_DATA_Z_LSB);
    if (Wire.endTransmission() != 0) return false;

    Wire.requestFrom((uint8_t)BNO055_I2C_ADDR, (uint8_t)4);
    if (Wire.available() < 4) return false;

    uint8_t gz_lsb = Wire.read();
    uint8_t gz_msb = Wire.read();
    uint8_t h_lsb  = Wire.read();
    uint8_t h_msb  = Wire.read();

    int16_t raw_gz = (int16_t)(((uint16_t)gz_msb << 8) | gz_lsb);
    int16_t raw_h  = (int16_t)(((uint16_t)h_msb  << 8) | h_lsb);

    // BNO055 Gyro Z scale: 16 LSB = 1 deg/s (CCW positive around +Z)
    gyro_z = (float)raw_gz / 16.0f;

    // BNO055 Euler heading scale: 1 degree = 16 LSB (0 to 360 deg, increasing clockwise)
    float raw_deg = (float)raw_h / 16.0f;

    // Convert clockwise BNO055 heading into standard counter-clockwise (CCW positive) navigation frame
    float standard_deg = (raw_deg == 0.0f) ? 0.0f : (360.0f - raw_deg);
    heading_deg = standard_deg;

    return true;
}

void IMU::update(float dt_seconds, float encoder_yaw_rate, float linear_speed_mm_s) {
    if (dt_seconds <= 0.0f) {
        dt_seconds = CONTROL_DT_S;
    }

    poll_counter_++;
    bool is_i2c_tick = (poll_counter_ % 5 == 0); // 100 Hz I2C sampling

    if (hardware_detected_ && is_i2c_tick) {
        float raw_h = 0.0f;
        float raw_gz = 0.0f;
        if (readBNO055Data(raw_h, raw_gz)) {
            // 1. Zero-Velocity Update (ZUPT): ONLY update static gyro bias when robot is ACTUALLY stationary
            bool is_stationary = (fabsf(linear_speed_mm_s) < 8.0f) && (fabsf(encoder_yaw_rate) < 1.0f);
            if (is_stationary) {
                gyro_bias_z_ = 0.98f * gyro_bias_z_ + 0.02f * raw_gz;
            }
            state_.gyro_z_deg_s = raw_gz - gyro_bias_z_;

            // 2. Relative heading from BNO055
            float bno_rel = raw_h - heading_offset_deg_;
            while (bno_rel > 180.0f)  bno_rel -= 360.0f;
            while (bno_rel <= -180.0f) bno_rel += 360.0f;

            // 3. Complementary Innovation Filter
            // Smoothly pulls high-rate encoder dead-reckoning into alignment with BNO055
            float err = bno_rel - state_.heading_deg;
            while (err > 180.0f)  err -= 360.0f;
            while (err <= -180.0f) err += 360.0f;

            state_.heading_deg += 0.25f * err; // K = 0.25 filter gain
            goto normalize_heading;
        }
    }

    // High-rate 500 Hz dead reckoning between I2C polls (no I2C traffic)
    {
        float active_rate = (hardware_detected_ && fabsf(state_.gyro_z_deg_s) > 0.05f)
                                ? state_.gyro_z_deg_s
                                : encoder_yaw_rate;
        state_.heading_deg += active_rate * dt_seconds;
        if (!hardware_detected_) {
            state_.gyro_z_deg_s = encoder_yaw_rate;
        }
    }

normalize_heading:
    while (state_.heading_deg > 180.0f)  state_.heading_deg -= 360.0f;
    while (state_.heading_deg <= -180.0f) state_.heading_deg += 360.0f;

    state_.heading_rad = state_.heading_deg * (PI / 180.0f);
}

IMUState IMU::getState() const {
    return state_;
}

float IMU::getHeadingDeg() const {
    return state_.heading_deg;
}

float IMU::getGyroZ() const {
    return state_.gyro_z_deg_s;
}

void IMU::resetHeading(float initial_heading_deg) {
    if (hardware_detected_) {
        float current_raw = 0.0f;
        float gz = 0.0f;
        if (readBNO055Data(current_raw, gz)) {
            heading_offset_deg_ = current_raw - initial_heading_deg;
        }
    }
    state_.heading_deg = initial_heading_deg;
    state_.heading_rad = initial_heading_deg * (PI / 180.0f);
    state_.gyro_z_deg_s = 0.0f;
}

bool IMU::isHardwareConnected() const {
    return hardware_detected_;
}
