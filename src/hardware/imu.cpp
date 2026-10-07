#include "imu.h"
#include "math_utils.h"


// =============================================================
// BNO055 REGISTER MAP
// =============================================================

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


// =============================================================
// CONSTRUCTOR
// =============================================================

IMU::IMU()
    : hardware_detected_(false),
      heading_offset_deg_(0.0f),
      gyro_bias_z_(0.0f),
      poll_counter_(0) {

    memset(
        &state_,
        0,
        sizeof(state_)
    );
}


// =============================================================
// BEGIN
// =============================================================

void IMU::begin() {

    hardware_detected_ =
        initBNO055();


    // =========================================================
    // BNO055 FOUND
    // =========================================================

    if (hardware_detected_) {

        Serial.println(
            "[IMU] Bosch BNO055 initialized in IMU/Fusion mode."
        );


        // -----------------------------------------------------
        // Measure static gyro bias while mouse is stationary
        // -----------------------------------------------------

        float sum_gz = 0.0f;

        int valid_samples = 0;


        for (
            int i = 0;
            i < 50;
            ++i
        ) {

            float h = 0.0f;

            float gz = 0.0f;


            if (
                readBNO055Data(
                    h,
                    gz
                )
            ) {

                sum_gz += gz;

                valid_samples++;
            }


            delay(5);
        }


        // -----------------------------------------------------
        // Calculate gyro bias
        // -----------------------------------------------------

        if (valid_samples > 0) {

            gyro_bias_z_ =
                sum_gz
                /
                (float)valid_samples;


            Serial.printf(
                "[IMU] Calibrated Z-Gyro Static Bias: %5.3f deg/s\n",
                gyro_bias_z_
            );
        }

    }


    // =========================================================
    // BNO055 NOT FOUND
    // =========================================================

    else {

        Serial.println(
            "[IMU] BNO055 not detected on I2C. Falling back to differential encoder odometry."
        );
    }


    // =========================================================
    // INITIAL HEADING = 0
    // =========================================================

    resetHeading(
        0.0f
    );
}


// =============================================================
// INITIALIZE BNO055
// =============================================================

bool IMU::initBNO055() {

    // =========================================================
    // 1. CHECK CHIP ID
    //
    // Expected BNO055 ID = 0xA0
    // =========================================================

    Wire.beginTransmission(
        BNO055_I2C_ADDR
    );


    Wire.write(
        BNO055_CHIP_ID_ADDR
    );


    if (
        Wire.endTransmission() != 0
    ) {

        return false;
    }


    Wire.requestFrom(
        (uint8_t)BNO055_I2C_ADDR,
        (uint8_t)1
    );


    if (
        !Wire.available()
    ) {

        return false;
    }


    uint8_t id =
        Wire.read();


    if (
        id != 0xA0
    ) {

        return false;
    }


    // =========================================================
    // 2. CONFIG MODE
    // =========================================================

    Wire.beginTransmission(
        BNO055_I2C_ADDR
    );


    Wire.write(
        BNO055_OPR_MODE_ADDR
    );


    Wire.write(
        OPERATION_MODE_CONFIG
    );


    Wire.endTransmission();


    delay(25);


    // =========================================================
    // 3. NORMAL POWER MODE
    // =========================================================

    Wire.beginTransmission(
        BNO055_I2C_ADDR
    );


    Wire.write(
        BNO055_PWR_MODE_ADDR
    );


    Wire.write(
        0x00
    );


    Wire.endTransmission();


    delay(10);


    // =========================================================
    // 4. PAGE 0
    // =========================================================

    Wire.beginTransmission(
        BNO055_I2C_ADDR
    );


    Wire.write(
        BNO055_PAGE_ID_ADDR
    );


    Wire.write(
        0x00
    );


    Wire.endTransmission();


    // =========================================================
    // 5. IMUPLUS MODE
    //
    // Uses gyro + accelerometer fusion.
    // =========================================================

    Wire.beginTransmission(
        BNO055_I2C_ADDR
    );


    Wire.write(
        BNO055_OPR_MODE_ADDR
    );


    Wire.write(
        OPERATION_MODE_IMUPLUS
    );


    Wire.endTransmission();


    delay(20);


    state_.is_calibrated =
        true;


    return true;
}


// =============================================================
// READ BNO055
// =============================================================

bool IMU::readBNO055Data(
    float& heading_deg,
    float& gyro_z
) {

    // =========================================================
    // Burst read 4 bytes beginning at 0x18
    //
    // 0x18 = Gyro Z LSB
    // 0x19 = Gyro Z MSB
    // 0x1A = Euler Heading LSB
    // 0x1B = Euler Heading MSB
    // =========================================================

    Wire.beginTransmission(
        BNO055_I2C_ADDR
    );


    Wire.write(
        BNO055_GYRO_DATA_Z_LSB
    );


    if (
        Wire.endTransmission() != 0
    ) {

        return false;
    }


    Wire.requestFrom(
        (uint8_t)BNO055_I2C_ADDR,
        (uint8_t)4
    );


    if (
        Wire.available() < 4
    ) {

        return false;
    }


    // =========================================================
    // READ BYTES
    // =========================================================

    uint8_t gz_lsb =
        Wire.read();

    uint8_t gz_msb =
        Wire.read();

    uint8_t h_lsb =
        Wire.read();

    uint8_t h_msb =
        Wire.read();


    // =========================================================
    // COMBINE BYTES
    // =========================================================

    int16_t raw_gz =
        (int16_t)(
            ((uint16_t)gz_msb << 8)
            |
            gz_lsb
        );


    int16_t raw_h =
        (int16_t)(
            ((uint16_t)h_msb << 8)
            |
            h_lsb
        );


    // =========================================================
    // GYROSCOPE
    //
    // BNO055:
    // 16 LSB = 1 degree/second
    // =========================================================

    gyro_z =
        (float)raw_gz
        /
        16.0f;


    // =========================================================
    // EULER HEADING
    //
    // BNO055:
    // 16 LSB = 1 degree
    //
    // Native BNO055 heading increases clockwise.
    // =========================================================

    float raw_deg =
        (float)raw_h
        /
        16.0f;


    // =========================================================
    // CONVERT TO ROBOT COORDINATE SYSTEM
    //
    // We want:
    //
    // LEFT / CCW  = positive
    // RIGHT / CW  = negative
    //
    // Example:
    //
    // Start        0°
    // Left 90°    +90°
    // Right 90°   -90°
    // =========================================================

    float standard_deg =
        (raw_deg == 0.0f)
        ?
        0.0f
        :
        (360.0f - raw_deg);


    heading_deg =
        standard_deg;


    return true;
}


// =============================================================
// UPDATE
// =============================================================

void IMU::update(
    float dt_seconds,
    float encoder_yaw_rate,
    float linear_speed_mm_s
) {

    // =========================================================
    // VALIDATE DT
    // =========================================================

    if (
        dt_seconds <= 0.0f
    ) {

        dt_seconds =
            CONTROL_DT_S;
    }


    poll_counter_++;


    // =========================================================
    // READ BNO055 EVERY 5 CONTROL ITERATIONS
    // =========================================================

    bool is_i2c_tick =
        (
            poll_counter_
            %
            5
        )
        ==
        0;


    // =========================================================
    // BNO055 IS CONNECTED
    // =========================================================

    if (hardware_detected_) {

        if (is_i2c_tick) {

            float raw_h =
                0.0f;


            float raw_gz =
                0.0f;


            if (
                readBNO055Data(
                    raw_h,
                    raw_gz
                )
            ) {

                // =================================================
                // CHECK IF ROBOT IS STATIONARY
                // =================================================

                bool is_stationary =

                    (
                        fabsf(
                            linear_speed_mm_s
                        )
                        <
                        8.0f
                    )

                    &&

                    (
                        fabsf(
                            encoder_yaw_rate
                        )
                        <
                        1.0f
                    );


                // =================================================
                // UPDATE GYRO BIAS WHILE STATIONARY
                // =================================================

                if (is_stationary) {

                    gyro_bias_z_ =

                        0.98f
                        *
                        gyro_bias_z_

                        +

                        0.02f
                        *
                        raw_gz;
                }


                // =================================================
                // SAVE GYRO RATE
                // =================================================

                state_.gyro_z_deg_s =

                    raw_gz

                    -

                    gyro_bias_z_;


                // =================================================
                // RELATIVE HEADING
                //
                // heading_offset_deg_ is the BNO055 heading when
                // resetHeading() was called.
                //
                // Therefore:
                //
                // current heading - starting heading
                //
                // gives us the amount the mouse has rotated.
                // =================================================

                float relative_heading =

                    raw_h

                    -

                    heading_offset_deg_;


                // =================================================
                // NORMALIZE
                //
                // Keep heading between:
                //
                // -180° and +180°
                // =================================================

                relative_heading =

                    normalizeAngle180(
                        relative_heading
                    );


                // =================================================
                // USE BNO055 HEADING DIRECTLY
                //
                // No complementary filter.
                // No encoder integration.
                // No gyro integration.
                //
                // This makes it easy to verify that the physical
                // IMU heading is correct.
                // =================================================

                state_.heading_deg =

                    relative_heading;


                state_.heading_rad =

                    state_.heading_deg

                    *

                    (
                        PI
                        /
                        180.0f
                    );
            }
        }


        // =====================================================
        // Between BNO055 reads, keep the most recent heading.
        // =====================================================

        return;
    }


    // =========================================================
    // FALLBACK
    //
    // BNO055 was not detected.
    //
    // Use differential encoder odometry instead.
    // =========================================================

    state_.gyro_z_deg_s =

        encoder_yaw_rate;


    state_.heading_deg +=

        encoder_yaw_rate

        *

        dt_seconds;


    // Keep between -180 and +180
    state_.heading_deg =

        normalizeAngle180(
            state_.heading_deg
        );


    state_.heading_rad =

        state_.heading_deg

        *

        (
            PI
            /
            180.0f
        );
}


// =============================================================
// GET COMPLETE IMU STATE
// =============================================================

IMUState IMU::getState() const {

    return state_;
}


// =============================================================
// GET HEADING
// =============================================================

float IMU::getHeadingDeg() const {

    return state_.heading_deg;
}


// =============================================================
// GET GYRO Z
// =============================================================

float IMU::getGyroZ() const {

    return state_.gyro_z_deg_s;
}


// =============================================================
// RESET HEADING
// =============================================================

void IMU::resetHeading(
    float initial_heading_deg
) {

    // =========================================================
    // IF BNO055 EXISTS:
    //
    // Store its current physical direction as our new reference.
    // =========================================================

    if (hardware_detected_) {

        float current_raw =
            0.0f;


        float gz =
            0.0f;


        if (
            readBNO055Data(
                current_raw,
                gz
            )
        ) {

            heading_offset_deg_ =

                current_raw

                -

                initial_heading_deg;
        }
    }


    // =========================================================
    // RESET SOFTWARE STATE
    // =========================================================

    state_.heading_deg =

        initial_heading_deg;


    state_.heading_rad =

        initial_heading_deg

        *

        (
            PI
            /
            180.0f
        );


    state_.gyro_z_deg_s =
        0.0f;
}


// =============================================================
// HARDWARE STATUS
// =============================================================

bool IMU::isHardwareConnected() const {

    return hardware_detected_;
}