#include "imu.h"

// =============================================================
// CONSTRUCTOR
// =============================================================

IMU::IMU()
    : bno_(55, 0x28, &Wire),
      hardware_detected_(false),
      heading_offset_deg_(0.0f) {

    memset(&state_, 0, sizeof(state_));
}


// =============================================================
// BEGIN
// =============================================================

void IMU::begin() {

    Serial.println("[IMU] Initializing BNO055...");

    // Initialize the BNO055 using the Adafruit library.
    hardware_detected_ = bno_.begin();

    if (!hardware_detected_) {

        Serial.println("[IMU] BNO055 not detected.");
        Serial.println("[IMU] Using encoder heading fallback.");

        return;
    }

    Serial.println("[IMU] BNO055 detected.");

    // Use the BNO055 external crystal.
    bno_.setExtCrystalUse(true);

    // Give the BNO055 time to produce a valid orientation reading
    // before establishing the starting heading.
    delay(500);

    state_.is_calibrated = true;

    // Whatever direction the mouse is facing right now becomes 0°.
    resetHeading(0.0f);

    Serial.println("[IMU] Heading reset to 0 deg.");
}


// =============================================================
// UPDATE
// =============================================================

void IMU::update(
    float dt_seconds,
    float encoder_yaw_rate,
    float linear_speed_mm_s
) {

    // Currently unused, but kept because the rest of the robot
    // uses this function signature.
    (void)linear_speed_mm_s;

    if (dt_seconds <= 0.0f) {
        dt_seconds = CONTROL_DT_S;
    }


    // =========================================================
    // NORMAL CASE: BNO055 CONNECTED
    // =========================================================

    if (hardware_detected_) {

        sensors_event_t orientationData;
        sensors_event_t gyroData;


        // -----------------------------------------------------
        // READ HEADING
        // -----------------------------------------------------

        bno_.getEvent(
            &orientationData,
            Adafruit_BNO055::VECTOR_EULER
        );

        float raw_heading =
            orientationData.orientation.x;


        // -----------------------------------------------------
        // CONVERT TO ROBOT CONVENTION
        // -----------------------------------------------------
        //
        // Desired convention:
        //
        // LEFT / CCW  = positive
        // RIGHT / CW  = negative
        //
        // Therefore we negate the BNO055 heading.
        // -----------------------------------------------------

        float robot_heading =
            -raw_heading;


        // -----------------------------------------------------
        // MAKE HEADING RELATIVE TO STARTING POSITION
        // -----------------------------------------------------

        float relative_heading =
            robot_heading - heading_offset_deg_;

        relative_heading =
            normalizeAngle180(relative_heading);


        // -----------------------------------------------------
        // SAVE HEADING
        // -----------------------------------------------------

        state_.heading_deg =
            relative_heading;

        state_.heading_rad =
            relative_heading * DEG_TO_RAD;


        // -----------------------------------------------------
        // READ GYROSCOPE
        // -----------------------------------------------------

        bno_.getEvent(
            &gyroData,
            Adafruit_BNO055::VECTOR_GYROSCOPE
        );

        // Adafruit reports gyro in radians/second.
        // Convert to degrees/second.
        state_.gyro_z_deg_s =
            gyroData.gyro.z * RAD_TO_DEG;

        return;
    }


    // =========================================================
    // FALLBACK: NO BNO055
    // =========================================================
    //
    // If the BNO055 cannot be detected, estimate the heading
    // using the encoder yaw rate.
    // =========================================================

    state_.gyro_z_deg_s =
        encoder_yaw_rate;

    state_.heading_deg +=
        encoder_yaw_rate * dt_seconds;

    state_.heading_deg =
        normalizeAngle180(state_.heading_deg);

    state_.heading_rad =
        state_.heading_deg * DEG_TO_RAD;
}


// =============================================================
// RESET HEADING
// =============================================================

void IMU::resetHeading(float initial_heading_deg) {

    // ---------------------------------------------------------
    // PHYSICAL BNO055 AVAILABLE
    // ---------------------------------------------------------

    if (hardware_detected_) {

        sensors_event_t orientationData;

        // Read the current absolute orientation.
        bno_.getEvent(
            &orientationData,
            Adafruit_BNO055::VECTOR_EULER
        );

        float raw_heading =
            orientationData.orientation.x;

        // Convert to our robot convention.
        float robot_heading =
            -raw_heading;


        // -----------------------------------------------------
        // SAVE CURRENT DIRECTION AS THE REFERENCE
        // -----------------------------------------------------
        //
        // Example:
        //
        // Current robot heading = -157°
        // Desired starting heading = 0°
        //
        // offset = -157 - 0
        //
        // Later:
        //
        // relative = current - offset
        //
        // If the mouse hasn't moved:
        //
        // relative = -157 - (-157)
        //          = 0°
        //
        // -----------------------------------------------------

        heading_offset_deg_ =
            robot_heading - initial_heading_deg;
    }

    else {

        // No physical IMU, so there is no hardware
        // reference heading to save.
        heading_offset_deg_ = 0.0f;
    }


    // ---------------------------------------------------------
    // RESET SOFTWARE STATE
    // ---------------------------------------------------------

    state_.heading_deg =
        normalizeAngle180(initial_heading_deg);

    state_.heading_rad =
        state_.heading_deg * DEG_TO_RAD;

    state_.gyro_z_deg_s =
        0.0f;
}


// =============================================================
// NORMALIZE ANGLE TO -180 ... +180
// =============================================================

float IMU::normalizeAngle180(float angle_deg) {

    while (angle_deg > 180.0f) {
        angle_deg -= 360.0f;
    }

    while (angle_deg <= -180.0f) {
        angle_deg += 360.0f;
    }

    return angle_deg;
}


// =============================================================
// GET COMPLETE STATE
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
// HARDWARE STATUS
// =============================================================

bool IMU::isHardwareConnected() const {

    return hardware_detected_;
}