#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "config.h"
#include "types.h"


class IMU {

public:

    // =========================================================
    // CONSTRUCTOR
    // =========================================================

    IMU();


    // =========================================================
    // INITIALIZATION
    // =========================================================

    void begin();


    // =========================================================
    // UPDATE
    // =========================================================
    //
    // Reads the physical BNO055 and updates the IMU state.
    //
    // encoder_yaw_rate is kept as a fallback in case the
    // BNO055 is not detected.
    //
    // linear_speed_mm_s is kept so the interface remains
    // compatible with the rest of the robot code.
    // =========================================================

    void update(
        float dt_seconds,
        float encoder_yaw_rate = 0.0f,
        float linear_speed_mm_s = 0.0f
    );


    // =========================================================
    // GETTERS
    // =========================================================

    IMUState getState() const;

    float getHeadingDeg() const;

    float getGyroZ() const;


    // =========================================================
    // RESET HEADING
    // =========================================================
    //
    // Makes the mouse's current physical direction correspond
    // to initial_heading_deg.
    //
    // Normally:
    //
    //     resetHeading();
    //
    // means the current direction becomes 0 degrees.
    // =========================================================

    void resetHeading(
        float initial_heading_deg = 0.0f
    );


    // =========================================================
    // HARDWARE STATUS
    // =========================================================

    bool isHardwareConnected() const;


private:

    // =========================================================
    // ANGLE HELPER
    // =========================================================
    //
    // Converts an angle to the range:
    //
    //     -180 degrees to +180 degrees
    // =========================================================

    float normalizeAngle180(
        float angle_deg
    );


    // =========================================================
    // BNO055
    // =========================================================
    //
    // This is the same Adafruit BNO055 library used by the
    // working standalone IMU test.
    // =========================================================

    Adafruit_BNO055 bno_;


    // =========================================================
    // CURRENT IMU STATE
    // =========================================================

    IMUState state_;


    // =========================================================
    // HARDWARE CONNECTION
    // =========================================================

    bool hardware_detected_;


    // =========================================================
    // HEADING ZERO / OFFSET
    // =========================================================
    //
    // Stores the physical BNO055 direction that corresponds
    // to our software heading reference.
    //
    // Example:
    //
    // Mouse starts facing some direction.
    //
    // Physical BNO055 heading = 247 degrees
    //
    // We call:
    //
    //     resetHeading(0);
    //
    // That physical direction now becomes:
    //
    //     Robot heading = 0 degrees
    //
    // Turning left should then produce positive headings and
    // turning right should produce negative headings.
    // =========================================================

    float heading_offset_deg_;
};