#pragma once

// Drivers for everything physically wired to the ESP32:
//   Encoders   - N20 magnetic quadrature encoders (hardware pulse counters)
//   Motors     - Pololu Motoron M2T256 dual motor driver (I2C)
//   IRSensors  - 6 pulsed IR wall sensors
//   IMU        - Bosch BNO055 heading sensor (I2C), with encoder-odometry backup

#include "config.h"
#include "types.h"

// ==============================================================================
// ENCODERS
// ==============================================================================

#include <Arduino.h>
#include "driver/pcnt.h"

class Encoders {
public:
    Encoders();
    void begin();

    // Call every control loop tick (e.g. 1ms / 1kHz)
    void update(float dt_seconds);

    // Get current snapshot of encoder state
    EncoderState getState() const;

    // Reset distance & position accumulators
    void reset();

    // Invert channel direction if wiring is reversed
    void setInverted(bool invert_left, bool invert_right);
    void getInverted(bool& invert_left, bool& invert_right) const;

    // Wheel-speed smoothing weight (see ENCODER_SPEED_FILTER_ALPHA); adjustable for live tuning
    void setSpeedFilterAlpha(float alpha) { speed_filter_alpha_ = alpha; }
    float getSpeedFilterAlpha() const { return speed_filter_alpha_; }

    // Distance correction for live tuning: 1.05 makes every move 5 % longer on the floor.
    // Raise it if the robot stops short of a cell, lower it if it overshoots.
    void setDistanceScale(float scale) { distance_scale_ = scale; }
    float getDistanceScale() const { return distance_scale_; }

private:
    void initPcntUnit(pcnt_unit_t unit, int pin_a, int pin_b);

    EncoderState state_;
    float speed_filter_alpha_;
    float distance_scale_;
    bool invert_left_;
    bool invert_right_;

    float left_dist_acc_mm_;
    float right_dist_acc_mm_;
};

// ==============================================================================
// MOTORS
// ==============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <Motoron.h>

class Motors {
public:
    Motors();
    void begin();

    // Effort range: -1.0 (full reverse) to +1.0 (full forward) with trim applied
    void setEffort(float left_effort, float right_effort);

    // Raw effort without trim multipliers applied (used for calibration)
    void setRawEffort(float left_effort, float right_effort);

    // Motor balance trim (Left and Right multipliers)
    void setTrim(float trim_left, float trim_right);
    void getTrim(float& trim_left, float& trim_right) const;

    // Minimum breakaway duty cycle deadbands
    void setDeadband(float deadband_left, float deadband_right);
    void getDeadband(float& deadband_left, float& deadband_right) const;

    // Flash NVS storage for motor balance calibration
    void saveToNVS();
    bool loadFromNVS();

    // Actively brakes both motors
    void brake();

    // Freewheel / coast both motors (0 duty)
    void coast();

    // Enable/disable 12V boost converter power
    void setMotorPowerEnabled(bool enabled);

    // Invert motor polarities if needed
    void setInverted(bool invert_left, bool invert_right);
    void getInverted(bool& invert_left, bool& invert_right) const;

    // Driver health (refreshed every MOTORON_HEALTH_PERIOD_MS while commands are flowing)
    bool isConnected() const { return connected_; }
    uint16_t getStatusFlags() const { return status_flags_; }
    uint16_t getResetRecoveries() const { return reset_recoveries_; }
    uint16_t getBusFaults() const { return bus_faults_; }

    // Motor supply voltage measured by the Motoron (0 until the first reading)
    float getSupplyVolts() const { return supply_volts_; }

private:
    // (Re)applies all Motoron settings; returns true if the driver answered and is ready to drive
    bool configureMotoron();

    // Polls the Motoron status flags and transparently recovers from driver resets / bus faults
    void maintain(uint32_t now);

    void sendEfforts(float left_effort, float right_effort);

    MotoronI2C mc_;
    bool invert_left_;
    bool invert_right_;
    bool power_enabled_;
    float trim_left_;
    float trim_right_;
    float deadband_left_;
    float deadband_right_;

    // I2C bus bandwidth optimization & state tracking
    bool is_braking_;
    int16_t last_left_cmd_;
    int16_t last_right_cmd_;
    uint32_t last_cmd_time_ms_;

    // Health monitoring
    bool connected_;
    uint16_t status_flags_;
    uint32_t last_health_ms_;
    uint16_t reset_recoveries_;
    uint16_t bus_faults_;
    float supply_volts_;
};

// ==============================================================================
// IR WALL SENSORS
// ==============================================================================

#include <Arduino.h>
#include <Preferences.h>

class IRSensors {
public:
    IRSensors();
    void begin();

    // Call inside the control loop (pulses one emitter group per call and samples ADC)
    void update();

    // Current filtered readings
    IRReadings getReadings() const;

    // Helper wall queries
    bool hasLeftWall() const;
    bool hasRightWall() const;
    bool hasFrontWall() const;

    // Pillar / post edge detection (falling and rising)
    bool hasLeftPostEdge() const;
    bool hasRightPostEdge() const;
    bool hasLeftPostRising() const;
    bool hasRightPostRising() const;

    // Anticipated opening detection (wall terminating ahead)
    bool hasLeftOpening() const;
    bool hasRightOpening() const;

    // Predictive centering error for corridor following
    float getCenteringError() const;

    // Diagonal post guard: how much closer than normal the nearest post is, in [-1, 1].
    // Positive = too close on the left (steer right), negative = too close on the right.
    float getDiagonalGuardError() const;

    // Calibration: Run while stationary inside a cell with Left, Right, and Front walls
    bool calibrateInCell(uint16_t sample_count = 200);

    // Save and load calibrated thresholds to/from ESP32 NVS Flash
    void saveToNVS();
    bool loadFromNVS();

    void setThresholds(uint16_t thresh_l90, uint16_t thresh_l45,
                       uint16_t thresh_front,
                       uint16_t thresh_r45, uint16_t thresh_r90);
    void setNominalCenters(uint16_t center_l45, uint16_t center_r45);

    // Wiring check. Lights each emitter on its own for `on_time_us` and records how much every
    // receiver's reading rises: rise[emitter][receiver]. With the pin map right, each emitter's
    // own receiver (same index) shows by far the biggest rise when there is a wall in front.
    // Normal sampling is paused while it runs. Call only while the robot is standing still.
    void measureCrossTable(int16_t rise[6][6], uint32_t on_time_us);

    // For checking a sensor by hand: its latest raw ADC readings with the emitter off and on.
    // Channel 0..5 = L90, L45, FL, FR, R45, R90.
    uint16_t getRawAmbient(uint8_t channel) const { return last_ambient_[channel]; }
    uint16_t getRawLit(uint8_t channel) const { return last_lit_[channel]; }
    uint16_t getThresholdFront() const { return thresh_front_; }
    uint16_t getThresholdL90() const { return thresh_l90_; }
    uint16_t getThresholdR90() const { return thresh_r90_; }

    // Calibrated levels of the two 45° sensors: "a wall is there" threshold and centred reading
    uint16_t getThresholdL45() const { return thresh_l45_; }
    uint16_t getThresholdR45() const { return thresh_r45_; }
    uint16_t getNominalL45() const { return nominal_center_l45_; }
    uint16_t getNominalR45() const { return nominal_center_r45_; }

private:
    // Fires one interleaved emitter group and updates its three filtered channels
    void sampleGroup(const uint8_t* channels, uint8_t count);

    IRReadings readings_;
    uint16_t filtered_[6];            // CH1..CH6 = L90, L45, FL, FR, R45, R90
    uint16_t history_[6][2];          // Previous two raw samples per channel (median-of-3 spike filter)
    volatile bool paused_;            // True while measureCrossTable() has the emitters to itself
    uint16_t last_ambient_[6];        // Most recent raw ADC reading with the emitter off...
    uint16_t last_lit_[6];            // ...and with it on (for checking the sensors: `ir` command)
    volatile uint32_t update_count_;  // Even ticks sample group A, odd ticks group B

    uint16_t prev_l90_;
    uint16_t prev_r90_;

    uint16_t thresh_l90_;
    uint16_t thresh_l45_;
    uint16_t thresh_front_;
    uint16_t thresh_r45_;
    uint16_t thresh_r90_;

    uint16_t nominal_center_l45_;
    uint16_t nominal_center_r45_;

    Preferences prefs_;
};

// ==============================================================================
// IMU
// ==============================================================================

#include <Arduino.h>
#include <Wire.h>

// Bosch BNO055 heading source with encoder-odometry backup.
//
// Heading comes straight from the BNO055 fusion output (100 Hz), extrapolated to the 500 Hz control
// rate between reads. If the sensor is missing, glitches, drops off the bus, or resets itself, the
// heading keeps going on differential encoder odometry and the sensor is picked back up when it
// recovers, without a jump in the reported heading.
class IMU {
public:
    IMU();
    void begin();

    // Call inside control loop
    void update(float dt_seconds, float encoder_yaw_rate = 0.0f, float linear_speed_mm_s = 0.0f);

    IMUState getState() const;
    float getHeadingDeg() const;
    float getGyroZ() const;
    void resetHeading(float initial_heading_deg = 0.0f);

    // True if a BNO055 answered at boot
    bool isHardwareConnected() const;

    // True while heading is running on encoder odometry (no BNO055, or it is currently faulted)
    bool isUsingFallback() const;

    // Heading smoothing weight (see IMU_FILTER_ALPHA); adjustable for live tuning
    void setFilterAlpha(float alpha) { filter_alpha_ = alpha; }
    float getFilterAlpha() const { return filter_alpha_; }

    // Number of times the BNO055 dropped out or had to be put back into fusion mode since boot
    uint16_t getFaultCount() const { return fault_count_; }
    uint32_t getBadReadCount() const { return bad_read_total_; }  // Every failed or rejected read since power-on

    // True if the gyro Z axis had to be flipped to agree with the fused heading
    bool isGyroFlipped() const { return gyro_sign_ < 0; }

private:
    bool initBNO055(uint8_t address);
    bool readBNO055Data(float& heading_deg, float& gyro_z);
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegister(uint8_t reg, uint8_t& value);
    void acceptReading(float raw_h, float raw_gz, bool rate_valid,
                       float encoder_yaw_rate, float linear_speed_mm_s);

    IMUState state_;
    uint8_t address_;
    bool hardware_detected_;

    float heading_offset_deg_;     // BNO055 heading at the last resetHeading()
    float last_bno_heading_deg_;   // Most recent accepted BNO055 heading (sensor frame)
    float filter_alpha_;           // Share of each new BNO055 reading that is believed
    float heading_rate_deg_s_;     // Turn rate estimated by the heading filter (acceptReading)
    float time_since_good_s_;      // Time since the last accepted reading

    float gyro_bias_z_;
    int8_t gyro_sign_;
    int8_t gyro_agreement_;

    uint32_t poll_counter_;
    uint8_t bad_reads_;            // Consecutive rejected/failed reads (saturates at IMU_FAULT_READS)
    uint8_t settle_reads_;
    uint8_t frozen_reads_;
    bool reanchor_pending_;
    uint16_t fault_count_;
    uint32_t bad_read_total_;
    bool retry_read_;              // The last scheduled read failed: try again on the next tick
};
