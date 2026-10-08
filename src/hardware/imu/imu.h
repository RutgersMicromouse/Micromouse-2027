#pragma once

// Bosch BNO055 heading sensor (I2C), with encoder-odometry backup

#include "config.h"
#include "types.h"

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

    // Why reads fail, for the `health` command: the sensor did not answer its address, answered
    // with too few bytes, or sent a heading that cannot be real. Plus the longest a read has taken.
    uint32_t getReadsNoAnswer() const { return fail_no_answer_; }
    uint32_t getReadsShort() const    { return fail_short_; }
    uint32_t getReadsBadValue() const { return fail_bad_value_; }
    uint32_t getReadsOk() const       { return reads_ok_; }
    uint32_t getSlowestReadUs() const { return slowest_read_us_; }

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
    uint32_t fail_no_answer_ = 0, fail_short_ = 0, fail_bad_value_ = 0, reads_ok_ = 0, slowest_read_us_ = 0;
    bool retry_read_;              // The last scheduled read failed: try again on the next tick
};
