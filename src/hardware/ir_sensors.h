#pragma once

#include <Arduino.h>
#include "config.h"
#include "types.h"

// =============================================================================
// 5-Channel Analog IR Distance Sensors (Sharp GP2Y0A51SK0F / 0A51SK)
// Range: 2 cm to 15 cm (20 mm to 150 mm)
// Pins:
//   Front      = 17
//   Front-left = 16, rear-left = 15
//   Front-right = 14, rear-right = 20
// =============================================================================

class IRSensorArray {
public:
    IRSensorArray();

    void begin();

    // High frequency update (called every control tick, e.g. 250 - 500 Hz)
    void update();

    // Auto-calibration in a known starting cell (Left, Right, and Front walls)
    bool calibrateInCell(uint16_t sample_count = 100);

    // Wall presence queries
    bool hasFrontWall() const;
    bool hasLeftWall() const;
    bool hasRightWall() const;

    // Corner / Post edge transitions for longitudinal position resetting
    bool hasLeftPostRising() const;
    bool hasLeftPostFalling() const;
    bool hasRightPostRising() const;
    bool hasRightPostFalling() const;

    // Centering steering error (-1.0 to +1.0)
    float getCenteringError() const;

    // Raw sensor readings and distances
    DistanceSensors getReadings() const;

    // Direct access to filtered ADC readings
    uint16_t getFront() const    { return readings_.front; }
    uint16_t getFrontLeft() const  { return readings_.front_left; }
    uint16_t getRearLeft() const   { return readings_.rear_left; }
    uint16_t getFrontRight() const { return readings_.front_right; }
    uint16_t getRearRight() const  { return readings_.rear_right; }

    // Physical millimeter distance queries (Sharp GP2Y0A51SK0F model)
    float getFrontMM() const     { return readings_.front_mm; }
    float getFrontLeftMM() const  { return readings_.front_left_mm; }
    float getRearLeftMM() const   { return readings_.rear_left_mm; }
    float getFrontRightMM() const { return readings_.front_right_mm; }
    float getRearRightMM() const  { return readings_.rear_right_mm; }

    float getWallAlignmentErrorDeg() const { return readings_.wall_alignment_error_deg; }

    // Sharp GP2Y0A51SK0F voltage and distance conversion
    static float adcToVoltage(uint16_t raw_adc);
    static float voltageToDistanceMM(float voltage);

private:
    uint16_t readAnalogOversampled(uint8_t pin, uint8_t samples = 4);

    DistanceSensors readings_;

    // Calibration baselines and detection thresholds
    uint16_t thresh_front_;
    uint16_t thresh_side_;

    float nominal_side_dist_mm_;

    // Previous readings for edge detection
    uint16_t prev_left_side_;
    uint16_t prev_right_side_;
    bool post_rising_l_;
    bool post_falling_l_;
    bool post_rising_r_;
    bool post_falling_r_;
    bool filter_initialized_;
};

extern IRSensorArray ir_sensors;
