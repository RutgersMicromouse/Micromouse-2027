#pragma once

#include <Arduino.h>
#include "config.h"
#include "types.h"

// =============================================================================
// 5-Channel Analog IR Distance Sensors (Sharp GP2Y0A51SK0F / 0A51SK)
// Range: 2 cm to 15 cm (20 mm to 150 mm)
// Pins:
//   FIR  = 17 (Front)
//   L1IR = 16 (Left 45°)
//   L2IR = 15 (Left 90°)
//   R1IR = 14 (Right 45°)
//   R2IR = 20 (Right 90°)
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
    uint16_t getLeft45() const   { return readings_.left_45; }
    uint16_t getLeft90() const   { return readings_.left_90; }
    uint16_t getRight45() const  { return readings_.right_45; }
    uint16_t getRight90() const  { return readings_.right_90; }

    // Physical millimeter distance queries (Sharp GP2Y0A51SK0F model)
    float getFrontMM() const     { return readings_.front_mm; }
    float getLeft45MM() const    { return readings_.left_45_mm; }
    float getLeft90MM() const    { return readings_.left_90_mm; }
    float getRight45MM() const   { return readings_.right_45_mm; }
    float getRight90MM() const   { return readings_.right_90_mm; }

    // Sharp GP2Y0A51SK0F voltage and distance conversion
    static float adcToVoltage(uint16_t raw_adc);
    static float voltageToDistanceMM(float voltage);

private:
    uint16_t readAnalogOversampled(uint8_t pin, uint8_t samples = 4);

    DistanceSensors readings_;

    // Calibration baselines and detection thresholds
    uint16_t thresh_front_;
    uint16_t thresh_l45_;
    uint16_t thresh_l90_;
    uint16_t thresh_r45_;
    uint16_t thresh_r90_;

    uint16_t nominal_center_l90_;
    uint16_t nominal_center_r90_;
    uint16_t nominal_center_l45_;
    uint16_t nominal_center_r45_;

    float nominal_side_dist_mm_;

    // Previous readings for edge detection
    uint16_t prev_l90_;
    uint16_t prev_r90_;
    bool post_rising_l_;
    bool post_falling_l_;
    bool post_rising_r_;
    bool post_falling_r_;
};

extern IRSensorArray ir_sensors;
