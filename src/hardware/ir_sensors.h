#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"
#include "types.h"

class IRSensors {
public:
    IRSensors();
    void begin();

    // Call inside the control loop (pulses IR LEDs and samples ADC)
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

    // Calibration: Run while stationary inside a cell with Left, Right, and Front walls
    bool calibrateInCell(uint16_t sample_count = 200);

    // Save and load calibrated thresholds to/from ESP32 NVS Flash
    void saveToNVS();
    bool loadFromNVS();

    void setThresholds(uint16_t thresh_l90, uint16_t thresh_l45,
                       uint16_t thresh_front,
                       uint16_t thresh_r45, uint16_t thresh_r90);
    void setNominalCenters(uint16_t center_l45, uint16_t center_r45);

private:
    uint16_t readPulsedChannel(uint8_t emitter_pin, uint8_t receiver_pin);

    IRReadings readings_;

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
