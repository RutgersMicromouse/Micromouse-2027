#pragma once

// The six pulsed IR wall sensors

#include "config.h"
#include "types.h"

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
    bool isFrontLevelMeasured() const { return front_level_measured_; } // false = only an estimate so far

    // How far the robot is turned away from square to a wall in front of it, from the two front
    // sensors: positive = the left one is closer. Each sensor is first divided by what IT read
    // facing a wall squarely at calibration, because the two do not read alike (on this robot
    // the right one reads about twice the left). 0 = square, or not calibrated facing a wall.
    float getFrontSkew() const;
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
    bool front_level_measured_;       // thresh_front_ came from a real wall in front, not an estimate
    uint16_t nominal_fl_, nominal_fr_; // What each front sensor read facing a wall squarely at calibration (0 = unknown)
    uint16_t thresh_r45_;
    uint16_t thresh_r90_;

    uint16_t nominal_center_l45_;
    uint16_t nominal_center_r45_;

    Preferences prefs_;
};
