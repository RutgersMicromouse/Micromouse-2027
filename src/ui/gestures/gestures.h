#pragma once

// Hand waves in front of the front IR sensors: counting them, and acting on the count

#include "types.h"

// ==============================================================================
// HAND-WAVE DETECTOR
// ==============================================================================

#include <stdint.h>

// Counts hand waves in front of the front IR sensors.
//
// The robot has no buttons, so this is how the operator talks to it. Feed it the front sensor
// reading regularly; when the operator has finished waving it reports how many waves it saw.
//
// It learns the resting level of the sensor by itself, so it works whether the robot is looking
// down an open corridor or at a wall: a hand right in front of the sensors always reads brighter
// than whatever is behind it.
class GestureInput {
public:
    GestureInput();

    // Forget everything and re-learn the resting level (call at power-on and after every run)
    void reset(uint32_t now_ms);

    // Call every ~20 ms with the front sensor reading.
    // Returns 0 normally, or the number of waves once the operator has stopped waving.
    uint8_t update(uint16_t front_reading, uint32_t now_ms);

    // True once per counted wave (for giving the operator a blink of feedback)
    bool consumeWaveCounted();

    // True while something is in front of the sensors
    bool isHandPresent() const { return hand_present_; }

    // Waves counted so far in the sequence that is still in progress
    uint8_t getPendingCount() const { return wave_count_; }

    // For checking the sensors by eye: what they read with nothing in front, and the reading a
    // hand has to push them above to be noticed
    float getRestingLevel() const { return resting_level_; }
    float getHandLevel() const;

private:
    float resting_level_;       // What the sensor reads with no hand in front
    bool hand_present_;
    uint8_t confirm_samples_;   // Consecutive samples disagreeing with the current state (debounce)
    uint8_t wave_count_;
    bool wave_counted_flag_;
    uint32_t started_ms_;
    uint32_t hand_since_ms_;
    uint32_t last_wave_end_ms_;
};

// ==============================================================================
// HAND-WAVE CONTROLS
// ==============================================================================

// Hand-wave control of the robot (it has no buttons or switches).
//
// Wave a hand in front of the two front sensors, pause, and the robot acts on the count:
//
//   1 wave   Search run            (Green)
//   2 waves  Speed run, hybrid     (Yellow)
//   3 waves  Speed run, diagonals  (Cyan)
//   4 waves  Speed run, curves     (Magenta)
//   5 waves  Calibrate IR sensors  (Yellow while sampling, then green = OK / red = failed)
//   6 waves  Clear the saved maze  (Blue flashes)
//
// The LED blinks white once per wave it counts, then blinks the count back in the action's color.
// Before a run it blinks rapidly for GESTURE_LAUNCH_DELAY_MS: get your hand out of the way, or
// cover the sensors again to cancel. Holding a hand in front mid-count also cancels.
namespace GestureUI {

// Start listening (also call after every run so it re-learns what the sensors see at rest)
void restart();

// Call from the navigation task every ~20 ms while no run is active
void update(const IRReadings& ir);

// One line for the debug console: what the front sensors read now, at rest, and need for a hand
void describe(char* buf, size_t size, const IRReadings& ir);

} // namespace GestureUI
