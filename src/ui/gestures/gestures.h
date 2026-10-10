#pragma once

// A hand in front of the IR sensors (left, front or right), and what each one makes the robot do

#include "types.h"

// ==============================================================================
// HAND DETECTOR
// ==============================================================================

#include <stdint.h>

// Notices a hand arriving in front of one group of IR sensors and leaving again.
//
// The robot has no buttons, so this is how the operator talks to it. Feed it the sensor reading
// regularly; it reports each hand once, at the moment it leaves.
//
// It learns the resting level of the sensor by itself, so it works whether the sensor is looking
// at open floor or at a wall: a hand right in front of a sensor always reads brighter than
// whatever is behind it.
class GestureInput {
public:
    GestureInput();

    // Forget everything and re-learn the resting level (call at power-on and after every run)
    void reset(uint32_t now_ms);

    // Call every ~20 ms with the sensor reading. True once, when a hand that came has left again
    // (a "hand" that stays longer than GESTURE_REBASE_MS is taken for scenery and never reported).
    bool update(uint16_t reading, uint32_t now_ms);

    // True while something is in front of the sensors
    bool isHandPresent() const { return hand_present_; }

    // True while readings are only being used to learn the resting level (hands are ignored)
    bool isWarmingUp(uint32_t now_ms) const;

    // For checking the sensors by eye: what they read with nothing in front, and the reading a
    // hand has to push them above to be noticed
    float getRestingLevel() const { return resting_level_; }
    float getHandLevel() const;
    float getDropLevel() const;   // ...or drop below, when it is hiding a wall (below zero = cannot happen)

private:
    float resting_level_;       // What the sensor reads with no hand in front
    bool hand_present_;
    uint8_t confirm_samples_;   // Consecutive samples disagreeing with the current state (debounce)
    uint32_t started_ms_;
    uint32_t hand_since_ms_;
};

// ==============================================================================
// HAND CONTROLS
// ==============================================================================

// Hand control of the robot (it has no buttons or switches). Only the two front sensors are
// used, and only one sign: a hand in front of them, taken away again.
//
// Solid cyan = ready (dim cyan = wait, it is still learning what it sees). A hand starts the
// choosing. The LED then blinks the colour of a stage:
//
//   Green    Search run
//   Yellow   Speed run
//   White    Reset the sensors, then calibrate the IR
//   Blue     Forget the saved maze
//
// A hand before GESTURE_CONFIRM_BLINKS blinks have gone by = the next stage (after blue: back to
// cyan, nothing chosen). Leave it alone for that many blinks = that stage is done.
//
// A speed run then asks for its speed, and takes its time over it: the LED blinks the level's
// number in yellow (1 blink = slowest, GESTURE_SPEED_PERCENTS; also brighter for faster), pauses,
// and blinks it again. A hand = the next level (round again after the last). Left alone for
// GESTURE_SPEED_WAIT_MS it blinks rapidly for GESTURE_SPEED_GO_MS (a hand even then = next
// level) and goes.
//
// Lifting or turning the robot at any point cancels the choosing and changes nothing. Hands are
// ignored while the robot is being moved (it waits until it has stood still for GESTURE_WARMUP_MS).
namespace GestureUI {

// Start listening (also call after every run so it re-learns what the sensors see at rest)
void restart();

// Call from the operator task every ~20 ms while no run is active
void update(const IRReadings& ir);

// One line for the debug console: what the sensors read now and what a hand must exceed
void describe(char* buf, size_t size, const IRReadings& ir);

// For the phone app's hand-control display: what the LED is showing right now, in words, as a
// colour ("0f0" = green; "000" = dark) and as a brightness (0-255)
void describeForApp(char* meaning, size_t size, char color_hex[4], uint8_t& brightness);

} // namespace GestureUI
