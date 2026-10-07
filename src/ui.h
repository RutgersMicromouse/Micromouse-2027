#pragma once

// How the operator talks to the robot (it has no buttons or switches):
//   StatusLED    - the RGB LED on the ESP32 board
//   Actions      - everything the operator can ask for (select mode, launch, stop, calibrate...)
//   GestureInput - counts hand waves in front of the front IR sensors
//   GestureUI    - turns a wave count into an Action
//   Console      - debug-only text console (reports, settings, stop)

#include "types.h"

// ==============================================================================
// STATUS LED
// ==============================================================================

#include <Arduino.h>

// The RGB LED on the ESP32-S3 DevKit board: the robot's only indicator.
//
//   Green / Yellow / Cyan / Magenta = selected run mode (see Actions below)
//   White blink                     = a hand wave was counted
//   Red                             = stopped by a fault or low battery
namespace StatusLED {

struct Color {
    bool red;
    bool green;
    bool blue;
};

const Color OFF     = { false, false, false };
const Color RED     = { true,  false, false };
const Color GREEN   = { false, true,  false };
const Color BLUE    = { false, false, true  };
const Color YELLOW  = { true,  true,  false };
const Color CYAN    = { false, true,  true  };
const Color MAGENTA = { true,  false, true  };
const Color WHITE   = { true,  true,  true  };

void begin();

// Solid color
void set(Color color);

// Blink `count` times, ending with the LED off (blocks for count * 2 * delay_ms)
void flash(Color color, int count = 3, int delay_ms = 150);

} // namespace StatusLED

// ==============================================================================
// ACTIONS
// ==============================================================================

#include <Arduino.h>

// Everything the operator can ask the robot to do. Hand waves (GestureUI) are the only way to
// start any of it.
namespace Actions {

// The four run modes. The number of hand waves that selects each one is its value + 1.
enum RunMode : uint8_t {
    MODE_SEARCH    = 0, // 1 wave  - Green   - explore the maze, then return to start
    MODE_HYBRID    = 1, // 2 waves - Yellow  - speed run, robot picks curves or diagonals
    MODE_DIAGONALS = 2, // 3 waves - Cyan    - speed run, diagonals only
    MODE_CURVES    = 3, // 4 waves - Magenta - speed run, smooth curves only
    MODE_COUNT     = 4
};

const char* modeName(RunMode mode);
StatusLED::Color modeColor(RunMode mode);

RunMode getSelectedMode();
void selectMode(RunMode mode);

// Show the selected mode's color on the LED (the LED's normal state)
void showSelectedMode();

// True while the robot is driving a search, return, or speed run
bool isRunActive();

// What the robot is doing right now, in words (for the debug console)
const char* stateDescription();

// Start the selected run. Does nothing if a run is already active.
// The robot must be in the start cell, facing into the maze.
void launchSelectedRun();

// Brake immediately and abandon the run
void stopRun();

// Learn the IR wall levels for this maze. Robot must sit centred in a cell between two side
// walls. Returns true on success. Blocks for about a second.
bool calibrateIR();

// Forget the maze saved in flash
void clearSavedMaze();

// Speed tier the next speed run will use: 1 (safest) to 3 (fastest). See SPEED_TIER_* in config.h.
uint8_t getSpeedTier();

// Call once when a run ends. Shows the result on the LED and moves the speed tier:
// up after a speed run that reached the centre, down after one that was aborted.
void onRunEnded(bool aborted);

} // namespace Actions

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

// ==============================================================================
// TEXT COMMAND CONSOLE
// ==============================================================================

#include <Arduino.h>

// Debug console over Bluetooth, Wi-Fi Telnet, or the USB serial monitor: for watching what the
// robot is doing and adjusting settings on the bench. It can stop a run but never start one;
// the competition build has no radios at all. Send "help" for the list.
namespace Console {

// Read and carry out any commands that have arrived. Call from the navigation task.
void poll();

} // namespace Console
