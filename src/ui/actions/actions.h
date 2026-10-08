#pragma once

// Everything the operator can ask for: select a mode, launch, stop, calibrate...

#include "types.h"

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
void launchSelectedRun(bool one_move_at_a_time = false); // true = search that waits before every move

// After a speed run has ended in the goal cell: drive back to the start cell and face into the
// maze. Returns false if no speed run has just finished (the robot's position is then not known).
bool returnToStart();

// Brake immediately and abandon the run
void stopRun();

// Learn the IR wall levels for this maze. Robot must sit centred in a cell between two side
// walls. Returns true on success. Blocks for about a second.
bool calibrateIR();

// Forget the maze saved in flash
void clearSavedMaze();

// Speed tier the next speed run will use: 1 (safest) to 3 (fastest). See SPEED_TIER_* in config.h.
uint8_t getSpeedTier();

// Speed chosen by hand for the NEXT speed run only, as a percentage (10..100) of the SPEEDRUN_*
// speeds and accelerations, instead of the tier. For trying speed levels from the phone app
// without re-flashing. 0 = use the tier as usual.
void setNextSpeedRunPercent(uint8_t percent);

// Call once when a run ends. Shows the result on the LED and moves the speed tier:
// up after a speed run that reached the centre, down after one that was aborted.
void onRunEnded(bool aborted);

} // namespace Actions
