#pragma once

// Finds how hard the robot can be driven and still stop and turn where it should

#include <Arduino.h>

// ==============================================================================
// LIMITS TEST
// ==============================================================================

// Console: "limits 4" for a corridor 4 cells long ("limits clear" removes the ceiling it set).
//
// Set-up: a straight corridor of that many cells, CLOSED AT BOTH ENDS, the robot in the middle of
// one end cell facing along it, sensors calibrated (the front level measured against a wall).
//
// It shuttles from one end to the other and back, each pair of runs harder than the last
// (LIMITS_STEPS, as a percentage of the speed run's speed and acceleration). Each run is driven
// as a speed run drives a straight: on the wheels' distance alone, no stop at the front wall.
// At the far end the wall itself says how far from the cell centre the robot stopped; the robot
// then steps onto the centre, turns round, and that turn is measured too.
//
// A level passes if both runs stop within LIMITS_STOP_TOLERANCE_MM of the centre, end within
// LIMITS_HEADING_TOLERANCE_DEG of straight, turn round within LIMITS_TURN_TOLERANCE_DEG, and the
// safety supervisor did not have to brake. At the first level that fails it stops, and the last
// level that passed becomes the ceiling for every speed run (Actions::setSpeedLimitPercent, kept
// in flash). One [LIMITS] line per run, then "[LIMITS] done".
//
// It blocks the caller (the operator task). A STOP, or lifting the robot, ends it at once.
namespace LimitsTest {

// Returns the best level that passed (percent), 0 if none did, -1 if it was stopped by hand.
int run(int cells);

// Loads the braking strength measured earlier, if any (call once at start-up)
void begin();

// Console "limits brake": on 1.2 m of clear floor, gets up to 40, 70 and 100 % of the speed
// run's speed and brakes as the safety supervisor would, each time. From the three stopping
// distances it works out the delay before the brake bites and the braking itself, and those
// (with a fifth of margin on each) replace the guessed SAFETY_LATENCY_S and
// SAFETY_BRAKE_DECEL_MM_S2 in the safety supervisor's stopping-distance sum, and are saved.
// Prints [BRAKE] lines. Blocks.
void measureBraking();

} // namespace LimitsTest
