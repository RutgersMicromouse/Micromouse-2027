#pragma once

// A batch of turns on the spot, measured, for tuning the turns

#include <Arduino.h>

// ==============================================================================
// TURN TEST
// ==============================================================================

// The app's "Turn tuning test" button (console: "turntest", or "turntest 8" for 8 pairs).
// The robot stands on the floor and turns 90 degrees left, then 90 degrees right, on the spot,
// `pairs` times. After each turn it prints a [TURN] line: which way, where it stopped, where it
// was a moment later, how far past (+) or short of (-) the target that is, the fastest it turned,
// and how long it took. At the end one [TURNTEST] line sums it up for each direction, with the
// battery, the IMU's failed reads and every turn setting that was in force, so that whoever reads
// the recording can see what a change of setting did.
//
// It blocks the caller (the operator task) for about 1.5 s a turn. A STOP ends it at once.
namespace TurnTest {

// fast = at the speed-run turn speed and acceleration (SPEEDRUN_TURN_*) instead of the search's
// Returns how far off 90 degrees the worst turn ended (degrees, never negative), or -1 if the
// test was stopped or gave up before every turn was done.
float run(int pairs, bool fast = false);

// From the last run: how far a turn moved the middle of the axle off the spot, on average, in mm
// (by the wheels: half the sum of their two distances). 0 is right.
float lastWalkedMm();

} // namespace TurnTest
