#pragma once

// A measured straight through a few cells, for tuning how the robot drives straight (stage E)

#include <Arduino.h>

// ==============================================================================
// STRAIGHT TEST
// ==============================================================================

// The app's "Straight test" buttons (console: "straighttest 3" for 3 cells). The robot stands in
// the middle of a cell facing along the corridor to be driven, with the IR sensors calibrated. It
// drives that many cells exactly as the search does (same move, same speed, wall centring on,
// stopping at the cell centre or at a wall in front) and measures 50 times a second:
//
//   wobble   how far the heading wandered from where it started (mean, spread, most), and how
//            many times it swung from one side to the other
//   walls    which side walls it saw over which millimetres, so whoever reads the recording can
//            see which wall arrangement this run was without being told
//   centre   the wall-centring error (what the steering was working against)
//   end      how far each wheel went, the heading it stopped at, and, with a wall in front, how
//            far from the cell centre the front sensors put it
//
// One [STRAIGHT] line per stretch of walls, then a [STRAIGHTTEST] done line with everything and
// the settings in force. It blocks the caller (the operator task). A STOP ends it at once.
namespace StraightTest {

void run(int cells);

} // namespace StraightTest
