#pragma once

// The checklist the robot is taken through, one kind of move at a time, each at the fastest
// speed it can still do cleanly

#include <Arduino.h>

// ==============================================================================
// LADDER
// ==============================================================================

// Six steps, in order. A step is only open when the one before it is finished, and a finished
// step fixes what the robot is allowed to do in every later run:
//
//   1 Turns on the spot      the turn test, at the tuned turn speed and then faster (LADDER_TURN_PERCENTS);
//                            a speed passes if every turn ends within LADDER_TURN_TOLERANCE_DEG, and the
//                            fastest that passed becomes the speed of every turn on the spot
//   2 Straights              the limits test in a closed corridor; its best level becomes the speed ceiling
//   3 Curves, 5x5            speed runs with smooth curves in the 5x5 maze
//   4 Diagonals, 5x5         the same with diagonals and V turns
//   5 Everything, 8x8        the full move set in a half-size maze
//   6 Everything, 16x16      the full move set in the full maze
//
// The robot judges every step itself. Steps 3 to 6 are speed runs, each starting at
// LADDER_START_PERCENT of full speed. During one it keeps the least room it measured between
// its sides and the walls (it knows its own width and where its side sensors sit), the nearest
// it came to a wall ahead, the most it tipped under braking or acceleration, how hard the post
// guard objected on diagonals, and whether the safety supervisor braked; the run is CLEAN only
// if all of those are within the LADDER_* limits in config.h. What it cannot measure: a brush
// its sensors did not see (they look sideways and ahead, not at its corners), so the owner can
// still overrule a "clean" with "ladder fail". Clean = try
// LADDER_STEP_PERCENT faster. Not clean, or the run was aborted = the step is finished at the
// last speed that was clean ("a slight step back"), and that becomes the ceiling. Not clean at
// the very first speed = the step stays open: tune, then run it again. A step never goes faster
// than the ceiling the steps before it set.
//
// Until step 3 is finished, real speed runs use only straights and turns on the spot, and the
// search stops at every corner (Navigator::setSearchCurves); until step 4, no diagonals
// (Navigator::setMoveSet). So a hand-started speed run never uses a move
// that has not been proven here.
//
// Console (and the app's "Checklist" card):
//   ladder            where things stand
//   ladder run [N]    run the open step (N = corridor length in cells for step 2, default 4)
//   ladder clean      the speed run just driven was clean
//   ladder fail       it was not
//   ladder reset      start over from step 1 (back to turns on the spot only, ceiling removed)
//
// Everything is kept in flash and printed as [LADDER] lines.
namespace Ladder {

static const int STEPS = 6;

void begin();                         // Loads the saved place and applies the move set it allows

void report();
void run(int corridor_cells);         // Blocks for steps 1 and 2; for 3 to 6 it starts a speed run and returns
void verdict(bool clean);             // The owner's word on the speed run just driven
void reset();

// Call often while a run is on (the operator task does): gathers what the robot uses to judge a
// checklist speed run. Does nothing unless such a run is under way.
void sample();

// Called by Actions when a run ends: a checklist run that was aborted is "not clean" without asking
void onRunEnded(bool aborted, bool reached_goal);

// For the app: {"cur":2,"best":[150,60,0,0,0,0],"trial":30,"wait":0}
void statusJson(char* buf, size_t size);

} // namespace Ladder
