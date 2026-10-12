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
//   1 Turns on the spot      first it finds the t_hold that keeps a turn on the spot (LADDER_HOLD_VALUES) and
//                            SAVES it with the tuning; then
//                            the turn test, at the tuned turn speed and then faster (LADDER_TURN_PERCENTS);
//                            a speed passes if every turn ends within LADDER_TURN_TOLERANCE_DEG, and the
//                            fastest that passed becomes the speed of every turn on the spot
//   2 Straights              the limits test in a closed corridor; its best level becomes the speed ceiling
//   (step 3 goes on after its speed reaches the ceiling: it then lifts the curve speed cap,
//    LADDER_CURVE_CAP_STEP_MM_S at a time, for as long as runs stay clean)
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
//   ladder search     one search at the next search speed to try (see searchRun)
//
// Everything is kept in flash and printed as [LADDER] lines.
namespace Ladder {

static const int STEPS = 6;

void begin();                         // Loads the saved place and applies the move set it allows

void report();
void run(int corridor_cells);         // Blocks for steps 1 and 2; for 3 to 6 it starts a speed run and returns
void verdict(bool clean);             // The owner's word on the speed run just driven
void reset();

// The search's speed, found the same way as the steps' speeds but not a numbered step, since it
// can be done in any maze at any time. Each call FORGETS THE SAVED MAP OF THE MAZE SIZE IN USE
// (a search over known ground is not the same test) and drives one whole search, out and back,
// judged like a checklist speed run. Clean, and back in the start cell: that speed is kept and
// the next call tries LADDER_SEARCH_STEP_PERCENT more. Not clean, or it did not finish: back to
// the last speed that was clean (or the bring-up speed), which is then the search speed.
void searchRun();

// Call often while a run is on (the operator task does): gathers what the robot uses to judge a
// checklist speed run. Does nothing unless such a run is under way.
void sample();

// Called by Actions when a run ends: a checklist run that was aborted is "not clean" without asking
// (also when a search-speed trial ends: `reached_goal` then means the search finished back in the start cell)
void onRunEnded(bool aborted, bool reached_goal);

// For the app: {"cur":2,"best":[150,60,0,0,0,0],"trial":30,"wait":0}
void statusJson(char* buf, size_t size);

} // namespace Ladder
