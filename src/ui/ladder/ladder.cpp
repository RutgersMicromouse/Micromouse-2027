#include <Preferences.h>
#include "robot.h"
#include "ui/status_led/status_led.h"
#include "ui/actions/actions.h"
#include "ui/turn_test/turn_test.h"
#include "ui/limits_test/limits_test.h"
#include "ui/ladder/ladder.h"

// ==============================================================================
// LADDER
// ==============================================================================

namespace Ladder {

struct Step {
    const char* name;
    uint8_t maze_size;   // Maze the step is driven in (0 = it does not matter)
    uint8_t move_set;    // What a speed run may use during this step, and from then on
};
static const Step kSteps[STEPS] = {
    { "turns on the spot",        0,  Navigator::MOVES_TURNS_ON_THE_SPOT },
    { "straights",                0,  Navigator::MOVES_TURNS_ON_THE_SPOT },
    { "curves in the 5x5 maze",   5,  Navigator::MOVES_CURVES },
    { "diagonals in the 5x5 maze", 5, Navigator::MOVES_DIAGONALS },
    { "everything in the 8x8 maze", 8, Navigator::MOVES_DIAGONALS },
    { "everything in the 16x16 maze", 16, Navigator::MOVES_DIAGONALS },
};

static uint8_t current_ = 0;          // The open step (STEPS = all finished)
static const uint8_t kTurnPercents[] = LADDER_TURN_PERCENTS;
static const float kHoldValues[] = LADDER_HOLD_VALUES;

static uint8_t best_[STEPS] = {};     // Fastest clean speed of each step, percent (step 1: of the tuned turn speed)
static uint8_t trial_ = 0;            // Speed of the run now being driven or judged, percent
static bool running_ = false;         // A checklist speed run is under way
static bool waiting_ = false;         // (kept for the app: a run is waiting to be judged; the robot now judges at once)
static int16_t undo_best_ = -1;       // Best speed before the robot's last "clean", so the owner can overrule it (-1 = nothing to undo)
static uint8_t undo_trial_ = 0;
static bool search_running_ = false;  // A search-speed trial is under way
static uint8_t search_trial_ = 0;     // Its speed, percent of the bring-up search speed (0 = none started yet)
static uint8_t search_best_ = 0;      // The fastest search speed judged clean so far in this session

// What the robot measured for itself during a checklist speed run. It knows its own width
// (ROBOT_WIDTH_MM) and where its side sensors sit on it, so a side reading becomes room
// between the body and the wall.
struct Judge {
    float side_room_mm;     // Least room between the body's side and a wall, while square to the maze
    float front_gap_mm;     // Least distance from the front sensor tips to a wall ahead
    float tilt_deg;         // Most it tipped: braking too hard dives the nose, pulling away too hard lifts it
    float post_guard;       // On diagonals: the most the post guard had to object (0 = never, 1 = fully alarmed)
    uint16_t brakings;      // The safety supervisor's count when the run began
};
static Judge judge_;

static void judgeStart() {
    judge_ = { 999.0f, 999.0f, 0.0f, 0.0f, g_safety.getStatus().brakings };
}

static void save() {
    Preferences prefs;
    prefs.begin("ladder", false);
    prefs.putUChar("cur", current_);
    prefs.putBytes("best", best_, sizeof(best_));
    prefs.end();
}

// What finished steps allow in real runs: the move set of the last finished step (the search
// curves too once curves are proven), and turns on the spot at the speed step 1 found
static void applyMoveSet() {
    const uint8_t allowed = (current_ == 0) ? (uint8_t)Navigator::MOVES_TURNS_ON_THE_SPOT : kSteps[current_ - 1].move_set;
    g_turn_speed_scale = (current_ > 0 && best_[0] >= 100) ? (float)best_[0] / 100.0f : 1.0f;
    NavigatorLock lock;
    g_navigator->setMoveSet(allowed);
    g_navigator->setSearchCurves(allowed >= Navigator::MOVES_CURVES);
}

void begin() {
    Preferences prefs;
    prefs.begin("ladder", true);
    current_ = min((int)prefs.getUChar("cur", 0), STEPS);
    prefs.getBytes("best", best_, sizeof(best_));
    prefs.end();
    applyMoveSet();
}

void sample() {
    if (!running_ && !search_running_) return;
    judge_.tilt_deg = fmaxf(judge_.tilt_deg, g_imu.getTiltDeg());
    // How far the robot points from the nearest maze axis decides which sensors mean what
    const float heading = g_imu.getHeadingDeg();
    const float off_axis = fabsf(normalizeAngle180(heading - roundf(heading / 90.0f) * 90.0f));
    float gap = 0.0f;
    if (off_axis < LADDER_SQUARE_DEG) {
        if (g_ir_sensors.getSideGapMm(true, gap))  judge_.side_room_mm = fminf(judge_.side_room_mm, gap - SIDE_BODY_BEYOND_SENSOR_MM);
        if (g_ir_sensors.getSideGapMm(false, gap)) judge_.side_room_mm = fminf(judge_.side_room_mm, gap - SIDE_BODY_BEYOND_SENSOR_MM);
        if (g_ir_sensors.getFrontGapMm(gap))       judge_.front_gap_mm = fminf(judge_.front_gap_mm, gap);
    } else if (fabsf(off_axis - 45.0f) < 10.0f) {
        // On a diagonal the 45 degree sensors look along the maze's axes at the posts going by
        judge_.post_guard = fmaxf(judge_.post_guard, fabsf(g_ir_sensors.getDiagonalGuardError()));
    }
}

// The robot's own verdict on the run just finished, with its reasons printed
static bool judgedClean() {
    const uint16_t brakings = g_safety.getStatus().brakings - judge_.brakings;
    const bool side_ok  = judge_.side_room_mm >= LADDER_MIN_SIDE_ROOM_MM;
    const bool front_ok = judge_.front_gap_mm >= LADDER_MIN_FRONT_GAP_MM;
    const bool tilt_ok  = judge_.tilt_deg <= LADDER_MAX_TILT_DEG;
    const bool post_ok  = judge_.post_guard <= LADDER_MAX_POST_GUARD;
    Serial.printf("[LADDER] Measured: least room at the sides %.0f mm (%s, needs %.0f) | nearest to a front wall %.0f mm (%s, needs %.0f) | "
                  "tipped %.1f deg (%s, at most %.0f) | post guard %.2f (%s, at most %.2f) | safety brakings %u (%s)\n",
                  judge_.side_room_mm, side_ok ? "ok" : "TOO CLOSE", (float)LADDER_MIN_SIDE_ROOM_MM,
                  judge_.front_gap_mm, front_ok ? "ok" : "TOO CLOSE", (float)LADDER_MIN_FRONT_GAP_MM,
                  judge_.tilt_deg, tilt_ok ? "ok" : "TOO MUCH", (float)LADDER_MAX_TILT_DEG,
                  judge_.post_guard, post_ok ? "ok" : "TOO CLOSE TO A POST", (float)LADDER_MAX_POST_GUARD,
                  (unsigned)brakings, brakings == 0 ? "ok" : "BRAKED");
    return side_ok && front_ok && tilt_ok && post_ok && brakings == 0;
}

void report() {
    static const char* const kMoves[3] = { "straights and turns on the spot", "smooth curves too", "curves and diagonals" };
    if (current_ >= STEPS) {
        Serial.printf("[LADDER] Every step is finished. Speed runs use %s, up to %d%%.\n",
                      kMoves[g_navigator->getMoveSet()], (int)Actions::getSpeedLimitPercent());
        return;
    }
    Serial.printf("[LADDER] Step %d of %d open: %s. Speed runs so far use %s, up to %d%%.%s\n",
                  current_ + 1, STEPS, kSteps[current_].name, kMoves[g_navigator->getMoveSet()], (int)Actions::getSpeedLimitPercent(),
                  waiting_ ? " WAITING: was the last run clean? (ladder clean / ladder fail)" : "");
}

static void finish(uint8_t best) {
    best_[current_] = best;
    Serial.printf("[LADDER] Step %d (%s) FINISHED at %d%%.\n", current_ + 1, kSteps[current_].name, (int)best);
    if (current_ >= 1) Actions::setSpeedLimitPercent(best); // From the straights on, each step's best is the ceiling (step 1's is the turn speed)
    current_++;
    trial_ = 0;
    waiting_ = running_ = false;
    save();
    applyMoveSet();
    StatusLED::flash(StatusLED::GREEN, 3, 150);
    report();
}

void run(int corridor_cells) {
    if (Actions::isRunActive()) { Serial.println("[LADDER] A run is under way."); return; }
    if (current_ >= STEPS) { report(); return; }
    if (waiting_) { Serial.println("[LADDER] First say whether the last run was clean: ladder clean / ladder fail."); return; }

    if (current_ == 0) {
        // Turns: the robot measures them itself, at the tuned speed first and then faster, and
        // keeps the fastest speed at which every turn was still on target
        uint8_t best = 0;
        float worst = -1.0f;

        // First, at the tuned speed: does a turn stay on the spot? If the middle of the axle
        // walks, the robot ends each turn a little further off the cell centre. Try stronger
        // values of t_hold until it stays put, and keep the one that walked least with every
        // turn still on target. (A value that makes the turns worse ends the search.)
        g_turn_speed_scale = 1.0f;
        const float hold_before = g_motion_controller.getTune(MotionController::TUNE_T_HOLD);
        float hold_best = hold_before, walk_best = 1e9f;
        for (float hold : kHoldValues) {
            if (hold < hold_before) continue; // (never weaker than what the team has set)
            g_motion_controller.setTune(MotionController::TUNE_T_HOLD, hold);
            worst = TurnTest::run(5);
            if (worst < 0.0f) break;
            const float walked = TurnTest::lastWalkedMm();
            const bool on_target = worst <= LADDER_TURN_TOLERANCE_DEG;
            Serial.printf("[LADDER] Staying on the spot, t_hold %.3f: a turn walks %.1f mm, turns %s\n", hold, walked, on_target ? "on target" : "NOT on target");
            if (on_target && walked < walk_best - 0.3f) { walk_best = walked; hold_best = hold; }
            if (!on_target || walked <= LADDER_WALK_OK_MM) break;
        }
        g_motion_controller.setTune(MotionController::TUNE_T_HOLD, hold_best);
        if (worst < 0.0f) { Serial.println("[LADDER] The turn test did not finish: step 1 stays open."); return; }
        if (hold_best != hold_before) {
            g_motion_controller.saveToNVS();
            Serial.printf("[LADDER] t_hold is now %.3f (it was %.3f): a turn walks %.1f mm. Saved with the rest of the tuning.\n", hold_best, hold_before, walk_best);
        }

        for (uint8_t percent : kTurnPercents) {
            g_turn_speed_scale = (float)percent / 100.0f;
            worst = TurnTest::run(5);
            const bool ok = worst >= 0.0f && worst <= LADDER_TURN_TOLERANCE_DEG;
            Serial.printf("[LADDER] Turns at %d%% of the tuned speed (%.0f deg/s): %s\n", (int)percent, (float)SEARCH_TURN_SPEED_DEG_S,
                          worst < 0.0f ? "the test did not finish" : ok ? "every turn on target" : "NOT on target");
            if (!ok) break;
            best = percent;
        }
        if (best > 0) { finish(best); return; }
        g_turn_speed_scale = 1.0f;
        if (worst < 0.0f) Serial.println("[LADDER] The turn test did not finish: step 1 stays open.");
        else Serial.printf("[LADDER] Step 1 NOT passed: the worst turn was %.1f degrees off, and %.1f is allowed. Tune the turns, then run it again.\n",
                           worst, (float)LADDER_TURN_TOLERANCE_DEG);
        return;
    }
    if (current_ == 1) {
        // Straights: the limits test finds the fastest level by itself
        const int best = LimitsTest::run(corridor_cells);
        if (best > 0) finish((uint8_t)best);
        else Serial.println("[LADDER] Step 2 NOT passed: no speed level was clean. Tune the straights, then run it again.");
        return;
    }

    // Steps 3 to 6: one speed run at the next speed to try, in the right maze
    const Step& step = kSteps[current_];
    if (MAZE_ACTIVE_SIZE != step.maze_size) {
        Serial.printf("[LADDER] This step is driven in the %dx%d maze: switch the maze size first.\n", (int)step.maze_size, (int)step.maze_size);
        return;
    }
    const uint8_t ceiling = Actions::getSpeedLimitPercent();
    if (trial_ == 0) trial_ = min((uint8_t)LADDER_START_PERCENT, ceiling);
    {
        NavigatorLock lock;
        g_navigator->setMoveSet(step.move_set); // (taken back by applyMoveSet if the step is not finished)
    }
    Serial.printf("[LADDER] Step %d (%s): a speed run at %d%%. Robot in the start cell, facing into the maze.\n",
                  current_ + 1, step.name, (int)trial_);
    running_ = true;
    judgeStart();
    Actions::selectMode(Actions::MODE_HYBRID);
    Actions::setNextSpeedRunPercent(trial_);
    Actions::launchSelectedRun();
    if (!Actions::isRunActive()) { // No route yet: the maze has to be searched first
        running_ = false;
        applyMoveSet();
        Serial.println("[LADDER] The speed run did not start. Search this maze first (the robot needs a known route).");
    }
}

void searchRun() {
    if (Actions::isRunActive()) { Serial.println("[LADDER] A run is under way."); return; }
    if (search_trial_ == 0) search_trial_ = Actions::getSearchSpeedPercent(); // The first trial proves the speed in force
    Actions::clearSavedMaze();
    Actions::setSearchSpeedPercent(search_trial_);
    Serial.printf("[LADDER] Search-speed trial at %d%%: the map of this maze is forgotten, one whole search follows. Robot in the start cell, facing in.\n", (int)search_trial_);
    search_running_ = true;
    judgeStart();
    Actions::selectMode(Actions::MODE_SEARCH);
    Actions::launchSelectedRun();
    if (!Actions::isRunActive()) { search_running_ = false; Serial.println("[LADDER] The search did not start."); }
}

static void searchEnded(bool finished) {
    search_running_ = false;
    const bool clean = finished && judgedClean();
    if (clean) {
        search_best_ = search_trial_;
        if (search_trial_ >= SEARCH_SPEED_MAX_PERCENT) {
            Serial.printf("[LADDER] Search clean at %d%%, the most there is: that is the search speed.\n", (int)search_trial_);
            return;
        }
        search_trial_ = (uint8_t)min((int)search_trial_ + LADDER_SEARCH_STEP_PERCENT, (int)SEARCH_SPEED_MAX_PERCENT);
        Serial.printf("[LADDER] Search CLEAN at %d%% (kept). Next: ladder search, at %d%%.\n", (int)search_best_, (int)search_trial_);
        StatusLED::flash(StatusLED::GREEN, 3, 150);
        return;
    }
    const uint8_t back = search_best_ ? search_best_ : (uint8_t)100;
    Serial.printf("[LADDER] Search %s at %d%%: stepping back to %d%%, which is the search speed from now on.\n",
                  finished ? "NOT CLEAN" : "did not finish", (int)search_trial_, (int)back);
    Actions::setSearchSpeedPercent(back);
    search_trial_ = 0;
    search_best_ = 0;
    StatusLED::flash(StatusLED::RED, 3, 150);
}

void onRunEnded(bool aborted, bool reached_goal) {
    if (search_running_) { searchEnded(!aborted && reached_goal); return; }
    if (!running_) return;
    running_ = false;
    if (aborted || !reached_goal) {
        applyMoveSet();
        Serial.printf("[LADDER] The run at %d%% did not reach the goal. If that was the speed: ladder fail. If it was something else (lifted, bad start): ladder run tries %d%% again.\n",
                      (int)trial_, (int)trial_);
        return;
    }
    // The robot judges the run itself, from what it measured on the way. The owner can still
    // overrule a "clean" with ladder fail (it did brush something the sensors missed).
    waiting_ = true;
    const bool clean = judgedClean();
    Serial.printf("[LADDER] The run at %d%% reached the goal and the robot calls it %s.%s\n", (int)trial_, clean ? "CLEAN" : "NOT CLEAN",
                  clean ? " (If you saw it touch something: ladder fail.)" : "");
    verdict(clean);
}

void verdict(bool clean) {
    // "Clean" needs a run that reached the goal; "not clean" may also follow one that was aborted
    if (!(waiting_ || (!clean && trial_ > 0))) { Serial.println("[LADDER] No run is waiting for a verdict."); return; }
    if (!clean && !waiting_ && undo_best_ >= 0) {
        // The owner overrules the robot's last "clean": back to how things stood before it
        best_[current_] = (uint8_t)undo_best_;
        trial_ = undo_trial_;
        Serial.printf("[LADDER] Overruled: the run at %d%% does not count as clean.\n", (int)trial_);
    }
    undo_best_ = -1;
    waiting_ = false;
    const uint8_t ceiling = Actions::getSpeedLimitPercent();
    if (clean) {
        undo_best_ = best_[current_];
        undo_trial_ = trial_;
        best_[current_] = trial_;
        save();
        if (trial_ >= ceiling) { finish(trial_); return; } // Nothing faster is allowed: this is its top speed
        trial_ = min((int)trial_ + LADDER_STEP_PERCENT, (int)ceiling);
        Serial.printf("[LADDER] Clean at %d%%. Next: ladder run, at %d%%.\n", (int)best_[current_], (int)trial_);
        return;
    }
    if (best_[current_] > 0) {
        Serial.printf("[LADDER] Not clean at %d%%: stepping back to %d%%, the last clean speed.\n", (int)trial_, (int)best_[current_]);
        finish(best_[current_]);
        return;
    }
    Serial.printf("[LADDER] Not clean at %d%%, the lowest speed tried. Step %d stays open: tune, then ladder run again.\n", (int)trial_, current_ + 1);
    trial_ = 0;
    applyMoveSet();
}

void reset() {
    current_ = 0;
    memset(best_, 0, sizeof(best_));
    trial_ = 0;
    waiting_ = running_ = false;
    save();
    applyMoveSet();
    Actions::setSpeedLimitPercent(100);
    Serial.println("[LADDER] Started over: step 1 open, speed runs and the search back to straights and turns on the spot at the tuned turn speed, ceiling removed.");
}

void statusJson(char* buf, size_t size) {
    snprintf(buf, size, "{\"cur\":%d,\"best\":[%d,%d,%d,%d,%d,%d],\"trial\":%d,\"wait\":%d}",
             (int)current_, (int)best_[0], (int)best_[1], (int)best_[2], (int)best_[3], (int)best_[4], (int)best_[5], (int)trial_, waiting_ ? 1 : 0);
}

} // namespace Ladder
