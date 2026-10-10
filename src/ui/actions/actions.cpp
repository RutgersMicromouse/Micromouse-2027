#include <functional>
#include "robot.h"
#include "ui/status_led/status_led.h"
#include "ui/actions/actions.h"
#include "ui/gestures/gestures.h"
#include "ui/console/console.h"
#include "wireless/ble_debug/ble_debug.h"
#include "wireless/wifi_ota/wifi_ota.h"

// ==============================================================================
// ACTIONS
// ==============================================================================

namespace Actions {

static RunMode s_selected_mode = MODE_SEARCH;

static const float kSpeedTierScale[3] = { SPEED_TIER_1_SCALE, SPEED_TIER_2_SCALE, SPEED_TIER_3_SCALE };
static uint8_t s_speed_tier = 1; // 1..3
static uint8_t s_next_run_percent = 0; // Hand-picked speed for the next speed run; 0 = use the tier

const char* modeName(RunMode mode) {
    switch (mode) {
        case MODE_SEARCH:    return "SEARCH";
        case MODE_HYBRID:    return "SPEED RUN - HYBRID";
        case MODE_DIAGONALS: return "SPEED RUN - DIAGONALS";
        case MODE_CURVES:    return "SPEED RUN - CURVES";
        default:             return "UNKNOWN";
    }
}

StatusLED::Color modeColor(RunMode mode) {
    switch (mode) {
        case MODE_SEARCH:    return StatusLED::GREEN;
        case MODE_HYBRID:    return StatusLED::YELLOW;
        case MODE_DIAGONALS: return StatusLED::CYAN;
        case MODE_CURVES:    return StatusLED::MAGENTA;
        default:             return StatusLED::BLUE;
    }
}

RunMode getSelectedMode() {
    return s_selected_mode;
}

void selectMode(RunMode mode) {
    if (mode >= MODE_COUNT) return;
    s_selected_mode = mode;
    Serial.printf("[UI] Selected mode %d: %s\n", (int)mode, modeName(mode));
    showSelectedMode();
}

void showSelectedMode() {
    StatusLED::set(modeColor(s_selected_mode));
}

bool isRunActive() {
    NavState state = g_navigator->getState();
    return state == NAV_STATE_EXPLORING_TO_CENTER ||
           state == NAV_STATE_RETURNING_TO_START ||
           state == NAV_STATE_SPEED_RUNNING;
}

const char* stateDescription() {
    if (g_navigator->isWaitingForNextMove()) return "SEARCHING one move at a time - waiting for 'Drive one cell'";
    switch (g_navigator->getState()) {
        case NAV_STATE_IDLE:                 return "IDLE - ready to start";
        case NAV_STATE_CALIBRATING:          return "CALIBRATING the IR sensors";
        case NAV_STATE_EXPLORING_TO_CENTER:  return "SEARCHING - driving to the goal";
        case NAV_STATE_RETURNING_TO_START:   return "SEARCHING - exploring back to the start cell";
        case NAV_STATE_PREPARING_SPEED_RUN:  return "SEARCH DONE - back at the start, ready for the next run";
        case NAV_STATE_SPEED_RUNNING:        return "SPEED RUN in progress";
        case NAV_STATE_FINISHED:             return "SPEED RUN DONE - at the goal";
        case NAV_STATE_ERROR:                return "ERROR - no route found (forget the maze and try again)";
        default:                             return "UNKNOWN";
    }
}

void launchSelectedRun(bool one_move_at_a_time) {
    if (isRunActive()) return;

    Serial.printf("[UI] Launching: %s\n", modeName(s_selected_mode));
    showSelectedMode();

    // The robot was placed by hand: forget where the last run left its heading and distance
    prepareForNewRun();

    NavigatorLock lock; // The navigation task takes over from the first move onwards
    if (s_selected_mode != MODE_SEARCH) {
        if (s_next_run_percent > 0) {
            Serial.printf("[UI] Speed chosen by hand: %d%% (the tier is not used for this run)\n", (int)s_next_run_percent);
            g_navigator->setSpeedScale((float)s_next_run_percent / 100.0f);
            s_next_run_percent = 0;
        } else {
            Serial.printf("[UI] Speed tier %d of 3 (%.0f%% speed)\n", (int)s_speed_tier, kSpeedTierScale[s_speed_tier - 1] * 100.0f);
            g_navigator->setSpeedScale(kSpeedTierScale[s_speed_tier - 1]);
        }
    }

    switch (s_selected_mode) {
        case MODE_SEARCH: {
            g_navigator->startSearchRun(one_move_at_a_time);
            // The search is driven by wall readings, so take the first step right away
            RobotTelemetry telemetry = {};
            getTelemetry(telemetry);
            g_navigator->step(telemetry.ir);
            break;
        }
        case MODE_HYBRID:    g_navigator->startSpeedRun(SPEEDRUN_HYBRID_AUTO);    break;
        case MODE_DIAGONALS: g_navigator->startSpeedRun(SPEEDRUN_DIAGONALS_ONLY); break;
        case MODE_CURVES:    g_navigator->startSpeedRun(SPEEDRUN_CURVES_ONLY);    break;
        default: break;
    }
}

bool returnToStart() {
    // The robot has not been touched since the speed run, so its heading and distance tracking
    // carry on from where that run ended (no prepareForNewRun() here)
    RobotTelemetry telemetry = {};
    getTelemetry(telemetry);

    NavigatorLock lock;
    if (!g_navigator->startReturnToStart()) return false;
    Serial.println("[UI] Returning to the start cell.");
    g_navigator->step(telemetry.ir);
    return true;
}

void stopRun() {
    requestStop(); // Brakes on the next control tick, whatever else is going on
    Serial.println("\n[UI] STOP! Halting robot.");
    {
        NavigatorLock lock;
        g_navigator->stop();
    }
    StatusLED::flash(StatusLED::RED, 3, 100);
    showSelectedMode();
}

bool calibrateIR(bool keep_side_centre) {
    if (isRunActive()) return false;

    StatusLED::set(StatusLED::YELLOW);
    bool ok = g_ir_sensors.calibrateInCell(200, keep_side_centre);
    StatusLED::flash(ok ? StatusLED::GREEN : StatusLED::RED, 3, 120);
    showSelectedMode();
    return ok;
}

void clearSavedMaze() {
    if (isRunActive()) return;

    {
        NavigatorLock lock;
        g_navigator->clearSavedMaze();
    }
    StatusLED::flash(StatusLED::BLUE, 4, 100);
    showSelectedMode();
}

bool setMazeSize(uint8_t size) {
#ifdef MAZE_SIZE_SWITCHABLE
    if (isRunActive() || (size != 3 && size != 5 && size != 16)) return false;
    {
        NavigatorLock lock;
        g_navigator->changeMazeSize(size);
    }
    StatusLED::flash(StatusLED::BLUE, 2, 100);
    showSelectedMode();
    return true;
#else
    (void)size;
    return false; // This build is fixed at one size
#endif
}

uint8_t getSpeedTier() {
    return s_speed_tier;
}

void setNextSpeedRunPercent(uint8_t percent) {
    s_next_run_percent = (percent == 0) ? 0 : constrain(percent, (uint8_t)10, (uint8_t)100);
}

// The cell paths of the turn test, each starting facing north. Left-hand shapes start at x = 3 and
// right-hand ones at x = 0 only so that every cell number stays positive; the planner works from
// the directions between the cells, not from where they are in the maze.
struct CurveShape {
    const char* name;
    bool diagonals;
    uint8_t cells;
    Coordinate path[7];
    const char* what;
};
static const CurveShape kCurveShapes[] = {
    { "right",    false, 3, { {0,0}, {0,1}, {1,1} },                         "one smooth 90 degree curve to the RIGHT (3 cells in an L)" },
    { "left",     false, 3, { {3,0}, {3,1}, {2,1} },                         "one smooth 90 degree curve to the LEFT (3 cells in an L)" },
    { "uright",   false, 4, { {0,0}, {0,1}, {1,1}, {1,0} },                  "a U-turn to the RIGHT: two 90 degree curves in a row (2 x 2 cells)" },
    { "uleft",    false, 4, { {3,0}, {3,1}, {2,1}, {2,0} },                  "a U-turn to the LEFT: two 90 degree curves in a row (2 x 2 cells)" },
    { "zigright", true,  4, { {0,0}, {0,1}, {1,1}, {1,2} },                  "a diagonal: right then left, 45 in and 45 out (staircase of 4 cells)" },
    { "zigleft",  true,  4, { {3,0}, {3,1}, {2,1}, {2,2} },                  "a diagonal: left then right, 45 in and 45 out (staircase of 4 cells)" },
    { "veeright", true,  6, { {0,0}, {0,1}, {1,1}, {1,2}, {0,2}, {0,3} },    "a diagonal with a V turn: right, left, left, right (6 cells)" },
    { "veeleft",  true,  6, { {3,0}, {3,1}, {2,1}, {2,2}, {3,2}, {3,3} },    "a diagonal with a V turn: left, right, right, left (6 cells)" },
};
static bool s_test_run = false; // The run now on (or just ended) is a turn test, not a speed run

bool launchCurveTest(const char* shape, uint8_t percent) {
    if (isRunActive()) return false;
    const CurveShape* chosen = nullptr;
    for (const CurveShape& candidate : kCurveShapes) {
        if (strcmp(candidate.name, shape) == 0) chosen = &candidate;
    }
    if (chosen == nullptr) return false;
    percent = constrain(percent, 10, 100);

    Serial.printf("[CURVETEST] Starting '%s' at %d%% speed: %s.\n", chosen->name, (int)percent, chosen->what);
    // The robot was placed by hand: forget where the last run left its heading and distance
    prepareForNewRun();

    NavigatorLock lock; // The navigation task takes over from the first move onwards
    g_navigator->setSpeedScale((float)percent / 100.0f);
    s_test_run = true;
    if (!g_navigator->startPathTest(chosen->path, chosen->cells, chosen->diagonals)) {
        s_test_run = false;
        Serial.println("[CURVETEST] Could not plan that shape.");
        return false;
    }
    return true;
}

void onRunEnded(bool aborted) {
    if (s_test_run) {
        // A turn test is not a speed run: it leaves the speed tier alone
        s_test_run = false;
        const MotionController& mc = g_motion_controller;
        Serial.printf("[CURVETEST] %s | heading at the end %.1f | supply %.1f V | h_kp=%.4g h_ki=%.4g c_kp=%.4g c_ff=%.4g c_ka=%.4g turn_ff=%.4g ff_kv=%.4g ff_ka=%.4g v_kp=%.4g k_sync=%.4g\n",
                      aborted ? "ABORTED" : "done", g_imu.getHeadingDeg(), g_motors.getSupplyVolts(),
                      mc.getTune(MotionController::TUNE_H_KP), mc.getTune(MotionController::TUNE_H_KI),
                      mc.getTune(MotionController::TUNE_C_KP), mc.getTune(MotionController::TUNE_C_FF),
                      mc.getTune(MotionController::TUNE_C_KA),
                      mc.getTune(MotionController::TUNE_TURN_FF),
                      mc.getTune(MotionController::TUNE_FF_KV), mc.getTune(MotionController::TUNE_FF_KA),
                      mc.getTune(MotionController::TUNE_V_KP), mc.getTune(MotionController::TUNE_K_SYNC));
        StatusLED::flash(aborted ? StatusLED::RED : StatusLED::GREEN, 2, 150);
        showSelectedMode();
        return;
    }
    const NavState state = g_navigator->getState();
    const bool was_speed_run = (s_selected_mode != MODE_SEARCH);

    if (aborted) {
        if (was_speed_run && s_speed_tier > 1) {
            s_speed_tier--;
            Serial.printf("[UI] Speed run aborted: next one drops to tier %d.\n", (int)s_speed_tier);
        }
        StatusLED::flash(StatusLED::RED, 5, 100);
    } else if (state == NAV_STATE_FINISHED) {
        if (s_speed_tier < 3) s_speed_tier++;
        Serial.printf("[UI] Speed run finished: next one uses tier %d.\n", (int)s_speed_tier);
        StatusLED::flash(StatusLED::GREEN, 3, 150);
    } else if (state == NAV_STATE_PREPARING_SPEED_RUN) {
        // Search finished. Green = the best route is fully explored; yellow = searching again
        // might find a shorter one.
        StatusLED::flash(g_navigator->isBestRouteExplored() ? StatusLED::GREEN : StatusLED::YELLOW, 2, 300);
    } else if (state == NAV_STATE_ERROR) {
        StatusLED::flash(StatusLED::RED, 2, 400);
    }
    showSelectedMode();
}

} // namespace Actions
