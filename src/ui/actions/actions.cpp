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
    Serial.println("\n[UI] STOP! Halting robot.");
    {
        NavigatorLock lock;
        g_navigator->stop();
    }
    StatusLED::flash(StatusLED::RED, 3, 100);
    showSelectedMode();
}

bool calibrateIR() {
    if (isRunActive()) return false;

    StatusLED::set(StatusLED::YELLOW);
    bool ok = g_ir_sensors.calibrateInCell(200);
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

uint8_t getSpeedTier() {
    return s_speed_tier;
}

void setNextSpeedRunPercent(uint8_t percent) {
    s_next_run_percent = (percent == 0) ? 0 : constrain(percent, (uint8_t)10, (uint8_t)100);
}

void onRunEnded(bool aborted) {
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
