#include <functional>
#include "ui.h"
#include "robot.h"
#include "wireless.h"

// ==============================================================================
// STATUS LED
// ==============================================================================

namespace StatusLED {

void begin() {
    pinMode(PIN_ESP32_RGB_LED, OUTPUT);
    set(OFF);
}

void set(Color color) {
    const uint8_t level = RGB_BRIGHTNESS_LEVEL;
    neopixelWrite(PIN_ESP32_RGB_LED,
                  color.red   ? level : 0,
                  color.green ? level : 0,
                  color.blue  ? level : 0);
}

void flash(Color color, int count, int delay_ms) {
    for (int i = 0; i < count; ++i) {
        set(color);
        delay(delay_ms);
        set(OFF);
        delay(delay_ms);
    }
}

} // namespace StatusLED

// ==============================================================================
// ACTIONS
// ==============================================================================

namespace Actions {

static RunMode s_selected_mode = MODE_SEARCH;

static const float kSpeedTierScale[3] = { SPEED_TIER_1_SCALE, SPEED_TIER_2_SCALE, SPEED_TIER_3_SCALE };
static uint8_t s_speed_tier = 1; // 1..3

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

void launchSelectedRun() {
    if (isRunActive()) return;

    Serial.printf("[UI] Launching: %s\n", modeName(s_selected_mode));
    showSelectedMode();

    // The robot was placed by hand: forget where the last run left its heading and distance
    prepareForNewRun();

    if (s_selected_mode != MODE_SEARCH) {
        Serial.printf("[UI] Speed tier %d of 3 (%.0f%% speed)\n", (int)s_speed_tier, kSpeedTierScale[s_speed_tier - 1] * 100.0f);
        g_navigator->setSpeedScale(kSpeedTierScale[s_speed_tier - 1]);
    }

    switch (s_selected_mode) {
        case MODE_SEARCH: {
            g_navigator->startSearchRun();
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

void stopRun() {
    Serial.println("\n[UI] STOP! Halting robot.");
    g_navigator->stop();
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

    g_navigator->clearSavedMaze();
    StatusLED::flash(StatusLED::BLUE, 4, 100);
    showSelectedMode();
}

uint8_t getSpeedTier() {
    return s_speed_tier;
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

// ==============================================================================
// HAND-WAVE DETECTOR
// ==============================================================================

// Samples in a row needed before believing the hand arrived / left (2 samples = ~40 ms)
static const uint8_t DEBOUNCE_SAMPLES = 2;

GestureInput::GestureInput() {
    reset(0);
}

void GestureInput::reset(uint32_t now_ms) {
    resting_level_ = -1.0f; // Not learned yet
    hand_present_ = false;
    confirm_samples_ = 0;
    wave_count_ = 0;
    wave_counted_flag_ = false;
    started_ms_ = now_ms;
    hand_since_ms_ = now_ms;
    last_wave_end_ms_ = now_ms;
}

uint8_t GestureInput::update(uint16_t front_reading, uint32_t now_ms) {
    float reading = (float)front_reading;

    // Warm-up: just learn the resting level, so nothing that happens while the robot is being
    // put down can count as a wave
    if (resting_level_ < 0.0f) resting_level_ = reading;
    if (now_ms - started_ms_ < GESTURE_WARMUP_MS) {
        resting_level_ += (reading - resting_level_) * 0.2f;
        return 0;
    }

    // How far above the resting level counts as "a hand is there" (with hysteresis on the way out)
    float rise = resting_level_ * GESTURE_RISE_RATIO;
    if (rise < (float)GESTURE_MIN_RISE) rise = (float)GESTURE_MIN_RISE;
    float arrive_level = resting_level_ + rise;
    float leave_level  = resting_level_ + rise * 0.5f;

    if (!hand_present_) {
        if (reading > arrive_level) {
            if (++confirm_samples_ >= DEBOUNCE_SAMPLES) {
                hand_present_ = true;
                hand_since_ms_ = now_ms;
                confirm_samples_ = 0;
            }
        } else {
            confirm_samples_ = 0;
            resting_level_ += (reading - resting_level_) * 0.05f; // Slowly follow ambient changes
        }

        // The operator has stopped waving: report the total
        if (wave_count_ > 0 && !hand_present_ && (now_ms - last_wave_end_ms_ > GESTURE_SEQUENCE_GAP_MS)) {
            uint8_t total = wave_count_;
            wave_count_ = 0;
            return total;
        }
        return 0;
    }

    // A hand is currently in front of the sensors
    if (reading < leave_level) {
        if (++confirm_samples_ >= DEBOUNCE_SAMPLES) {
            hand_present_ = false;
            confirm_samples_ = 0;
            last_wave_end_ms_ = now_ms;

            if (now_ms - hand_since_ms_ <= GESTURE_MAX_WAVE_MS) {
                wave_count_++;
                wave_counted_flag_ = true;
            } else {
                wave_count_ = 0; // Holding a hand there is the "never mind" gesture
            }
        }
    } else {
        confirm_samples_ = 0;
        if (now_ms - hand_since_ms_ > GESTURE_REBASE_MS) {
            // Not a hand: the robot was moved and is now looking at something closer
            resting_level_ = reading;
            hand_present_ = false;
            wave_count_ = 0;
            last_wave_end_ms_ = now_ms;
        }
    }
    return 0;
}

bool GestureInput::consumeWaveCounted() {
    bool counted = wave_counted_flag_;
    wave_counted_flag_ = false;
    return counted;
}

// ==============================================================================
// HAND-WAVE CONTROLS
// ==============================================================================

namespace GestureUI {

static const uint8_t WAVES_CALIBRATE_IR = 5;
static const uint8_t WAVES_CLEAR_MAZE   = 6;

static GestureInput s_input;

void restart() {
    s_input.reset(millis());
}

// Rapid-blink countdown. Returns false if the operator covered the sensors to cancel.
static bool countdown(StatusLED::Color color, uint32_t duration_ms) {
    uint32_t start_ms = millis();
    while (millis() - start_ms < duration_ms) {
        bool led_on = ((millis() - start_ms) / 100) % 2 == 0;
        StatusLED::set(led_on ? color : StatusLED::OFF);

        RobotTelemetry telemetry;
        if (getTelemetry(telemetry)) {
            s_input.update(telemetry.ir.front_center, millis());
        }
        if (s_input.isHandPresent()) {
            Serial.println("[UI] Cancelled by hand.");
            StatusLED::flash(StatusLED::RED, 2, 100);
            return false;
        }
        delay(20);
    }
    return true;
}

static void perform(uint8_t waves) {
    Serial.printf("[UI] Counted %d hand wave(s).\n", waves);

    if (waves >= 1 && waves <= Actions::MODE_COUNT) {
        Actions::RunMode mode = (Actions::RunMode)(waves - 1);
        Actions::selectMode(mode);
        StatusLED::flash(Actions::modeColor(mode), waves, 150);
        if (mode != Actions::MODE_SEARCH) {
            // Speed tier the run will use: 1, 2, or 3 blue blinks
            delay(300);
            StatusLED::flash(StatusLED::BLUE, Actions::getSpeedTier(), 150);
        }
        if (countdown(Actions::modeColor(mode), GESTURE_LAUNCH_DELAY_MS)) {
            Actions::launchSelectedRun();
        }
    } else if (waves == WAVES_CALIBRATE_IR) {
        StatusLED::flash(StatusLED::YELLOW, waves, 150);
        if (countdown(StatusLED::YELLOW, GESTURE_LAUNCH_DELAY_MS)) {
            Actions::calibrateIR();
        }
    } else if (waves == WAVES_CLEAR_MAZE) {
        Actions::clearSavedMaze();
    } else {
        StatusLED::flash(StatusLED::RED, 2, 100); // No action for that many waves
    }

    Actions::showSelectedMode();
    restart();
}

void update(const IRReadings& ir) {
    uint8_t waves = s_input.update(ir.front_center, millis());

    if (s_input.consumeWaveCounted()) {
        // Blink white so the operator can see each wave being counted
        StatusLED::set(StatusLED::WHITE);
        delay(60);
        Actions::showSelectedMode();
    }

    if (waves > 0) {
        perform(waves);
    }
}

} // namespace GestureUI

// ==============================================================================
// TEXT COMMAND CONSOLE
// ==============================================================================

#ifndef ENABLE_CALIBRATION
#define ENABLE_CALIBRATION 0 // Set to 1 only by the `calibration` build in platformio.ini
#endif
#if ENABLE_CALIBRATION
// Extra commands of the calibration build: motorcal, motorrpm [duty], dyno ... (bench/calibration.cpp)
namespace Calibration {
bool handleCommand(const String& cmd, std::function<void(const char*)> reply);
}
#endif

namespace Console {

// Where a command came from, so the answer goes back the same way
enum Source { SOURCE_USB, SOURCE_BLE, SOURCE_TELNET };

static Source s_source = SOURCE_USB;

static void reply(const char* msg) {
    Serial.println(msg);
#if ENABLE_BLE_DEBUG
    if (s_source == SOURCE_BLE) BLEDebug::println(msg);
#endif
#if ENABLE_WIFI_OTA
    if (s_source == SOURCE_TELNET) WifiOTA::println(msg);
#endif
}

// Parses the two numbers after a command name, e.g. "motortrim 0.98 1.0"
static bool parseTwoFloats(const String& cmd, float& a, float& b) {
    int space_idx = cmd.indexOf(' ');
    return space_idx > 0 && sscanf(cmd.c_str() + space_idx + 1, "%f %f", &a, &b) == 2;
}

static bool parseTwoFlags(const String& cmd, bool& a, bool& b) {
    float fa = 0.0f, fb = 0.0f;
    if (!parseTwoFloats(cmd, fa, fb)) return false;
    a = (fa != 0.0f);
    b = (fb != 0.0f);
    return true;
}

static void handle(String cmd, Source source) {
    cmd.toLowerCase();
    cmd.trim();
    if (cmd.length() == 0) return;

    s_source = source;
    Serial.printf("[CMD] Received: '%s'\n", cmd.c_str());

    const bool running = Actions::isRunActive();
    char buf[200];

    // ---------------------------------------------------------------- Safety
    // The console can stop the robot but never start it: runs, mode selection, IR calibration,
    // and clearing the maze are hand-wave only (GestureUI), exactly as at a competition.
    if (cmd == "stop" || cmd == "estop" || cmd == "halt") {
        Actions::stopRun();
        reply("ACK: STOPPED");
        return;
    }

    // ---------------------------------------------------------------- Debug helpers
    if (cmd == "resetenc" || cmd == "resetheading") {
        // Sent by the web dashboard (tools/web_dashboard) "Reset" buttons
        if (running) { reply("ERR: CANNOT_RESET_WHILE_RUNNING"); return; }
        if (cmd == "resetenc") {
            requestEncoderReset();
            reply("ACK: ENCODERS RESET");
        } else {
            requestHeadingReset();
            reply("ACK: HEADING RESET");
        }
        return;
    }

    // ---------------------------------------------------------------- Reports
    if (cmd == "status") {
        RobotTelemetry telemetry = {};
        getTelemetry(telemetry);
        snprintf(buf, sizeof(buf), "STATUS: VBat=%.2fV | Mode=%d | State=%d",
                 telemetry.vbat_volts, (int)Actions::getSelectedMode(), (int)g_navigator->getState());
        reply(buf);
        return;
    }
    if (cmd == "health") {
        RobotTelemetry telemetry = {};
        getTelemetry(telemetry);
        snprintf(buf, sizeof(buf),
                 "HEALTH: Motoron=%s (supply=%.1fV, flags=0x%04X, resets recovered=%u, bus faults=%u) | IMU=%s (dropouts=%u%s) | Encoder faults=%u | Loop overruns=%lu",
                 g_motors.isConnected() ? "OK" : "NOT RESPONDING",
                 g_motors.getSupplyVolts(),
                 (unsigned int)g_motors.getStatusFlags(),
                 (unsigned int)g_motors.getResetRecoveries(),
                 (unsigned int)g_motors.getBusFaults(),
                 !g_imu.isHardwareConnected() ? "MISSING, on encoders" : (g_imu.isUsingFallback() ? "FAULT, on encoders" : "OK"),
                 (unsigned int)g_imu.getFaultCount(),
                 g_imu.isGyroFlipped() ? ", gyro axis flipped" : "",
                 (unsigned int)g_motion_controller.getEncoderFaults(),
                 (unsigned long)telemetry.timing.loop_overruns);
        reply(buf);
        return;
    }
    if (cmd == "log" || cmd == "log clear") {
        // Planned vs. actual motion from the last run, as CSV (paste into a spreadsheet and plot)
        if (running) { reply("ERR: CANNOT_READ_LOG_WHILE_RUNNING"); return; }
        if (cmd == "log clear") {
            g_motion_controller.clearRunLog();
            reply("ACK: LOG CLEARED");
            return;
        }
        if (s_source == SOURCE_BLE) { reply("ERR: LOG IS TOO LONG FOR BLUETOOTH, USE USB OR TELNET"); return; }

        const RunLog& log = g_motion_controller.getRunLog();
        reply("t_ms,action,target_mm_s,speed_mm_s,heading_deg,heading_err_deg,forward_pct,turn_pct,L90,L45,FL,FR,R45,R90");
        for (uint16_t i = 0; i < log.size(); ++i) {
            const RunLogSample& row = log.at(i);
            snprintf(buf, sizeof(buf), "%lu,%u,%d,%d,%.1f,%.1f,%d,%d,%u,%u,%u,%u,%u,%u",
                     (unsigned long)i * RUN_LOG_DIVIDER * 1000UL / CONTROL_LOOP_FREQ_HZ, (unsigned int)row.action,
                     (int)row.target_speed_mm_s, (int)row.speed_mm_s,
                     row.heading_x10 / 10.0f, row.heading_err_x10 / 10.0f,
                     (int)row.forward_pct, (int)row.turn_pct,
                     (unsigned int)row.ir[0], (unsigned int)row.ir[1], (unsigned int)row.ir[2],
                     (unsigned int)row.ir[3], (unsigned int)row.ir[4], (unsigned int)row.ir[5]);
            reply(buf);
        }
        snprintf(buf, sizeof(buf), "ACK: %u LOG ROWS", (unsigned int)log.size());
        reply(buf);
        return;
    }
    if (cmd == "perf") {
        RobotTelemetry telemetry = {};
        getTelemetry(telemetry);
        snprintf(buf, sizeof(buf), "PERF: Loop=%u us (Peak=%u us, Budget=2000 us) | Overruns=%lu | Stack Rem=%lu words",
                 (unsigned int)telemetry.timing.loop_time_us, (unsigned int)telemetry.timing.max_loop_time_us,
                 (unsigned long)telemetry.timing.loop_overruns, (unsigned long)telemetry.timing.stack_high_water);
        reply(buf);
        return;
    }
    if (cmd == "enc" || cmd == "encinfo") {
        RobotTelemetry telemetry = {};
        getTelemetry(telemetry);
        const EncoderState& enc = telemetry.encoders;
        bool inv_l = false, inv_r = false;
        g_encoders.getInverted(inv_l, inv_r);
        snprintf(buf, sizeof(buf), "ENC: L=%ld (%4.1fmm, %4.0fmm/s, inv=%d) | R=%ld (%4.1fmm, %4.0fmm/s, inv=%d)",
                 (long)enc.left_ticks_total, enc.left_dist_mm, enc.left_speed_mm_s, (int)inv_l,
                 (long)enc.right_ticks_total, enc.right_dist_mm, enc.right_speed_mm_s, (int)inv_r);
        reply(buf);
        return;
    }

    // ---------------------------------------------------------------- Motor / encoder settings
    if (cmd.startsWith("motortrim")) {
        float trim_left = 1.0f, trim_right = 1.0f;
        if (cmd == "motortrim") {
            g_motors.getTrim(trim_left, trim_right);
            snprintf(buf, sizeof(buf), "MOTORS TRIM: Left=%.4f | Right=%.4f", trim_left, trim_right);
        } else if (parseTwoFloats(cmd, trim_left, trim_right)) {
            g_motors.setTrim(trim_left, trim_right);
            g_motors.saveToNVS();
            snprintf(buf, sizeof(buf), "ACK: TRIM UPDATED & SAVED (L=%.4f, R=%.4f)", trim_left, trim_right);
        } else {
            snprintf(buf, sizeof(buf), "ERR: USAGE 'motortrim <left> <right>'");
        }
        reply(buf);
        return;
    }
    if (cmd.startsWith("motorinv")) {
        bool inv_l = false, inv_r = false;
        if (cmd == "motorinv") {
            g_motors.getInverted(inv_l, inv_r);
            snprintf(buf, sizeof(buf), "MOTOR INVERT: L=%d | R=%d", (int)inv_l, (int)inv_r);
        } else if (parseTwoFlags(cmd, inv_l, inv_r)) {
            g_motors.setInverted(inv_l, inv_r);
            snprintf(buf, sizeof(buf), "ACK: MOTOR INVERT SET (L=%d, R=%d)", (int)inv_l, (int)inv_r);
        } else {
            snprintf(buf, sizeof(buf), "ERR: USAGE 'motorinv <left:0/1> <right:0/1>'");
        }
        reply(buf);
        return;
    }
    if (cmd.startsWith("encinv")) {
        bool inv_l = false, inv_r = false;
        if (cmd == "encinv") {
            g_encoders.getInverted(inv_l, inv_r);
            snprintf(buf, sizeof(buf), "ENCODER INVERT: L=%d | R=%d", (int)inv_l, (int)inv_r);
        } else if (parseTwoFlags(cmd, inv_l, inv_r)) {
            g_encoders.setInverted(inv_l, inv_r);
            snprintf(buf, sizeof(buf), "ACK: ENCODER INVERT SET (L=%d, R=%d)", (int)inv_l, (int)inv_r);
        } else {
            snprintf(buf, sizeof(buf), "ERR: USAGE 'encinv <left:0/1> <right:0/1>'");
        }
        reply(buf);
        return;
    }

#if ENABLE_CALIBRATION
    // ---------------------------------------------------------------- Calibration build only
    if (Calibration::handleCommand(cmd, reply)) return;
#endif

    if (cmd == "help") {
        reply("Debug commands: stop, status, health, perf, enc, log, log clear, motortrim [l r], motorinv [l r], "
              "encinv [l r], resetenc, resetheading. Runs are started by hand waves only.");
        return;
    }

    snprintf(buf, sizeof(buf), "ERR: UNKNOWN COMMAND '%s' (type 'help')", cmd.c_str());
    reply(buf);
}

void poll() {
#if ENABLE_BLE_DEBUG
    if (BLEDebug::hasCommand()) {
        handle(BLEDebug::readCommand(), SOURCE_BLE);
    }
#endif

#if ENABLE_WIFI_OTA
    WifiOTA::handle(); // Also services over-the-air firmware uploads
    if (WifiOTA::hasCommand()) {
        handle(WifiOTA::readCommand(), SOURCE_TELNET);
    }
#endif

    if (Serial.available()) {
        handle(Serial.readStringUntil('\n'), SOURCE_USB);
    }
}

} // namespace Console
