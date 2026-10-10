#include <functional>
#include "robot.h"
#include "ui/status_led/status_led.h"
#include "ui/actions/actions.h"
#include "ui/gestures/gestures.h"
#include "ui/console/console.h"
#include "ui/run_result/run_result.h"
#include "ui/proposal/proposal.h"
#include "ui/stages/stages.h"
#include "ui/turn_test/turn_test.h"
#include "ui/motor_check/motor_check.h"
#include "ui/move_check/move_check.h"
#include "ui/straight_test/straight_test.h"
#include "wireless/ble_debug/ble_debug.h"
#include "wireless/wifi_ota/wifi_ota.h"

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
    Serial.println(msg); // Reaches USB directly, and Bluetooth / Telnet through the debug log copy
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
    cmd.trim();
    const String typed = cmd; // Before lower-casing, for the commands that carry a sentence
    cmd.toLowerCase();
    if (cmd.length() == 0) return;

    s_source = source;
    // (says where it came from, so a recording shows what was pressed in the app and what was
    // sent by the computer over Bluetooth)
    Serial.printf("[CMD] Received (%s): '%s'\n",
                  source == SOURCE_TELNET ? "app" : (source == SOURCE_BLE ? "Bluetooth" : "USB"), cmd.c_str());

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

#if ENABLE_BLE_DEBUG || ENABLE_WIFI_OTA
    // ---------------------------------------------------------------- Phone app buttons
    // Added at the owner's request (2026-10-07). Only in builds with a radio: the competition
    // build has none of these, so there a run can still only be started by hand waves.
    // "speedrun" uses the speed tier; "speedrun 45" runs this one at 45 % of full speed (10..100),
    // which is what the app's speed slider sends.
    if (cmd == "start" || cmd.startsWith("speedrun")) {
        if (running) { reply("ERR: ALREADY_RUNNING"); return; }
        if (cmd != "start") {
            const long percent = cmd.substring(8).toInt(); // 0 when no number was given
            if (percent != 0 && (percent < 10 || percent > 100)) { reply("ERR: SPEED MUST BE 10..100 PERCENT"); return; }
            Actions::setNextSpeedRunPercent((uint8_t)percent);
        }
        if (cmd == "start") {
            reply("Calibrating the IR sensors, then starting the search...");
            if (!Actions::calibrateIR()) reply("IR calibration failed here; using the stored / default levels.");
            Actions::selectMode(Actions::MODE_SEARCH);
        } else {
            Actions::selectMode(Actions::MODE_HYBRID);
        }
        Actions::launchSelectedRun();
        reply("ACK: STARTED");
        return;
    }
    // After a speed run: drive back to the start cell and face into the maze, ready to go again
    if (cmd == "return") {
        if (running) { reply("ERR: ALREADY_RUNNING"); return; }
        reply(Actions::returnToStart() ? "ACK: RETURNING TO THE START CELL"
                                       : "ERR: ONLY AFTER A SPEED RUN HAS FINISHED AT THE GOAL (otherwise the robot does not know where it is)");
        return;
    }
    if (cmd == "calib") {
        if (running) { reply("ERR: CANNOT_CALIB_WHILE_RUNNING"); return; }
        // Measures where it stands, turns round, measures again, ends facing into the maze
        reply(Actions::calibrateIRBothWays());
        return;
    }
    if (cmd == "clear") {
        if (running) { reply("ERR: CANNOT_CLEAR_WHILE_RUNNING"); return; }
        Actions::clearSavedMaze();
        reply("ACK: MAZE CLEARED");
        return;
    }
    // Turn test: spin 45° on the spot, left or right, with no navigation. For checking that the
    // robot turns the way it is told and by the right amount. The heading is NOT zeroed between
    // presses, so two left turns should read 90: each press aims for the next exact multiple of
    // 45° (see the grid snapping in MotionController::executeCommand), which is how the maze
    // moves avoid adding up their errors. "resetall" zeroes the heading; do that after moving
    // the robot by hand, or it will first turn back to where it was.
    if (cmd == "left45" || cmd == "right45") {
        if (running) { reply("ERR: CANNOT_TEST_WHILE_RUNNING (STOP first)"); return; }
        MotionCommand turn = {};
        turn.action = (cmd == "left45") ? ACTION_TURN_LEFT_45 : ACTION_TURN_RIGHT_45;
        turn.param_value = 45.0f;
        turn.max_speed_mm_s = SEARCH_TURN_SPEED_DEG_S;  // deg/s for turns
        turn.acceleration = SEARCH_TURN_ACCEL_DEG_S2;
        xQueueSend(g_motion_cmd_queue, &turn, 0);
        reply(cmd == "left45" ? "Turning 45 degrees LEFT (anticlockwise seen from above; heading should go UP to the next multiple of 45)."
                              : "Turning 45 degrees RIGHT (clockwise seen from above; heading should go DOWN to the next multiple of 45).");
        return;
    }

    // Movement check (stage C): on the floor, finds which way each motor and encoder runs from
    // the heading, puts it right, then drives one cell and turns 90 degrees each way
    if (cmd == "movecheck") {
        if (running) { reply("ERR: CANNOT_TEST_WHILE_RUNNING (STOP first)"); return; }
        MoveCheck::run();
        return;
    }

    // Straight test (stage E): "straighttest 3" drives 3 cells as the search does and measures
    // the wobble, the walls seen and where it ended (see StraightTest)
    if (cmd == "straighttest" || cmd.startsWith("straighttest ")) {
        if (running) { reply("ERR: CANNOT_TEST_WHILE_RUNNING (STOP first)"); return; }
        const long cells = (cmd == "straighttest") ? 3 : cmd.substring(13).toInt();
        StraightTest::run(cells > 0 ? (int)cells : 3);
        return;
    }

    // Speed-run turn test (stage F): "curvetest left 40" drives one smooth left curve as a speed
    // run would, at 40 % speed. Shapes: left right uleft uright zigleft zigright veeleft veeright.
    if (cmd.startsWith("curvetest ")) {
        if (running) { reply("ERR: CANNOT_TEST_WHILE_RUNNING (STOP first)"); return; }
        String rest = cmd.substring(10);
        rest.trim();
        const int space = rest.indexOf(' ');
        const String shape = (space < 0) ? rest : rest.substring(0, space);
        const long percent = (space < 0) ? 35 : rest.substring(space + 1).toInt();
        if (!Actions::launchCurveTest(shape.c_str(), (uint8_t)(percent > 0 ? percent : 35))) {
            reply("ERR: USAGE 'curvetest <left|right|uleft|uright|zigleft|zigright|veeleft|veeright> [percent]'");
        }
        return;
    }

    // Motor check: each wheel alone, both ways, with what the encoders and the heading did
    if (cmd == "motorcheck") {
        if (running) { reply("ERR: CANNOT_TEST_WHILE_RUNNING (STOP first)"); return; }
        MotorCheck::run();
        return;
    }

    // Turn tuning test: several left and right turns on the spot, each measured (see TurnTest).
    // "turntest" does 5 of each, "turntest 3" does 3 of each.
    if (cmd == "turntest" || cmd.startsWith("turntest ")) {
        if (running) { reply("ERR: CANNOT_TEST_WHILE_RUNNING (STOP first)"); return; }
        // ("turntest fast" / "turntest 3 fast": at the speed-run turn speed)
        const bool fast = cmd.endsWith(" fast");
        const long pairs = (cmd == "turntest" || cmd == "turntest fast") ? 5 : cmd.substring(9).toInt();
        TurnTest::run(pairs > 0 ? (int)pairs : 5, fast);
        return;
    }

    // The two ways the SEARCH turns 90 degrees, on their own, so each can be checked like the
    // 45-degree test above:
    //   left90 / right90          the turn on the spot it makes at a cell centre. Same control
    //                             code as the 45-degree test, only the angle differs.
    //   curveleft / curveright    the smooth 90-degree curve it drives while rolling, when the
    //                             look-ahead has seen the gap in time. Different code: the robot
    //                             drives an arc (about 95 mm forward and 95 mm sideways) and the
    //                             heading target follows the planned distance along it. Here it
    //                             starts and ends at rest; give it room.
    if (cmd == "left90" || cmd == "right90" || cmd == "curveleft" || cmd == "curveright") {
        if (running) { reply("ERR: CANNOT_TEST_WHILE_RUNNING (STOP first)"); return; }
        const bool left = (cmd == "left90" || cmd == "curveleft");
        MotionCommand move = {};
        if (cmd == "left90" || cmd == "right90") {
            move.action = left ? ACTION_TURN_LEFT_90 : ACTION_TURN_RIGHT_90;
            move.param_value = 90.0f;
            move.max_speed_mm_s = SEARCH_TURN_SPEED_DEG_S;  // deg/s for turns
            move.acceleration = SEARCH_TURN_ACCEL_DEG_S2;
            reply(left ? "Turning 90 degrees LEFT on the spot (heading should go UP by 90)."
                       : "Turning 90 degrees RIGHT on the spot (heading should go DOWN by 90).");
        } else {
            move.action = left ? ACTION_CURVE_LEFT_90 : ACTION_CURVE_RIGHT_90;
            move.param_value = CURVE_90_LENGTH_MM;
            move.max_speed_mm_s = SEARCH_CURVE_SPEED_MM_S;
            move.acceleration = SEARCH_ACCEL_DEFAULT_MM_S2;
            reply(left ? "Driving a smooth 90 degree curve to the LEFT (heading should go UP by 90)."
                       : "Driving a smooth 90 degree curve to the RIGHT (heading should go DOWN by 90).");
        }
        xQueueSend(g_motion_cmd_queue, &move, 0);
        return;
    }

    // The search, one move at a time: the same maze-solving code as START, except that the robot
    // stops at every cell centre and waits. The first "cell" starts the search from the start
    // cell; each one after that lets it make its next move (one cell forward, or one turn).
    if (cmd == "cell") {
        if (g_navigator->isWaitingForNextMove()) {
            RobotTelemetry telemetry = {};
            getTelemetry(telemetry);
            NavigatorLock lock;
            g_navigator->continueOneMove(telemetry.ir, g_motion_controller.getWallPreview());
            return;
        }
        if (running) { reply("ERR: STILL_MOVING (or a run is in progress: STOP it first)"); return; }
        reply("Calibrating the IR sensors, then searching one move at a time...");
        if (!Actions::calibrateIR()) reply("IR calibration failed here; using the stored / default levels.");
        Actions::selectMode(Actions::MODE_SEARCH);
        Actions::launchSelectedRun(true);
        return;
    }
#endif

    // ---------------------------------------------------------------- Live tuning
    // "tune" lists the gains, "tune <name> <value>" changes one at once, "tune save" keeps the
    // current set through power-off, "tune reset" goes back to the values in the code.
    if (cmd.startsWith("tune")) {
        char name[12] = "";
        float value = 0.0f;
        if (cmd == "tune") {
            reply("TUNING   now  (value in the code)");
            for (int i = 0; i < MotionController::TUNE_COUNT; ++i) {
                snprintf(buf, sizeof(buf), "  %-6s = %-9.6g (%.6g)  %s", MotionController::tuneName(i),
                         g_motion_controller.getTune(i), MotionController::tuneDefault(i), MotionController::tuneDescription(i));
                reply(buf);
            }
            reply("To keep a set for good, copy the numbers into kTune in control.cpp (flash can be wiped).");
        } else if (running && !g_navigator->isWaitingForNextMove()) {
            reply("ERR: CANNOT_TUNE_WHILE_RUNNING");
        } else if (cmd == "tune save") {
            g_motion_controller.saveToNVS();
        } else if (cmd == "tune reset") {
            g_motion_controller.forgetSavedTuning();
        } else if (sscanf(cmd.c_str(), "tune %11s %f", name, &value) == 2) {
            int index = -1;
            for (int i = 0; i < MotionController::TUNE_COUNT; ++i) {
                if (strcmp(name, MotionController::tuneName(i)) == 0) index = i;
            }
            if (index < 0) {
                reply("ERR: no such setting (send 'tune' for the list)");
            } else if (!g_motion_controller.setTune(index, value)) {
                snprintf(buf, sizeof(buf), "ERR: %s refused, %g is outside the allowed range. It stays at %.6g.",
                         name, value, g_motion_controller.getTune(index));
                reply(buf);
            } else {
                snprintf(buf, sizeof(buf), "ACK: %s = %.6g (not saved yet: 'tune save' keeps it)", name, g_motion_controller.getTune(index));
                reply(buf);
            }
        } else {
            reply("ERR: USAGE 'tune', 'tune <name> <value>', 'tune save', 'tune reset'");
        }
        return;
    }

    // ---------------------------------------------------------------- Debug helpers
    if (cmd == "resetall") {
        // The phone app's "Reset sensors" button: wheel counters and heading back to zero
        if (running) { reply("ERR: CANNOT_RESET_WHILE_RUNNING"); return; }
        requestEncoderReset();
        requestHeadingReset();
        reply("ACK: WHEEL COUNTERS AND HEADING RESET TO ZERO");
        return;
    }
    if (cmd.startsWith("irtest")) {
        // Wiring and strength check: put the robot in a cell with walls close on the left, right
        // and in front. Each row lights ONE emitter; the columns are how much each receiver rose.
        // The biggest number in each row should be on the diagonal. Optional number = emitter
        // on-time in microseconds (default 300).
        if (running) { reply("ERR: CANNOT_TEST_WHILE_RUNNING"); return; }
        long on_time = cmd.substring(6).toInt();
        if (on_time < 50 || on_time > 20000) on_time = IR_PULSE_SETTLE_US;
        static int16_t rise[6][6];
        g_ir_sensors.measureCrossTable(rise, (uint32_t)on_time);
        static const char* const kNames[6] = { "L90", "L45", "FL ", "FR ", "R45", "R90" };
        snprintf(buf, sizeof(buf), "IRTEST (emitter on for %ld us)   receiver:  L90   L45    FL    FR   R45   R90", on_time);
        reply(buf);
        for (int e = 0; e < 6; ++e) {
            snprintf(buf, sizeof(buf), "  emitter %s alone ->            %5d %5d %5d %5d %5d %5d", kNames[e],
                     rise[e][0], rise[e][1], rise[e][2], rise[e][3], rise[e][4], rise[e][5]);
            reply(buf);
        }
        return;
    }
    if (cmd == "ir") {
        // Raw ADC readings per sensor, emitter off then on. A working sensor facing a wall shows
        // "on" well above "off". "off" stuck near 4095 means room light is swamping the sensor.
        static const char* const kNames[6] = { "L90", "L45", "FL ", "FR ", "R45", "R90" };
        for (uint8_t ch = 0; ch < 6; ++ch) {
            snprintf(buf, sizeof(buf), "IR %s: emitter off %4d, on %4d, difference %4d",
                     kNames[ch], (int)g_ir_sensors.getRawAmbient(ch), (int)g_ir_sensors.getRawLit(ch),
                     (int)g_ir_sensors.getRawLit(ch) - (int)g_ir_sensors.getRawAmbient(ch));
            reply(buf);
        }
        snprintf(buf, sizeof(buf), "IR wall levels: front %d, left %d, right %d (a reading above its level counts as a wall)",
                 (int)g_ir_sensors.getThresholdFront(), (int)g_ir_sensors.getThresholdL90(), (int)g_ir_sensors.getThresholdR90());
        reply(buf);
        snprintf(buf, sizeof(buf), "Front sensors compared (0 = square to the wall, or not calibrated facing one): %.2f",
                 g_ir_sensors.getFrontSkew());
        reply(buf);
        reply(g_ir_sensors.isFrontLevelMeasured()
              ? "The front level was measured against a real wall."
              : "The front level is ONLY A GUESS: calibrate once in the middle of a cell facing a wall (walls on both sides too).");
        return;
    }
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

    // ---------------------------------------------------------------- Maze size
    // "maze" says which maze the robot is set up for; "maze 3" / "maze 5" / "maze 16" switches (the app's
    // maze button). Starts nothing. Each size keeps its own saved map.
    if (cmd == "maze" || cmd.startsWith("maze ")) {
        const long size = cmd.substring(4).toInt();
        if (cmd != "maze" && !Actions::setMazeSize((uint8_t)size)) {
            reply("ERR: MAZE SIZE NOT CHANGED (use 'maze 3', 'maze 5' or 'maze 16', not during a run; the competition and test3x3 builds are fixed)");
        }
        snprintf(buf, sizeof(buf), "MAZE: %dx%d", (int)MAZE_ACTIVE_SIZE, (int)MAZE_ACTIVE_SIZE);
        reply(buf);
        return;
    }

    // ---------------------------------------------------------------- Verdict on the last run
    // "success" or "fail", optionally followed by a note ("fail clipped the post in 2,1"): the
    // app's two buttons. Starts nothing; it only prints what was in force (see RunResult).
    if (cmd.startsWith("success") || cmd.startsWith("fail")) {
        const bool success = cmd.startsWith("success");
        String note = typed.substring(success ? 7 : 4); // As typed, capitals and all
        note.trim();
        RunResult::report(success, note.c_str());
        return;
    }

    // ---------------------------------------------------------------- Question for the operator
    // A suggested change, shown in the phone app with Yes and No buttons (see Proposal). The text
    // arrives in pieces because a command line is short. Starts nothing on the robot.
    // The owner's staged plan: "stage" says where things stand, "stage c" works on stage C,
    // "stage done c" marks C finished and moves on (the app asks twice before sending that)
    if (cmd == "stage") { Stages::report(); return; }
    if (cmd.startsWith("stage ")) {
        const bool done = cmd.startsWith("stage done ");
        const String letter = cmd.substring(done ? 11 : 6);
        const int stage = (letter.length() == 1) ? letter[0] - 'a' : -1;
        if (stage < 0 || stage >= Stages::COUNT) { reply("ERR: USAGE 'stage', 'stage <a..g>' or 'stage done <a..g>'"); return; }
        if (done) Stages::markDone(stage); else Stages::goTo(stage);
        return;
    }

    if (cmd == "ask new")        { Proposal::startNew(); return; }
    if (cmd.startsWith("ask+ ")) { Proposal::append(typed.c_str() + 5); return; }
    if (cmd == "ask show")       { Proposal::show(); return; }
    if (cmd == "ask clear")      { Proposal::clear(); reply("ACK: QUESTION TAKEN DOWN"); return; }
    if (cmd == "yes" || cmd == "no") {
        if (!Proposal::answer(cmd == "yes")) reply("ERR: NO QUESTION IS ON SHOW");
        return;
    }

    // ---------------------------------------------------------------- Reports
    if (cmd == "status") {
        RobotTelemetry telemetry = {};
        getTelemetry(telemetry);
        snprintf(buf, sizeof(buf), "STATUS: %s | Selected mode: %s | Speed tier %d | Battery %.2f V",
                 Actions::stateDescription(), Actions::modeName(Actions::getSelectedMode()),
                 (int)Actions::getSpeedTier(), telemetry.vbat_volts);
        reply(buf);
        GestureUI::describe(buf, sizeof(buf), telemetry.ir);
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
        // The IMU is read 100 times a second (every 5th control tick), which is how often the
        // BNO055 produces a new heading. These say how those reads are going, and why any fail.
        snprintf(buf, sizeof(buf),
                 "IMU READS: %lu good | failed: %lu no answer, %lu cut short, %lu impossible value | slowest read %lu us (a 2000 us control tick has to fit it)",
                 (unsigned long)g_imu.getReadsOk(), (unsigned long)g_imu.getReadsNoAnswer(), (unsigned long)g_imu.getReadsShort(),
                 (unsigned long)g_imu.getReadsBadValue(), (unsigned long)g_imu.getSlowestReadUs());
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

        // 1500 rows would overflow the wireless log copy, so these go straight to USB (and to
        // Telnet if that is where they were asked for) instead of through reply()
        auto sendRow = [](const char* row) {
            usbSerial().println(row);
#if ENABLE_WIFI_OTA
            if (s_source == SOURCE_TELNET) WifiOTA::println(row);
#endif
        };
        const RunLog& log = g_motion_controller.getRunLog();
        sendRow("t_ms,action,target_mm_s,speed_mm_s,heading_deg,heading_err_deg,forward_pct,turn_pct,L90,L45,FL,FR,R45,R90");
        for (uint16_t i = 0; i < log.size(); ++i) {
            const RunLogSample& row = log.at(i);
            snprintf(buf, sizeof(buf), "%lu,%u,%d,%d,%.1f,%.1f,%d,%d,%u,%u,%u,%u,%u,%u",
                     (unsigned long)i * RUN_LOG_DIVIDER * 1000UL / CONTROL_LOOP_FREQ_HZ, (unsigned int)row.action,
                     (int)row.target_speed_mm_s, (int)row.speed_mm_s,
                     row.heading_x10 / 10.0f, row.heading_err_x10 / 10.0f,
                     (int)row.forward_pct, (int)row.turn_pct,
                     (unsigned int)row.ir[0], (unsigned int)row.ir[1], (unsigned int)row.ir[2],
                     (unsigned int)row.ir[3], (unsigned int)row.ir[4], (unsigned int)row.ir[5]);
            sendRow(buf);
        }
        snprintf(buf, sizeof(buf), "ACK: %u LOG ROWS", (unsigned int)log.size());
        sendRow(buf);
        return;
    }
    if (cmd == "stream" || cmd == "stream on" || cmd == "stream off") {
        // The 5 Hz line of sensor readings. Quiet by default so the console stays readable.
        bool on = (cmd == "stream") ? !isTelemetryStreaming() : (cmd == "stream on");
        setTelemetryStreaming(on);
        reply(on ? "ACK: STREAM ON (send 'stream' again to stop)" : "ACK: STREAM OFF");
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
        reply("Commands: start, speedrun [percent], return, calib, clear, cell, left45, right45, left90, right90, curveleft, curveright (builds with a radio only); stop, status, health, tune [name value | save | reset], ir, irtest [us], resetall, stream, perf, enc, log, log clear, motortrim [l r], motorinv [l r], "
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
    // (the Wi-Fi itself is serviced by telemetryTask; only the commands arrive here)
    if (WifiOTA::hasCommand()) {
        handle(WifiOTA::readCommand(), SOURCE_TELNET);
        WifiOTA::commandDone();
    }
#endif

    if (Serial.available()) {
        handle(Serial.readStringUntil('\n'), SOURCE_USB);
    }
}

} // namespace Console
