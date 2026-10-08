// ==============================================================================
// ANTIGRAVITIEEE MICROMOUSE - FIRMWARE ENTRY POINT
//
// The ESP32-S3 has two cores, and this file gives each one a job:
//
//   Core 1  motionControlTask   500 Hz: read sensors, run the PID loops, drive the motors
//   Core 0  navigationTask      decide where to go, listen for hand waves and text commands
//   Core 0  telemetryTask       copies everything printed to Bluetooth / Telnet; 5 Hz readings line when asked
//
// Four tasks share the two processor cores (see the banner above each one):
//   Core 1  motionControlTask  500 Hz, highest priority: sensors, control loops, motors
//   Core 0  navigationTask     sleeps until a move finishes, then sends the next one at once
//   Core 0  operatorTask       console, phone-app buttons, reports, LED, hand waves
//   Core 0  telemetryTask      the phone app's web page, Telnet, Bluetooth, the status line
//
// There are no buttons: power the robot on and control it with hand waves (GestureUI in ui/gestures/gestures.h).
// The LED on the ESP32 board shows what it is doing. The text console (Console in ui/console/console.h) is for
// debugging only and cannot start a run.
// ==============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <atomic>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "robot.h"
#include "ui/status_led/status_led.h"
#include "ui/actions/actions.h"
#include "ui/gestures/gestures.h"
#include "ui/console/console.h"
#include "wireless/ble_debug/ble_debug.h"
#include "wireless/wifi_ota/wifi_ota.h"

// ==============================================================================
// SHARED ROBOT OBJECTS (declared in robot.h)
// ==============================================================================
Encoders         g_encoders;
Motors           g_motors;
IRSensors        g_ir_sensors;
IMU              g_imu;
MotionController g_motion_controller(g_encoders, g_motors, g_ir_sensors, g_imu);
Navigator*       g_navigator        = nullptr;
QueueHandle_t    g_motion_cmd_queue = nullptr;

// Sensor snapshot published by the motion task for everyone on Core 0
static RobotTelemetry    g_shared_telemetry;
static SemaphoreHandle_t g_telemetry_mutex = nullptr;

// Posted by the motion task on the very control tick a move finishes, with what the sensors
// read on that tick. The navigation task sleeps on this queue, so it wakes the moment the move
// ends and decides the next one from readings that are not even one tick old.
struct MotionDone {
    IRReadings  ir;
    WallPreview preview; // What the 45° sensors saw of the cell ahead during the move
};
static QueueHandle_t g_motion_done_queue = nullptr;

// See robot.h
SemaphoreHandle_t g_navigator_mutex = nullptr;

// The last finished move, kept for the operator task's one-line summary of it ([POS]).
// Written by the navigation task and read by the operator task, both under a NavigatorLock.
static MotionDone g_last_done = {};
static uint32_t   g_moves_done = 0;

// Set by the navigation task when it aborts a run after a safety stop; the operator task then
// shows the result on the LED
static std::atomic<bool> g_run_aborted{false};

// The 5 Hz status line. Off at power-on; the debug console command "stream" switches it.
static std::atomic<bool> g_telemetry_streaming{false};
void setTelemetryStreaming(bool on) { g_telemetry_streaming.store(on); }
bool isTelemetryStreaming() { return g_telemetry_streaming.load(); }

// Reference-frame resets requested by Core 0 and carried out by the motion task
static const uint8_t RESET_REQ_ENCODERS = 0x01;
static const uint8_t RESET_REQ_HEADING  = 0x02;
static const uint8_t RESET_REQ_NEW_RUN  = 0x04;
static std::atomic<uint8_t> g_pending_resets{0};

bool getTelemetry(RobotTelemetry& out) {
    if (xSemaphoreTake(g_telemetry_mutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;
    out = g_shared_telemetry;
    xSemaphoreGive(g_telemetry_mutex);
    return true;
}

void requestEncoderReset() { g_pending_resets.fetch_or(RESET_REQ_ENCODERS); }
void requestHeadingReset() { g_pending_resets.fetch_or(RESET_REQ_HEADING); }

void prepareForNewRun() {
    g_pending_resets.fetch_or(RESET_REQ_NEW_RUN);
    // Wait for the motion task to carry it out (it runs every 2 ms)
    for (int i = 0; i < 50 && (g_pending_resets.load() & RESET_REQ_NEW_RUN); ++i) {
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// The latest verdict of the 45° sensors on the next cell's side walls, in words, for the phone app
static char s_look_text[40] = "nothing yet";

#if ENABLE_WIFI_OTA
// The robot's live numbers for the phone app, as one JSON object
static String buildAppStatus() {
    RobotTelemetry t = {};
    getTelemetry(t);

    // Where the navigator believes the robot is, and how many cells it has explored
    static const char* const kCompass[4] = { "north", "east", "south", "west" };
    const RobotPose pose = g_navigator->getPose();
    int cells_seen = 0;
    for (int8_t x = 0; x < MAZE_ACTIVE_SIZE; ++x) {
        for (int8_t y = 0; y < MAZE_ACTIVE_SIZE; ++y) {
            if (g_navigator->getMaze().isVisited(x, y)) cells_seen++;
        }
    }

    // The live-tuning values, in the order of MotionController's tuning table
    char tune[440] = "";
    for (int i = 0, used = 0; i < MotionController::TUNE_COUNT && used < (int)sizeof(tune) - 16; ++i) {
        used += snprintf(tune + used, sizeof(tune) - used, i ? ",%.6g" : "%.6g", g_motion_controller.getTune(i));
    }

    // IMU state, with how many of its reads have failed since power-on
    char imu_text[48];
    snprintf(imu_text, sizeof(imu_text), "%s, %lu failed reads",
             !g_imu.isHardwareConnected() ? "MISSING" : (g_imu.isUsingFallback() ? "FAULT" : "OK"),
             (unsigned long)g_imu.getBadReadCount());

    char buf[1400];
    snprintf(buf, sizeof(buf),
             "{\"state\":\"%s\",\"mode\":\"%s\",\"tier\":%d,\"cell\":\"(%d, %d) facing %s\",\"visited\":%d,\"look\":\"%s\",\"why\":\"%s\",\"late\":%u,"
             "\"ir\":[%d,%d,%d,%d,%d,%d],\"walls\":\"%c%c%c\","
             "\"heading\":%.1f,\"vbat\":%.2f,\"encL\":\"%ld (%.0f mm)\",\"encR\":\"%ld (%.0f mm)\","
             "\"motor\":\"%s\",\"imu\":\"%s\",\"supply\":%.1f,\"loop\":%u,\"busy\":%d,\"tune\":[%s]",
             Actions::stateDescription(), Actions::modeName(Actions::getSelectedMode()), (int)Actions::getSpeedTier(),
             (int)pose.cell_x, (int)pose.cell_y, kCompass[pose.current_dir % 4], cells_seen, s_look_text,
             g_navigator->getEdgeNote(), (unsigned int)g_motion_controller.getLateHandovers(),
             t.ir.left_90, t.ir.left_45, t.ir.front_left, t.ir.front_right, t.ir.right_45, t.ir.right_90,
             t.ir.wall_left ? 'L' : '.', t.ir.wall_front ? 'F' : '.', t.ir.wall_right ? 'R' : '.',
             t.imu.heading_deg, t.vbat_volts,
             (long)t.encoders.left_ticks_total, t.encoders.left_dist_mm,
             (long)t.encoders.right_ticks_total, t.encoders.right_dist_mm,
             g_motors.isConnected() ? "OK" : "NOT RESPONDING",
             imu_text,
             g_motors.getSupplyVolts(), (unsigned int)t.timing.loop_time_us,
             (int)((Actions::isRunActive() && !g_navigator->isWaitingForNextMove()) || !g_motion_controller.isCommandFinished()), tune);

    // The map for the app's maze picture. One character per cell, row by row from the start
    // cell (x counts right, then y counts forward):
    //   "map"  = hex digit of the walls believed there: 1 north, 2 east, 4 south, 8 west
    //   "seen" = 0 not visited, 1 visited, +2 if it is a goal cell
    String out;
    out.reserve(sizeof(buf) + 2 * MAZE_ACTIVE_SIZE * MAZE_ACTIVE_SIZE + 160);
    out += buf;
    const Maze& maze = g_navigator->getMaze();
    String seen;
    seen.reserve(MAZE_ACTIVE_SIZE * MAZE_ACTIVE_SIZE);
    out += ",\"map\":\"";
    for (int8_t y = 0; y < MAZE_ACTIVE_SIZE; ++y) {
        for (int8_t x = 0; x < MAZE_ACTIVE_SIZE; ++x) {
            const int walls = (maze.hasWall(x, y, DIR_NORTH) ? 1 : 0) | (maze.hasWall(x, y, DIR_EAST) ? 2 : 0) |
                              (maze.hasWall(x, y, DIR_SOUTH) ? 4 : 0) | (maze.hasWall(x, y, DIR_WEST) ? 8 : 0);
            out += "0123456789abcdef"[walls];
            seen += (char)('0' + (maze.isVisited(x, y) ? 1 : 0) + (Maze::isGoalCell(x, y) ? 2 : 0));
        }
    }
    out += "\",\"seen\":\"";
    out += seen;

    // Where the robot believes it is, and the reading at which each sensor calls "wall"
    // (order L90, L45, FL, FR, R45, R90, as "ir")
    char tail[150];
    snprintf(tail, sizeof(tail), "\",\"n\":%d,\"x\":%d,\"y\":%d,\"d\":%d,\"lvl\":[%d,%d,%d,%d,%d,%d]}",
             (int)MAZE_ACTIVE_SIZE, (int)pose.cell_x, (int)pose.cell_y, (int)(pose.current_dir % 4),
             (int)g_ir_sensors.getThresholdL90(), (int)g_ir_sensors.getThresholdL45(),
             (int)g_ir_sensors.getThresholdFront(), (int)g_ir_sensors.getThresholdFront(),
             (int)g_ir_sensors.getThresholdR45(), (int)g_ir_sensors.getThresholdR90());
    out += tail;
    return out;
}
#endif

// Battery Voltage Sense (10k / 10k divider on Computer Battery)
static float readBatteryVoltage() {
    // analogReadMilliVolts utilizes ESP32-S3 factory eFuse calibration for exact mV
    float pin_voltage = (float)analogReadMilliVolts(PIN_VSENSE_COM) / 1000.0f;
    return pin_voltage * BATTERY_DIVIDER_RATIO;
}

// ==============================================================================
// CORE 1 TASK: 500 Hz REAL-TIME MOTION CONTROL & SENSOR LOOP
// ==============================================================================
void motionControlTask(void* pvParameters) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period_ticks = pdMS_TO_TICKS(1000 / CONTROL_LOOP_FREQ_HZ);

    MotionCommand current_cmd;
    uint32_t loop_counter = 0;
    bool was_busy = false;
    TimingStats timing_stats = {0, 0, 0, 0};

    Serial.printf("[CORE 1] Real-Time Motion Task running on Core %d @ %d Hz\n",
                  xPortGetCoreID(), CONTROL_LOOP_FREQ_HZ);

    for (;;) {
        vTaskDelayUntil(&last_wake_time, period_ticks > 0 ? period_ticks : 1);
        uint32_t t_start_us = micros();

        // 0. Carry out reference-frame resets requested from Core 0 (only between motions)
        if (!was_busy && g_pending_resets.load() != 0) {
            uint8_t resets = g_pending_resets.exchange(0);
            if (resets & RESET_REQ_ENCODERS) g_encoders.reset();
            if (resets & RESET_REQ_HEADING)  g_motion_controller.resetHeading();
            if (resets & RESET_REQ_NEW_RUN)  g_motion_controller.resetTracking();
        }

        // 1. Read Encoders (PCNT hardware 4x decoding) & Pulsed 6-Channel IR (one emitter group per tick)
        g_encoders.update(CONTROL_DT_S);
        g_ir_sensors.update();

        EncoderState enc = g_encoders.getState();

        // 2. Heading: BNO055 at 100 Hz, smoothed to 500 Hz, with encoder odometry as the backup
        float yaw_rate = (enc.right_speed_mm_s - enc.left_speed_mm_s) / WHEEL_BASE_MM * (180.0f / PI);
        g_imu.update(CONTROL_DT_S, yaw_rate, enc.linear_speed_mm_s);

        // 3. Check for motion commands from Navigation
        if (xQueueReceive(g_motion_cmd_queue, &current_cmd, 0) == pdTRUE) {
            g_motion_controller.executeCommand(current_cmd);
            was_busy = true;
        }

        // 4. Update Motion PID + Wall Centering
        g_motion_controller.update(CONTROL_DT_S);

        // 5. Signal movement completion: wakes the navigation task at once, with this tick's readings
        if (was_busy && g_motion_controller.isCommandFinished()) {
            was_busy = false;
            MotionDone done;
            done.ir = g_ir_sensors.getReadings();
            done.preview = g_motion_controller.getWallPreview();
            xQueueOverwrite(g_motion_done_queue, &done);
        }

        // 6. Measure real-time loop duration & jitter (2000 µs period)
        uint32_t t_exec_us = micros() - t_start_us;
        timing_stats.loop_time_us = (uint16_t)t_exec_us;
        if (t_exec_us > timing_stats.max_loop_time_us) {
            timing_stats.max_loop_time_us = (uint16_t)t_exec_us;
        }
        if (t_exec_us > 2000) {
            timing_stats.loop_overruns++;
        }

        // 7. Update Telemetry Snapshot at 50 Hz
        // (The battery voltage is only reported. Nothing stops the robot because of it.)
        if (loop_counter++ % 10 == 0) {
            float vbat = readBatteryVoltage();
            timing_stats.stack_high_water = (uint32_t)uxTaskGetStackHighWaterMark(NULL);

            if (xSemaphoreTake(g_telemetry_mutex, 0) == pdTRUE) {
                g_shared_telemetry.encoders = enc;
                g_shared_telemetry.ir = g_ir_sensors.getReadings();
                g_shared_telemetry.imu = g_imu.getState();
                g_shared_telemetry.motion_completed = g_motion_controller.isCommandFinished();
                g_shared_telemetry.timing = timing_stats;
                g_shared_telemetry.vbat_volts = vbat;
                g_shared_telemetry.loop_count = loop_counter;
                xSemaphoreGive(g_telemetry_mutex);
            }
        }
    }
}

// ==============================================================================
// CORE 0 TASK: NAVIGATION - SENDS THE NEXT MOVE THE MOMENT THE LAST ONE FINISHES
// ==============================================================================
// This task does nothing but sleep until the motion task reports a finished move, then step the
// navigator, which puts the next move on the queue. It has a higher priority than everything
// else of ours on this core and does no console, Wi-Fi, or LED work, so the gap between two
// moves is the navigator's own thinking time and nothing more. That is what keeps the robot
// rolling: the motion controller only coasts 70 ms waiting for a follow-on move before braking.
void navigationTask(void* pvParameters) {
    Serial.printf("[CORE 0] Navigation Task running on Core %d\n", xPortGetCoreID());

    uint32_t test_report_due_ms = 0;       // When to print the follow-up report on a test move (0 = none due)
    float test_heading_at_finish = 0.0f, test_error_at_finish = 0.0f;
    uint32_t test_bad_reads_seen = 0;

    for (;;) {
        // 1. Sleep until a move finishes. Wake now and then anyway to check for a safety stop.
        MotionDone done;
        const TickType_t wait = pdMS_TO_TICKS(Actions::isRunActive() ? NAV_WATCH_PERIOD_MS : 20);
        bool move_finished = xQueueReceive(g_motion_done_queue, &done, wait) == pdTRUE;

        {
            NavigatorLock lock;

            // 2. Automatic safety stop (motor stall / dead encoder): abort the run, don't send the next move
            if (g_motion_controller.consumeSafetyStop()) {
                Serial.println("\n[SAFETY] Run aborted by the motion controller (motor stall or encoder fault).");
                g_navigator->stop();
                xQueueReset(g_motion_done_queue);
                move_finished = false;
                g_run_aborted.store(true);
            }

            // 3. Advance the navigator strictly when a physical motion completes
            if (move_finished) {
                // A test move (turn test) rather than a run: report how it ended, and look again
                // half a second later to see whether the robot went on turning after it "stopped"
                if (!Actions::isRunActive()) {
                    test_heading_at_finish = g_imu.getHeadingDeg();
                    test_error_at_finish = g_motion_controller.getHeadingErrorDeg();
                    test_report_due_ms = millis() + 500;
                    if (test_report_due_ms == 0) test_report_due_ms = 1;
                }
                g_navigator->notifyMotionComplete();
                g_navigator->step(done.ir, done.preview);
                g_last_done = done;
                g_moves_done++;
            }
        }

        if (test_report_due_ms != 0 && (int32_t)(millis() - test_report_due_ms) >= 0) {
            test_report_due_ms = 0;
            const float heading_now = g_imu.getHeadingDeg();
            const float moved_after = normalizeAngle180(heading_now - test_heading_at_finish);
            const uint32_t bad_reads = g_imu.getBadReadCount();
            Serial.printf("[MOVE] Stopped at heading %.1f, %.1f deg from the target. Half a second later: %.1f (moved %.1f more). "
                          "IMU %s, %lu failed reads since the last test. Tuning imu_a=%.3g h_kp=%.3g h_ki=%.3g h_kd=%.3g\n",
                          test_heading_at_finish, test_error_at_finish, heading_now, moved_after,
                          !g_imu.isHardwareConnected() ? "MISSING" : (g_imu.isUsingFallback() ? "FAULT" : "OK"),
                          (unsigned long)(bad_reads - test_bad_reads_seen),
                          g_motion_controller.getTune(8), g_motion_controller.getTune(3),
                          g_motion_controller.getTune(4), g_motion_controller.getTune(5));
            test_bad_reads_seen = bad_reads;
            Serial.println(fabsf(moved_after) > 3.0f
                ? "[MOVE] -> The robot kept turning after the controller thought it had stopped: the heading reading is behind the real turn, or it coasts."
                : fabsf(test_error_at_finish) > 3.0f
                    ? "[MOVE] -> The controller knew it was off target and ran out of settling time."
                    : "[MOVE] -> Landed within tolerance.");
        }
    }
}

// ==============================================================================
// CORE 0 TASK: OPERATOR - CONSOLE, PHONE-APP BUTTONS, REPORTS, LED, HAND WAVES
// ==============================================================================
// Everything that talks to a person. It may print, blink the LED, and take as long as it likes:
// the navigation task has the higher priority and interrupts it whenever a move finishes.
// It starts and stops runs through Actions, which hold a NavigatorLock while they do.
void operatorTask(void* pvParameters) {
    static RobotTelemetry telemetry = {}; // Kept between loops so a missed snapshot reuses the last one
    bool was_running = false;
    NavState last_state = NAV_STATE_ERROR; // Anything but the real one, so the first state is announced
    uint32_t moves_reported = 0;

    Actions::showSelectedMode();
    GestureUI::restart();

    for (;;) {
        // 1. Debug console on Bluetooth / Telnet / USB, and the phone app's buttons
        Console::poll();

        getTelemetry(telemetry);

        // Say what the 45° sensors made of the next cell, the moment they have decided
        if (g_motion_controller.consumePreviewReady()) {
            g_motion_controller.printPreviewReport();
            const WallPreview look = g_motion_controller.getWallPreview();
            snprintf(s_look_text, sizeof(s_look_text), "left %s, right %s",
                     look.left_wall  ? "WALL" : (look.left_open  ? "open" : "not sure"),
                     look.right_wall ? "WALL" : (look.right_open ? "open" : "not sure"));
        }

        // One line per finished move with everything needed to see afterwards what the robot
        // believed at that moment (the Bluetooth recorder, tools/run_recorder, keeps these)
        MotionDone move;
        uint32_t moves_done;
        RobotPose pose;
        {
            NavigatorLock lock;
            move = g_last_done;
            moves_done = g_moves_done;
            pose = g_navigator->getPose();
        }
        if (moves_done != moves_reported) {
            moves_reported = moves_done;
            static const char* const kCompass[4] = { "north", "east", "south", "west" };
            Serial.printf("[POS] move %lu done | now in cell (%d, %d) facing %s | heading %.1f, %.1f off target | "
                          "IR L90=%d L45=%d FL=%d FR=%d R45=%d R90=%d | walls %c%c%c | wall levels side %d/%d front %d | "
                          "look-ahead left %s right %s | wheels L %.0f R %.0f mm | IMU %s, %lu failed reads | late %u\n",
                          (unsigned long)moves_done, (int)pose.cell_x, (int)pose.cell_y, kCompass[pose.current_dir % 4],
                          g_imu.getHeadingDeg(), g_motion_controller.getHeadingErrorDeg(),
                          move.ir.left_90, move.ir.left_45, move.ir.front_left, move.ir.front_right, move.ir.right_45, move.ir.right_90,
                          move.ir.wall_left ? 'L' : '.', move.ir.wall_front ? 'F' : '.', move.ir.wall_right ? 'R' : '.',
                          (int)g_ir_sensors.getThresholdL90(), (int)g_ir_sensors.getThresholdR90(), (int)g_ir_sensors.getThresholdFront(),
                          move.preview.left_wall  ? "WALL" : (move.preview.left_open  ? "open" : "unsure"),
                          move.preview.right_wall ? "WALL" : (move.preview.right_open ? "open" : "unsure"),
                          telemetry.encoders.left_dist_mm, telemetry.encoders.right_dist_mm,
                          !g_imu.isHardwareConnected() ? "MISSING" : (g_imu.isUsingFallback() ? "FAULT" : "OK"),
                          (unsigned long)g_imu.getBadReadCount(), (unsigned int)g_motion_controller.getLateHandovers());
        }

        // Say what the robot is doing whenever that changes
        if (g_navigator->getState() != last_state) {
            last_state = g_navigator->getState();
            Serial.printf("[STATE] %s\n", Actions::stateDescription());
        }

        // 2. Hand-wave controls, only while the robot is standing still between runs
        bool running = Actions::isRunActive();
        if (was_running && !running) {
            Actions::onRunEnded(g_run_aborted.exchange(false)); // LED result + speed tier up / down
        }
#if !ENABLE_GESTURE_UI && AUTO_START_DELAY_S > 0
        // Hand waves are switched off: start one search by itself shortly after power-on.
        // Put the robot in the start cell, facing into the maze, before switching it on.
        static bool auto_started = false;
        if (!auto_started && !running && millis() > AUTO_START_DELAY_S * 1000UL) {
            auto_started = true;
            Serial.println("[UI] Hand waves are off: calibrating IR and starting the search by itself.");
            StatusLED::flash(StatusLED::WHITE, 3, 200);
            if (!Actions::calibrateIR()) {
                Serial.println("[UI] IR calibration failed here; carrying on with the stored / default levels.");
            }
            Actions::selectMode(Actions::MODE_SEARCH);
            Actions::launchSelectedRun();
            running = Actions::isRunActive();
        }
#endif
#if ENABLE_GESTURE_UI
        if (was_running && !running) {
            GestureUI::restart(); // Re-learn what the front sensors see where the robot stopped
        }
        if (!running) {
            GestureUI::update(telemetry.ir);
        }
#endif
        was_running = running;

        vTaskDelay(pdMS_TO_TICKS(running ? 10 : 20));
    }
}

// ==============================================================================
// CORE 0 TASK: SERIAL TELEMETRY & DIAGNOSTICS STREAM
// ==============================================================================
void telemetryTask(void* pvParameters) {
    uint32_t pass = 0, tick = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10));

#if ENABLE_WIFI_OTA
        // The phone app, Telnet and over-the-air updates are served here, not in the navigation
        // or operator task: a Wi-Fi exchange can take tens of milliseconds.
        WifiOTA::handle();
#endif
        if (++tick % 5 != 0) continue; // The rest runs every 50 ms

        // Forward everything printed since last time to whoever is listening wirelessly
        static char chunk[257];
        size_t n = g_debug_log.drain(chunk, sizeof(chunk) - 1);
        if (n > 0) {
            chunk[n] = '\0';
#if ENABLE_BLE_DEBUG
            if (BLEDebug::isConnected()) BLEDebug::print(chunk);
#endif
#if ENABLE_WIFI_OTA
            if (WifiOTA::isClientConnected()) WifiOTA::print(chunk);
            WifiOTA::appendWebLog(chunk, n);
#endif
        }

        if (++pass % 4 != 0) continue; // The rest runs at 5 Hz

        RobotTelemetry snap;
        if (!getTelemetry(snap)) continue;

#if ENABLE_BLE_DEBUG
        // Readable characteristics polled by the web dashboard (tools/web_dashboard)
        BLEDebug::updateTelemetry(snap.vbat_volts, snap.imu.heading_deg,
                                  (long)snap.encoders.left_ticks_total,
                                  (long)snap.encoders.right_ticks_total);
#endif

        // The live readings line, when switched on with the console command "stream"
        if (isTelemetryStreaming()) {
            Serial.printf("[TEL] VBat: %4.2fV | Loop: %3uus | Enc: L=%6.1f R=%6.1f mm | Spd: %5.1f mm/s | Hdg: %5.1f° | IR: L90=%3d L45=%3d FL=%3d FR=%3d R45=%3d R90=%3d | Walls: [%c%c%c]\n",
                          snap.vbat_volts,
                          (unsigned int)snap.timing.loop_time_us,
                          snap.encoders.left_dist_mm,
                          snap.encoders.right_dist_mm,
                          snap.encoders.linear_speed_mm_s,
                          snap.imu.heading_deg,
                          snap.ir.left_90, snap.ir.left_45, snap.ir.front_left, snap.ir.front_right, snap.ir.right_45, snap.ir.right_90,
                          snap.ir.wall_left ? 'L' : '.',
                          snap.ir.wall_front ? 'F' : '.',
                          snap.ir.wall_right ? 'R' : '.');
        }
    }
}

// ==============================================================================
// SETUP & SYSTEM INITIALIZATION
// ==============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("\n==================================================");
    Serial.println("  ANTIGRAVITIEEE MICROMOUSE - REVISION 1.0       ");
    Serial.println("==================================================");

    // 1. Status LED on the ESP32 board
    StatusLED::begin();
    StatusLED::set(StatusLED::BLUE); // Blue = Initializing

    // 2. Initialize Shared I2C Bus (SDA = GPIO21, SCL = GPIO20)
    Serial.println("[INIT] Initializing I2C Bus (SDA: 21, SCL: 20 @ 400kHz)...");
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_SPEED);
    Wire.setTimeOut(I2C_TIMEOUT_MS);

    // 3. Initialize Hardware Drivers
    Serial.println("[INIT] Initializing N20 PCNT Hardware Encoders (30:1, 840 CPR)...");
    g_encoders.begin();

    Serial.println("[INIT] Initializing Pololu Motoron M2T256 Motor Driver...");
    g_motors.begin();

    Serial.println("[INIT] Initializing 6-Channel SFH4545/TEFT4300 IR System...");
    g_ir_sensors.begin();

    Serial.println("[INIT] Initializing Bosch BNO055 IMU...");
    g_imu.begin();

    // Power-on self-check: the robot still boots with a missing peripheral, but says so clearly
    if (!g_motors.isConnected()) {
        Serial.println("[INIT] WARNING: Motor driver not responding -> robot cannot drive until it comes back.");
        StatusLED::flash(StatusLED::RED, 5, 120);
    }
    if (!g_imu.isHardwareConnected()) {
        Serial.println("[INIT] WARNING: IMU missing -> heading runs on encoder odometry (reduced turn accuracy).");
        StatusLED::flash(StatusLED::YELLOW, 3, 120);
    }

    Serial.println("[INIT] Initializing Cascaded Motion Controller...");
    g_motion_controller.begin();

    // 4. FreeRTOS Inter-Task Communication
    g_motion_cmd_queue = xQueueCreate(4, sizeof(MotionCommand));
    g_telemetry_mutex  = xSemaphoreCreateMutex();
    g_motion_done_queue = xQueueCreate(1, sizeof(MotionDone)); // Length 1: only the latest finish matters
    g_navigator_mutex   = xSemaphoreCreateMutex();

    // 5. Initialize Navigator
    g_navigator = new Navigator(g_motion_cmd_queue, nullptr);
    g_navigator->begin();

    // 6. Wireless links
#if ENABLE_BLE_DEBUG
    Serial.println("[INIT] Starting Nordic UART Bluetooth Low Energy Service...");
    BLEDebug::begin(BLE_DEVICE_NAME);
#endif
#if ENABLE_WIFI_OTA
    Serial.println("[INIT] Starting Wi-Fi Wireless Hotspot, ArduinoOTA & Telnet Console...");
    WifiOTA::begin();
    WifiOTA::setStatusProvider(buildAppStatus);
#endif

    // 7. Spawn FreeRTOS Tasks
    Serial.println("[INIT] Launching FreeRTOS tasks: motion (core 1), navigation, operator, telemetry (core 0)...");
    xTaskCreatePinnedToCore(motionControlTask, "MotionCtrl", 4096, nullptr, PRIORITY_MOTION_TASK, nullptr, CORE_MOTION_CONTROL);
    xTaskCreatePinnedToCore(navigationTask,    "NavTask",    6144, nullptr, PRIORITY_NAV_TASK,    nullptr, CORE_NAVIGATION);
    xTaskCreatePinnedToCore(operatorTask,      "Operator",   8192, nullptr, PRIORITY_OPERATOR_TASK, nullptr, CORE_NAVIGATION);
    xTaskCreatePinnedToCore(telemetryTask,     "Telemetry",  8192, nullptr, PRIORITY_TELEMETRY,   nullptr, CORE_NAVIGATION);

    Serial.println("[READY] Antigravitieee is Ready!");
#if !ENABLE_GESTURE_UI
    Serial.printf("[UI] HAND WAVES ARE OFF. A search starts by itself %d s after power-on.\n", (int)AUTO_START_DELAY_S);
#endif
    Serial.println("[UI] Wave a hand in front of the front sensors, then pause:");
    Serial.println("  1 wave  = Search run          2 waves = Speed run (hybrid)");
    Serial.println("  3 waves = Speed run (diag)    4 waves = Speed run (curves)");
    Serial.println("  5 waves = Calibrate IR        6 waves = Clear saved maze");
    Serial.println("  Cover the sensors during the blinking countdown to cancel.");
    Serial.println("  To halt a run: lift the robot and turn it sideways (or send 'stop' when debugging).");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
