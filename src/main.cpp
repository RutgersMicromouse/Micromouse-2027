// ==============================================================================
// ANTIGRAVITIEEE MICROMOUSE - FIRMWARE ENTRY POINT
//
// The ESP32-S3 has two cores, and this file gives each one a job:
//
//   Core 1  motionControlTask   500 Hz: read sensors, run the PID loops, drive the motors
//   Core 0  navigationTask      decide where to go, listen for hand waves and text commands
//   Core 0  telemetryTask       copies everything printed to Bluetooth / Telnet; 5 Hz readings line when asked
//
// There are no buttons: power the robot on and control it with hand waves (GestureUI in ui.h).
// The LED on the ESP32 board shows what it is doing. The text console (Console in ui.h) is for
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
#include "ui.h"
#include "wireless.h"

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

// Given by the motion task each time a motion command finishes
static SemaphoreHandle_t g_motion_done_sem = nullptr;

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

    char buf[620];
    snprintf(buf, sizeof(buf),
             "{\"state\":\"%s\",\"mode\":\"%s\",\"tier\":%d,\"cell\":\"(%d, %d) facing %s\",\"visited\":%d,"
             "\"ir\":[%d,%d,%d,%d,%d,%d],\"walls\":\"%c%c%c\","
             "\"heading\":%.1f,\"vbat\":%.2f,\"encL\":\"%ld (%.0f mm)\",\"encR\":\"%ld (%.0f mm)\","
             "\"motor\":\"%s\",\"imu\":\"%s\",\"supply\":%.1f,\"loop\":%u}",
             Actions::stateDescription(), Actions::modeName(Actions::getSelectedMode()), (int)Actions::getSpeedTier(),
             (int)pose.cell_x, (int)pose.cell_y, kCompass[pose.current_dir % 4], cells_seen,
             t.ir.left_90, t.ir.left_45, t.ir.front_left, t.ir.front_right, t.ir.right_45, t.ir.right_90,
             t.ir.wall_left ? 'L' : '.', t.ir.wall_front ? 'F' : '.', t.ir.wall_right ? 'R' : '.',
             t.imu.heading_deg, t.vbat_volts,
             (long)t.encoders.left_ticks_total, t.encoders.left_dist_mm,
             (long)t.encoders.right_ticks_total, t.encoders.right_dist_mm,
             g_motors.isConnected() ? "OK" : "NOT RESPONDING",
             !g_imu.isHardwareConnected() ? "MISSING" : (g_imu.isUsingFallback() ? "FAULT" : "OK"),
             g_motors.getSupplyVolts(), (unsigned int)t.timing.loop_time_us);
    return String(buf);
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

        // 5. Signal movement completion
        if (was_busy && g_motion_controller.isCommandFinished()) {
            was_busy = false;
            xSemaphoreGive(g_motion_done_sem);
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
// CORE 0 TASK: HIGH-LEVEL NAVIGATION & OPERATOR CONTROLS
// ==============================================================================
void navigationTask(void* pvParameters) {
    Serial.printf("[CORE 0] Navigation Task running on Core %d\n", xPortGetCoreID());

    static RobotTelemetry telemetry = {}; // Kept between loops so a missed snapshot reuses the last one
    bool was_running = false;
    bool run_aborted = false;
    NavState last_state = NAV_STATE_ERROR; // Anything but the real one, so the first state is announced

    Actions::showSelectedMode();
    GestureUI::restart();

    for (;;) {
        // 1. Debug console on Bluetooth / Telnet / USB (also keeps Wi-Fi OTA alive)
        Console::poll();

        getTelemetry(telemetry);

        // 2. Automatic safety stop (motor stall / dead encoder): abort the run, don't send the next move
        if (g_motion_controller.consumeSafetyStop()) {
            Serial.println("\n[SAFETY] Run aborted by the motion controller (motor stall or encoder fault).");
            g_navigator->stop();
            xSemaphoreTake(g_motion_done_sem, 0);
            run_aborted = true;
        }

        // 3. Advance the navigator strictly when a physical motion completes
        if (xSemaphoreTake(g_motion_done_sem, 0) == pdTRUE) {
            g_navigator->notifyMotionComplete();
            g_navigator->step(telemetry.ir, g_motion_controller.getWallPreview());
        }

        // Say what the robot is doing whenever that changes
        if (g_navigator->getState() != last_state) {
            last_state = g_navigator->getState();
            Serial.printf("[STATE] %s\n", Actions::stateDescription());
        }

        // 4. Hand-wave controls, only while the robot is standing still between runs
        bool running = Actions::isRunActive();
        if (was_running && !running) {
            Actions::onRunEnded(run_aborted); // LED result + speed tier up / down
            run_aborted = false;
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

        // Fast 1ms yield during active runs for seamless motion chaining; 20ms during idle
        vTaskDelay(running ? pdMS_TO_TICKS(1) : pdMS_TO_TICKS(20));
    }
}

// ==============================================================================
// CORE 0 TASK: SERIAL TELEMETRY & DIAGNOSTICS STREAM
// ==============================================================================
void telemetryTask(void* pvParameters) {
    uint32_t pass = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));

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
    g_motion_done_sem  = xSemaphoreCreateBinary();

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
    Serial.println("[INIT] Launching FreeRTOS Core 1 & Core 0 Tasks...");
    xTaskCreatePinnedToCore(motionControlTask, "MotionCtrl", 4096, nullptr, PRIORITY_MOTION_TASK, nullptr, CORE_MOTION_CONTROL);
    xTaskCreatePinnedToCore(navigationTask,    "NavTask",    6144, nullptr, PRIORITY_NAV_TASK,    nullptr, CORE_NAVIGATION);
    xTaskCreatePinnedToCore(telemetryTask,     "Telemetry",  4096, nullptr, PRIORITY_TELEMETRY,   nullptr, CORE_NAVIGATION);

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
