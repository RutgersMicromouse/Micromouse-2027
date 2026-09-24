#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "config.h"
#include "types.h"

#include "hardware/encoders.h"
#include "hardware/motors.h"
#include "hardware/ir_sensors.h"
#include "hardware/imu.h"

#include "control/motion_controller.h"
#include "navigation/navigator.h"
#include "hardware/ble_debug.h"
#include "hardware/wifi_ota.h"

// ==============================================================================
// GLOBAL HARDWARE & CONTROL INSTANCES
// ==============================================================================
static Encoders         g_encoders;
static Motors           g_motors;
static IRSensors        g_ir_sensors;
static IMU              g_imu;
static MotionController g_motion_controller(g_encoders, g_motors, g_ir_sensors, g_imu);

// FreeRTOS Inter-Task Communication
static QueueHandle_t     g_motion_cmd_queue = nullptr;
static QueueHandle_t     g_telemetry_queue  = nullptr;
static SemaphoreHandle_t g_telemetry_mutex  = nullptr;
static SemaphoreHandle_t g_motion_done_sem  = nullptr;

static RobotTelemetry    g_shared_telemetry;
static Navigator*        g_navigator        = nullptr;

// ==============================================================================
// RGB STATUS LED HELPER (Active HIGH via TJ-L5FCMXHTCSLCRGB-A5 to GND)
// ==============================================================================
void setRGB(bool red, bool green, bool blue) {
    digitalWrite(PIN_LED_RED,   red   ? HIGH : LOW);
    digitalWrite(PIN_LED_GREEN, green ? HIGH : LOW);
    digitalWrite(PIN_LED_BLUE,  blue  ? HIGH : LOW);
}

void flashRGB(bool red, bool green, bool blue, int count = 3, int delay_ms = 150) {
    for (int i = 0; i < count; ++i) {
        setRGB(red, green, blue);
        delay(delay_ms);
        setRGB(false, false, false);
        delay(delay_ms);
    }
}

void updateModeLED(uint8_t mode) {
    switch (mode) {
        case 0: setRGB(false, true, false); break; // Green (Search / Exploration)
        case 1: setRGB(true, true, false);  break; // Yellow (Hybrid Auto-Optimizer)
        case 2: setRGB(false, true, true);  break; // Cyan (Pure Diagonal Specialist)
        case 3: setRGB(true, false, true);  break; // Magenta (Pure Continuous Curves)
        default: setRGB(false, false, true); break; // Blue (Ready / Idle)
    }
}

void launchRunForMode(uint8_t mode, const IRReadings& ir_snapshot) {
    if (mode == 0) {
        Serial.println("[UI] CONFIRMED -> Launching Search Run!");
        g_navigator->startSearchRun();
        g_navigator->step(ir_snapshot);
        updateModeLED(0);
    } else if (mode == 1) {
        Serial.println("[UI] CONFIRMED -> Launching SPEED RUN: ⚡ HYBRID AUTO-OPTIMIZER!");
        g_navigator->startSpeedRun(SPEEDRUN_HYBRID_AUTO);
        updateModeLED(1);
    } else if (mode == 2) {
        Serial.println("[UI] CONFIRMED -> Launching SPEED RUN: 📐 PURE DIAGONAL SPECIALIST!");
        g_navigator->startSpeedRun(SPEEDRUN_DIAGONALS_ONLY);
        updateModeLED(2);
    } else {
        Serial.println("[UI] CONFIRMED -> Launching SPEED RUN: 🏎 PURE CONTINUOUS CURVES!");
        g_navigator->startSpeedRun(SPEEDRUN_CURVES_ONLY);
        updateModeLED(3);
    }
}

// Unified wireless command dispatcher (BLE + Wi-Fi Telnet)
void handleRemoteCommand(String cmd, uint8_t& selected_mode, bool is_active_run, const IRReadings& ir_snapshot, bool is_ble) {
    cmd.toLowerCase();
    cmd.trim();
    Serial.printf("[%s CMD] Received: '%s'\n", is_ble ? "BLE" : "TELNET", cmd.c_str());

    auto reply = [&](const char* msg) {
#if ENABLE_BLE_DEBUG
        if (is_ble) BLEDebug::println(msg);
#endif
#if ENABLE_WIFI_OTA
        if (!is_ble) WifiOTA::println(msg);
#endif
    };

    if (cmd == "stop" || cmd == "estop" || cmd == "halt") {
        Serial.println("[REMOTE] 🛑 Emergency stop triggered wirelessly!");
        g_navigator->stop();
        flashRGB(true, false, false, 3, 100);
        updateModeLED(selected_mode);
        reply("ACK: STOPPED");
    } else if (cmd == "start" || cmd == "go") {
        if (!is_active_run) {
            launchRunForMode(selected_mode, ir_snapshot);
            char buf[32];
            snprintf(buf, sizeof(buf), "ACK: STARTED MODE %d", selected_mode);
            reply(buf);
        } else {
            reply("ERR: ALREADY_RUNNING");
        }
    } else if (cmd == "mode 0" || cmd == "search") {
        selected_mode = 0;
        updateModeLED(selected_mode);
        reply("ACK: MODE 0 (SEARCH - Green LED)");
    } else if (cmd == "mode 1" || cmd == "hybrid") {
        selected_mode = 1;
        updateModeLED(selected_mode);
        reply("ACK: MODE 1 (HYBRID - Yellow LED)");
    } else if (cmd == "mode 2" || cmd == "diag") {
        selected_mode = 2;
        updateModeLED(selected_mode);
        reply("ACK: MODE 2 (DIAGONALS - Cyan LED)");
    } else if (cmd == "mode 3" || cmd == "curve") {
        selected_mode = 3;
        updateModeLED(selected_mode);
        reply("ACK: MODE 3 (CURVES - Magenta LED)");
    } else if (cmd == "calib") {
        if (!is_active_run) {
            reply("Starting in-cell IR auto-calibration...");
            setRGB(true, true, false);
            bool ok = g_ir_sensors.calibrateInCell(200);
            if (ok) {
                flashRGB(false, true, false, 3, 120);
                reply("ACK: CALIB SUCCESS");
            } else {
                flashRGB(true, false, false, 3, 120);
                reply("ERR: CALIB FAILED");
            }
            updateModeLED(selected_mode);
        } else {
            reply("ERR: CANNOT_CALIB_WHILE_RUNNING");
        }
    } else if (cmd == "motorcal") {
        if (!is_active_run) {
            reply("Starting automated motor speed calibration (wheels must be freewheeling)...");
            setRGB(true, true, false); // Yellow during calibration
            bool ok = g_motion_controller.calibrateMotors();
            if (ok) {
                flashRGB(false, true, false, 4, 100); // Green
                float tl = 1.0f, tr = 1.0f;
                g_motors.getTrim(tl, tr);
                char buf[64];
                snprintf(buf, sizeof(buf), "ACK: MOTOR CALIB SUCCESS (L=%.4f, R=%.4f)", tl, tr);
                reply(buf);
            } else {
                flashRGB(true, false, false, 4, 100); // Red
                reply("ERR: MOTOR CALIB FAILED (check wheels or battery)");
            }
            updateModeLED(selected_mode);
        } else {
            reply("ERR: CANNOT_CALIB_WHILE_RUNNING");
        }
    } else if (cmd.startsWith("motorrpm")) {
        if (!is_active_run) {
            float duty = 0.5f;
            int space_idx = cmd.indexOf(' ');
            if (space_idx > 0) {
                float parsed = cmd.substring(space_idx + 1).toFloat();
                if (parsed > 0.05f && parsed <= 1.0f) {
                    duty = parsed;
                }
            }
            char buf[64];
            snprintf(buf, sizeof(buf), "Running tachometer benchmark at %.0f%% duty for 4000ms...", duty * 100.0f);
            reply(buf);
            setRGB(false, true, true); // Cyan
            g_motion_controller.runTachometerBenchmark(duty, 4000);
            reply("ACK: TACH BENCHMARK FINISHED");
            updateModeLED(selected_mode);
        } else {
            reply("ERR: CANNOT_BENCHMARK_WHILE_RUNNING");
        }
    } else if (cmd.startsWith("motortrim")) {
        int space_idx = cmd.indexOf(' ');
        if (space_idx > 0) {
            float tl = 1.0f, tr = 1.0f;
            if (sscanf(cmd.c_str() + space_idx + 1, "%f %f", &tl, &tr) == 2) {
                g_motors.setTrim(tl, tr);
                g_motors.saveToNVS();
                char buf[64];
                snprintf(buf, sizeof(buf), "ACK: TRIM UPDATED & SAVED (L=%.4f, R=%.4f)", tl, tr);
                reply(buf);
            } else {
                reply("ERR: USAGE 'motortrim <left> <right>'");
            }
        } else {
            float tl = 1.0f, tr = 1.0f;
            g_motors.getTrim(tl, tr);
            char buf[64];
            snprintf(buf, sizeof(buf), "MOTORS TRIM: Left=%.4f | Right=%.4f", tl, tr);
            reply(buf);
        }
    } else if (cmd == "clear") {
        if (!is_active_run) {
            g_navigator->clearSavedMaze();
            flashRGB(false, false, true, 4, 100);
            updateModeLED(selected_mode);
            reply("ACK: MAZE CLEARED");
        } else {
            reply("ERR: CANNOT_CLEAR_WHILE_RUNNING");
        }
    } else if (cmd == "status") {
        float vbat = (float)analogReadMilliVolts(PIN_VSENSE_COM) / 1000.0f * BATTERY_DIVIDER_RATIO;
        char buf[64];
        snprintf(buf, sizeof(buf), "STATUS: VBat=%.2fV | Mode=%d | State=%d", vbat, selected_mode, (int)g_navigator->getState());
        reply(buf);
    } else if (cmd == "help") {
        reply("Commands: start, stop, search, hybrid, diag, curve, calib, clear, status, motorcal, motorrpm [duty], motortrim [l r]");
    } else {
        char buf[64];
        snprintf(buf, sizeof(buf), "ERR: UNKNOWN COMMAND '%s' (type 'help')", cmd.c_str());
        reply(buf);
    }
}

// Battery Voltage Sense (10k / 10k divider on Computer Battery)
float readBatteryVoltage() {
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

    Serial.printf("[CORE 1] Real-Time Motion Task running on Core %d @ %d Hz\n",
                  xPortGetCoreID(), CONTROL_LOOP_FREQ_HZ);

    for (;;) {
        vTaskDelayUntil(&last_wake_time, period_ticks > 0 ? period_ticks : 1);

        // 1. Read Encoders (PCNT hardware 4x decoding) & Pulsed 5-Channel IR
        g_encoders.update(CONTROL_DT_S);
        g_ir_sensors.update();

        EncoderState enc = g_encoders.getState();

        // 2. High-rate IMU sensor fusion (500 Hz continuous dead-reckoning + 100 Hz I2C poll)
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
            if (g_motion_done_sem) {
                xSemaphoreGive(g_motion_done_sem);
            }
        }

        // 6. Update Telemetry Snapshot at 50 Hz & Check Low Battery Cutoff
        if (loop_counter++ % 10 == 0) {
            float vbat = readBatteryVoltage();

            // Low-voltage battery protection (cuts 12V boost converter)
            if (vbat > 1.0f && vbat < BATTERY_MIN_SAFE_VOLT) {
                g_motion_controller.emergencyStop();
                g_motors.setMotorPowerEnabled(false);
                setRGB(true, false, false); // Solid Red Alarm
                Serial.printf("[SAFETY ALERT] Low Battery Voltage: %4.2f V! Motors disabled.\n", vbat);
            }

            if (xSemaphoreTake(g_telemetry_mutex, 0) == pdTRUE) {
                g_shared_telemetry.encoders = enc;
                g_shared_telemetry.ir = g_ir_sensors.getReadings();
                g_shared_telemetry.imu = g_imu.getState();
                g_shared_telemetry.motion_completed = g_motion_controller.isCommandFinished();
                g_shared_telemetry.vbat_volts = vbat;
                g_shared_telemetry.loop_count = loop_counter;
                xSemaphoreGive(g_telemetry_mutex);
            }
        }
    }
}

// ==============================================================================
// CORE 0 TASK: HIGH-LEVEL NAVIGATION & STATE CONTROLS
// ==============================================================================
void navigationTask(void* pvParameters) {
    Serial.printf("[CORE 0] Navigation & State Task running on Core %d\n", xPortGetCoreID());

    bool last_state_btn = HIGH;
    bool last_confirm_btn = HIGH;
    uint32_t state_press_start = 0;
#ifdef FORCE_PURE_DIAGONALS
    uint8_t selected_mode = 2; // Dedicated Pure Diagonal Specialist
    Serial.println("[UI] Default Profile: PURE DIAGONAL SPECIALIST (Cyan LED)");
#elif defined(FORCE_PURE_CURVES)
    uint8_t selected_mode = 3; // Dedicated Pure Continuous Curves
    Serial.println("[UI] Default Profile: PURE CONTINUOUS CURVES (Magenta LED)");
#elif defined(FORCE_HYBRID_AUTO)
    uint8_t selected_mode = 1; // Dedicated Hybrid Auto-Optimizer
    Serial.println("[UI] Default Profile: HYBRID AUTO-OPTIMIZER (Yellow LED)");
#else
    uint8_t selected_mode = 0; // Search Run
    Serial.println("[UI] Default Profile: SEARCH / EXPLORATION RUN (Green LED)");
#endif
    updateModeLED(selected_mode);

    uint32_t confirm_press_start = 0;

    for (;;) {
        bool curr_state_btn = digitalRead(PIN_BTN_STATE);
        bool curr_confirm_btn = digitalRead(PIN_BTN_CONFIRM);

        // Fetch latest IR readings for navigation
        IRReadings ir_snapshot;
        if (xSemaphoreTake(g_telemetry_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            ir_snapshot = g_shared_telemetry.ir;
            xSemaphoreGive(g_telemetry_mutex);
        }

        NavState cur_nav_state = g_navigator->getState();
        bool is_active_run = (cur_nav_state == NAV_STATE_EXPLORING_TO_CENTER ||
                              cur_nav_state == NAV_STATE_RETURNING_TO_START ||
                              cur_nav_state == NAV_STATE_SPEED_RUNNING);

#if ENABLE_BLE_DEBUG
        // --- BLUETOOTH LOW ENERGY REMOTE CONTROL COMMANDS ---
        if (BLEDebug::hasCommand()) {
            handleRemoteCommand(BLEDebug::readCommand(), selected_mode, is_active_run, ir_snapshot, true);
        }
#endif

#if ENABLE_WIFI_OTA
        // --- WI-FI OTA UPDATE HANDLER & TELNET CONSOLE COMMANDS ---
        WifiOTA::handle();
        if (WifiOTA::hasCommand()) {
            handleRemoteCommand(WifiOTA::readCommand(), selected_mode, is_active_run, ir_snapshot, false);
        }
#endif

        // --- USB SERIAL CONSOLE COMMANDS ---
        if (Serial.available()) {
            String serial_cmd = Serial.readStringUntil('\n');
            serial_cmd.trim();
            if (serial_cmd.length() > 0) {
                handleRemoteCommand(serial_cmd, selected_mode, is_active_run, ir_snapshot, false);
            }
        }

        // --- MANUAL E-STOP / PAUSE WHILE MOVING ---
        // Tapping either button while the robot is running immediately brakes and halts navigation
        if (is_active_run) {
            if ((last_state_btn == HIGH && curr_state_btn == LOW) ||
                (last_confirm_btn == HIGH && curr_confirm_btn == LOW)) {
                Serial.println("\n[UI] 🛑 USER EMERGENCY STOP ENGAGED! Halting robot.");
                g_navigator->stop();
                flashRGB(true, false, false, 3, 100); // Flash Red
                updateModeLED(selected_mode);
                last_state_btn = curr_state_btn;
                last_confirm_btn = curr_confirm_btn;
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }
        } else {
            // --- STATE BUTTON (when idle): Short press = cycle mode, Long press (>2s) = Auto-Calibrate ---
            if (last_state_btn == HIGH && curr_state_btn == LOW) {
                state_press_start = millis();
            } else if (last_state_btn == LOW && curr_state_btn == HIGH) {
                uint32_t hold_time = millis() - state_press_start;
                if (hold_time > 2000) {
                    // Long press: trigger in-cell auto-calibration
                    Serial.println("\n[UI] Long Press STATE Detected -> Starting IR Auto-Calibration!");
                    setRGB(true, true, false); // Yellow during calibration

                    bool success = g_ir_sensors.calibrateInCell(200);
                    if (success) {
                        flashRGB(false, true, false, 3, 120); // Flash Green = Success
                    } else {
                        flashRGB(true, false, false, 3, 120); // Flash Red = Failed
                    }
                    updateModeLED(selected_mode);
                } else if (hold_time > 50) {
                    // Short press: cycle between 4 operating modes
                    selected_mode = (selected_mode + 1) % 4;
                    updateModeLED(selected_mode);
                    switch (selected_mode) {
                        case 0: Serial.println("[UI] Selected Mode [0]: SEARCH / EXPLORATION RUN (Green LED)"); break;
                        case 1: Serial.println("[UI] Selected Mode [1]: SPEED RUN -> ⚡ HYBRID AUTO-OPTIMIZER (Yellow LED)"); break;
                        case 2: Serial.println("[UI] Selected Mode [2]: SPEED RUN -> 📐 PURE DIAGONAL SPECIALIST (Cyan LED)"); break;
                        case 3: Serial.println("[UI] Selected Mode [3]: SPEED RUN -> 🏎 PURE CONTINUOUS CURVES (Magenta LED)"); break;
                    }
                }
            }

            // --- CONFIRM BUTTON (when idle): Short press = Launch run, Long press (>2.5s) = Clear Flash Maze ---
            if (last_confirm_btn == HIGH && curr_confirm_btn == LOW) {
                confirm_press_start = millis();
            } else if (last_confirm_btn == LOW && curr_confirm_btn == HIGH) {
                uint32_t confirm_hold = millis() - confirm_press_start;
                if (confirm_hold > 2500) {
                    // Long press CONFIRM: Clear mapped maze in NVS
                    Serial.println("\n[UI] Long Press CONFIRM Detected -> Clearing Saved Flash Maze!");
                    g_navigator->clearSavedMaze();
                    flashRGB(false, false, true, 4, 100); // Flash Blue = Cleared
                    updateModeLED(selected_mode);
                } else if (confirm_hold > 50) {
                    // Short press CONFIRM: Launch selected run
                    launchRunForMode(selected_mode, ir_snapshot);
                }
            }
        }

        last_state_btn = curr_state_btn;
        last_confirm_btn = curr_confirm_btn;

        // Advance FSM strictly when physical motion completes
        TickType_t sem_wait = is_active_run ? pdMS_TO_TICKS(5) : 0;
        if (xSemaphoreTake(g_motion_done_sem, sem_wait) == pdTRUE) {
            g_navigator->notifyMotionComplete();
            g_navigator->step(ir_snapshot);
        }

        // Fast 1ms yield during active runs for seamless motion chaining; 20ms during idle
        vTaskDelay(is_active_run ? pdMS_TO_TICKS(1) : pdMS_TO_TICKS(20));
    }
}

// ==============================================================================
// CORE 0 TASK: SERIAL TELEMETRY & DIAGNOSTICS STREAM
// ==============================================================================
void telemetryTask(void* pvParameters) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(200)); // 5 Hz telemetry

        RobotTelemetry snap;
        if (xSemaphoreTake(g_telemetry_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            snap = g_shared_telemetry;
            xSemaphoreGive(g_telemetry_mutex);

            Serial.printf("[TEL] VBat: %4.2fV | Enc: L=%6.1f R=%6.1f mm | Spd: %5.1f mm/s | Hdg: %5.1f° | IR: L90=%3d L45=%3d FL=%3d FR=%3d R45=%3d R90=%3d | Walls: [%c%c%c]\n",
                          snap.vbat_volts,
                          snap.encoders.left_dist_mm,
                          snap.encoders.right_dist_mm,
                          snap.encoders.linear_speed_mm_s,
                          snap.imu.heading_deg,
                          snap.ir.left_90, snap.ir.left_45, snap.ir.front_left, snap.ir.front_right, snap.ir.right_45, snap.ir.right_90,
                          snap.ir.wall_left ? 'L' : '.',
                          snap.ir.wall_front ? 'F' : '.',
                          snap.ir.wall_right ? 'R' : '.');

#if ENABLE_BLE_DEBUG
            if (BLEDebug::isConnected()) {
                char ble_buf[128];
                snprintf(ble_buf, sizeof(ble_buf), "V:%.2fV|Spd:%.0f|Hdg:%.1f|IR:%d,%d,%d,%d,%d,%d|W:[%c%c%c]",
                         snap.vbat_volts,
                         snap.encoders.linear_speed_mm_s,
                         snap.imu.heading_deg,
                         snap.ir.left_90, snap.ir.left_45, snap.ir.front_left, snap.ir.front_right, snap.ir.right_45, snap.ir.right_90,
                         snap.ir.wall_left ? 'L' : '.',
                         snap.ir.wall_front ? 'F' : '.',
                         snap.ir.wall_right ? 'R' : '.');
                BLEDebug::println(ble_buf);
            }
#endif

#if ENABLE_WIFI_OTA
            if (WifiOTA::isClientConnected()) {
                char telnet_buf[160];
                snprintf(telnet_buf, sizeof(telnet_buf), "[TEL] VBat: %4.2fV | Enc: L=%6.1f R=%6.1f mm | Spd: %5.1f mm/s | Hdg: %5.1f° | IR: L90=%3d L45=%3d FL=%3d FR=%3d R45=%3d R90=%3d | Walls: [%c%c%c]\r\n",
                         snap.vbat_volts,
                         snap.encoders.left_dist_mm,
                         snap.encoders.right_dist_mm,
                         snap.encoders.linear_speed_mm_s,
                         snap.imu.heading_deg,
                         snap.ir.left_90, snap.ir.left_45, snap.ir.front_left, snap.ir.front_right, snap.ir.right_45, snap.ir.right_90,
                         snap.ir.wall_left ? 'L' : '.',
                         snap.ir.wall_front ? 'F' : '.',
                         snap.ir.wall_right ? 'R' : '.');
                WifiOTA::print(telnet_buf);
            }
#endif
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

    // 1. Initialize UI Controls & Indicators
    pinMode(PIN_BTN_CONFIRM, INPUT_PULLUP);
    pinMode(PIN_BTN_STATE, INPUT_PULLUP);

    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_BLUE, OUTPUT);
    setRGB(false, false, true); // Blue = Initializing

    // 2. Initialize Shared I2C Bus (SDA = GPIO21, SCL = GPIO20)
    Serial.println("[INIT] Initializing I2C Bus (SDA: 21, SCL: 20 @ 400kHz)...");
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_CLOCK_SPEED);

    // 3. Initialize Hardware Drivers
    Serial.println("[INIT] Initializing SN74LVC125 PCNT Hardware Encoders...");
    g_encoders.begin();

    Serial.println("[INIT] Initializing Pololu Motoron M2T256 Motor Driver...");
    g_motors.begin();

    Serial.println("[INIT] Initializing 5-Channel SFH4545/TEFT4300 IR System...");
    g_ir_sensors.begin();

    Serial.println("[INIT] Initializing Bosch BNO055 IMU...");
    g_imu.begin();

    Serial.println("[INIT] Initializing Cascaded Motion Controller...");
    g_motion_controller.begin();

    // 4. FreeRTOS Inter-Task Communication
    g_motion_cmd_queue = xQueueCreate(4, sizeof(MotionCommand));
    g_telemetry_mutex  = xSemaphoreCreateMutex();
    g_motion_done_sem  = xSemaphoreCreateBinary();

    // 5. Initialize Navigator
    g_navigator = new Navigator(g_motion_cmd_queue, g_telemetry_queue);
    g_navigator->begin();

    // 6. Spawn FreeRTOS Tasks
    Serial.println("[INIT] Launching FreeRTOS Core 1 & Core 0 Tasks...");

    xTaskCreatePinnedToCore(
        motionControlTask,
        "MotionCtrl",
        4096,
        nullptr,
        PRIORITY_MOTION_TASK,
        nullptr,
        CORE_MOTION_CONTROL
    );

    xTaskCreatePinnedToCore(
        navigationTask,
        "NavTask",
        4096,
        nullptr,
        PRIORITY_NAV_TASK,
        nullptr,
        CORE_NAVIGATION
    );

    xTaskCreatePinnedToCore(
        telemetryTask,
        "Telemetry",
        3072,
        nullptr,
        PRIORITY_TELEMETRY,
        nullptr,
        CORE_NAVIGATION
    );

    Serial.println("[READY] Antigravitieee is Ready!");
    Serial.printf("[BATT] Computer Battery Voltage: %4.2f V\n", readBatteryVoltage());
#if ENABLE_BLE_DEBUG
    Serial.println("[INIT] Starting Nordic UART Bluetooth Low Energy Service...");
    BLEDebug::begin(BLE_DEVICE_NAME);
#endif
#if ENABLE_WIFI_OTA
    Serial.println("[INIT] Starting Wi-Fi Wireless Hotspot, ArduinoOTA & Telnet Console...");
    WifiOTA::begin();
#endif
    Serial.println("[UI] Controls Guide:");
    Serial.println("  - When Moving: Press EITHER button or send 'stop' over BLE/Telnet -> Instant Emergency Stop");
    Serial.println("  - Short Press STATE (GPIO42) or send 'search/hybrid/diag/curve'   -> Cycle Mode");
    Serial.println("  - Long Press STATE (>2 sec) or send 'calib'                       -> In-Cell IR Auto-Calibration");
    Serial.println("  - Short Press CONFIRM (GPIO41) or send 'start'                    -> Launch Selected Run");
    Serial.println("  - Long Press CONFIRM (>2.5 sec) or send 'clear'                   -> Clear Saved Maze from Flash");
    Serial.println("  - Bench Motor Calib: send 'motorcal' or 'motorrpm [duty]'         -> Auto-balance wheel RPMs & Save");
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}