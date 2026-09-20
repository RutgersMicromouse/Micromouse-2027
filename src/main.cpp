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
        g_imu.update(CONTROL_DT_S, yaw_rate);

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
    setRGB(false, true, true);  // Cyan
#elif defined(FORCE_PURE_CURVES)
    uint8_t selected_mode = 3; // Dedicated Pure Continuous Curves
    Serial.println("[UI] Default Profile: PURE CONTINUOUS CURVES (Magenta LED)");
    setRGB(true, false, true);  // Magenta
#elif defined(FORCE_HYBRID_AUTO)
    uint8_t selected_mode = 1; // Dedicated Hybrid Auto-Optimizer
    Serial.println("[UI] Default Profile: HYBRID AUTO-OPTIMIZER (Yellow LED)");
    setRGB(true, true, false);  // Yellow
#else
    uint8_t selected_mode = 0; // Search Run
    Serial.println("[UI] Default Profile: SEARCH / EXPLORATION RUN (Green LED)");
    setRGB(false, true, false); // Green
#endif

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

        // --- MANUAL E-STOP / PAUSE WHILE MOVING ---
        // Tapping either button while the robot is running immediately brakes and halts navigation
        if (is_active_run) {
            if ((last_state_btn == HIGH && curr_state_btn == LOW) ||
                (last_confirm_btn == HIGH && curr_confirm_btn == LOW)) {
                Serial.println("\n[UI] 🛑 USER EMERGENCY STOP ENGAGED! Halting robot.");
                g_navigator->stop();
                flashRGB(true, false, false, 3, 100); // Flash Red
                setRGB(false, false, true);           // Blue = Idle Ready
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
                    setRGB(false, false, true); // Return to Blue
                } else if (hold_time > 50) {
                    // Short press: cycle between 4 operating modes
                    selected_mode = (selected_mode + 1) % 4;
                    if (selected_mode == 0) {
                        Serial.println("[UI] Selected Mode [0]: SEARCH / EXPLORATION RUN");
                        setRGB(false, true, false); // Green
                    } else if (selected_mode == 1) {
                        Serial.println("[UI] Selected Mode [1]: SPEED RUN -> ⚡ HYBRID AUTO-OPTIMIZER (Curves vs Diagonals)");
                        setRGB(true, true, false);  // Yellow
                    } else if (selected_mode == 2) {
                        Serial.println("[UI] Selected Mode [2]: SPEED RUN -> 📐 PURE DIAGONAL SPECIALIST (Maximum Diagonal Sprints)");
                        setRGB(false, true, true);  // Cyan
                    } else {
                        Serial.println("[UI] Selected Mode [3]: SPEED RUN -> 🏎 PURE CONTINUOUS CURVES (Zero Diagonals)");
                        setRGB(true, false, true);  // Magenta
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
                    setRGB(false, false, true);
                } else if (confirm_hold > 50) {
                    // Short press CONFIRM: Launch selected run
                    if (selected_mode == 0) {
                        Serial.println("[UI] CONFIRMED -> Launching Search Run!");
                        g_navigator->startSearchRun();
                        g_navigator->step(ir_snapshot);
                        setRGB(false, true, false);     // Solid Green
                    } else if (selected_mode == 1) {
                        Serial.println("[UI] CONFIRMED -> Launching SPEED RUN: ⚡ HYBRID AUTO-OPTIMIZER!");
                        g_navigator->startSpeedRun(SPEEDRUN_HYBRID_AUTO);
                        setRGB(true, true, false);      // Solid Yellow
                    } else if (selected_mode == 2) {
                        Serial.println("[UI] CONFIRMED -> Launching SPEED RUN: 📐 PURE DIAGONAL SPECIALIST!");
                        g_navigator->startSpeedRun(SPEEDRUN_DIAGONALS_ONLY);
                        setRGB(false, true, true);       // Solid Cyan
                    } else {
                        Serial.println("[UI] CONFIRMED -> Launching SPEED RUN: 🏎 PURE CONTINUOUS CURVES!");
                        g_navigator->startSpeedRun(SPEEDRUN_CURVES_ONLY);
                        setRGB(true, false, true);       // Solid Magenta
                    }
                }
            }
        }

        last_state_btn = curr_state_btn;
        last_confirm_btn = curr_confirm_btn;

        // Advance FSM strictly when physical motion completes
        if (xSemaphoreTake(g_motion_done_sem, 0) == pdTRUE) {
            g_navigator->notifyMotionComplete();
            g_navigator->step(ir_snapshot);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
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
    Serial.println("[UI] Controls Guide:");
    Serial.println("  - When Moving: Press EITHER button -> Instant Emergency Stop");
    Serial.println("  - Short Press STATE (GPIO42)      -> Cycle Mode (Search / Speed Run Profiles)");
    Serial.println("  - Long Press STATE (>2 sec)       -> In-Cell IR Auto-Calibration (Saves to Flash)");
    Serial.println("  - Short Press CONFIRM (GPIO41)    -> Launch Selected Run");
    Serial.println("  - Long Press CONFIRM (>2.5 sec)   -> Clear Saved Maze from Flash NVS");
    setRGB(false, false, true); // Blue = Ready
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}