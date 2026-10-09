#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "control/motion_controller.h"
#include "navigation/navigator.h"

// =============================================================================
// Ratatouieee Micromouse Firmware - Main Entry Point
// Rutgers Micromouse 2026-2027
// Target: Teensy 4.0 (ARM Cortex-M7 @ 600MHz)
// =============================================================================

RobotState current_robot_state = STATE_IDLE;
bool continuous_telemetry = false;
uint32_t last_telemetry_time = 0;

#define DEBUG_MANUAL_MAZE_BUILD_MODE // Uncomment to enable manual 5-second step & maze building debug mode
// #define SENSOR_DISTANCE_DIAGNOSTIC_MODE
// #define RUN_MOTOR_STARTUP_TEST  // Never enable for normal gesture startup.
// Uncomment to continuously print IR raw values, distances, and wall decisions.
// #define DEBUG_IR_SENSOR_STREAM

void printBanner() {
    Serial.println("\n========================================================");
    Serial.println("       RATATOUIEEE - RUTGERS MICROMOUSE 2026-2027       ");
    Serial.println("  Hardware: Teensy 4.0 | Motoron M2T256 | MinIMU-9 v5   ");
    Serial.println("  Sensors: 5x Analog IR (FIR, L1, L2, R1, R2) | Encoders");
    Serial.println("========================================================");
}

void printHelp() {
    Serial.println("\nCommands available via Serial Console:");
    Serial.println("  [e] - Explore to Center (Floodfill)");
    Serial.println("  [n] - Explore one cell, then wait for the next command");
    Serial.println("  [r] - Return to Start (0, 0)");
    Serial.println("  [f] - Execute High-Speed Speed Run");
    Serial.println("  [a] - Full Autonomy (Explore -> Return -> Speed Run)");
    Serial.println("  [c] - In-Cell IR Auto-Calibration");
    Serial.println("  [d] - Toggle Real-Time Diagnostic Stream");
    Serial.println("  [m] - Print 16x16 Maze ASCII Map");
    Serial.println("  [t] - Test 90-deg in-place turn");
    Serial.println("  [w] - Test 1-cell forward move");
    Serial.println("  [1] - Test left wheel only (lift robot first)");
    Serial.println("  [2] - Test right wheel only (lift robot first)");
    Serial.println("  [s] - Emergency Stop\n");
}

void testSingleWheel(bool right) {
    if (!motors.isConnected()) {
        Serial.println("[MOTOR TEST] Motoron is not connected; wheel test cancelled.");
        return;
    }

    Serial.printf("[MOTOR TEST] Testing %s wheel forward for 1 second. Keep the robot lifted.\n",
                  right ? "right" : "left");
    motors.stop(true);
    delay(100);
    if (right) {
        motors.setRightSpeed(250);
    } else {
        motors.setLeftSpeed(250);
    }
    delay(1000);
    motors.stop(true);
    Serial.printf("[MOTOR TEST] %s wheel test complete; motors stopped.\n",
                  right ? "Right" : "Left");
}

void streamDiagnostics() {
    DistanceSensors ir = ir_sensors.getReadings();
    Serial.printf("[DIAG] MOTOR BAT: %4.2fV | ENC: L=%6.1fmm R=%6.1fmm | IMU: %6.1f deg (%5.1f dps) | IR mm: [FL:%4.1f RL:%4.1f F:%4.1f FR:%4.1f RR:%4.1f] Center:%+4.2f Align:%+4.1fdeg\n",
                  battery.getVoltage(),
                  encoders.getLeftDistanceMM(),
                  encoders.getRightDistanceMM(),
                  imu.getHeadingDeg(),
                  imu.getYawRateDeg_S(),
                  ir.front_left_mm, ir.rear_left_mm, ir.front_mm,
                  ir.front_right_mm, ir.rear_right_mm,
                  ir.centering_error, ir.wall_alignment_error_deg);
}

void streamIRSensorDebug() {
    DistanceSensors ir = ir_sensors.getReadings();
    Serial.printf("[IR DEBUG] F=%s %.1fmm(raw=%u) | FL=%s %.1fmm(raw=%u) RL=%s %.1fmm(raw=%u) | FR=%s %.1fmm(raw=%u) RR=%s %.1fmm(raw=%u) | wall decisions F=%s L=%s R=%s | align=%+.1fdeg\n",
                  ir.front >= IR_WALL_DETECT_FRONT ? "BLOCKED" : "OPEN",
                  ir.front_mm, ir.front,
                  ir.front_left >= IR_WALL_DETECT_SIDE ? "BLOCKED" : "OPEN",
                  ir.front_left_mm, ir.front_left,
                  ir.rear_left >= IR_WALL_DETECT_SIDE ? "BLOCKED" : "OPEN",
                  ir.rear_left_mm, ir.rear_left,
                  ir.front_right >= IR_WALL_DETECT_SIDE ? "BLOCKED" : "OPEN",
                  ir.front_right_mm, ir.front_right,
                  ir.rear_right >= IR_WALL_DETECT_SIDE ? "BLOCKED" : "OPEN",
                  ir.rear_right_mm, ir.rear_right,
                  ir.wall_front ? "BLOCKED" : "OPEN",
                  ir.wall_left ? "BLOCKED" : "OPEN",
                  ir.wall_right ? "BLOCKED" : "OPEN",
                  ir.wall_alignment_error_deg);
}

void handleSerialCommands() {
    if (!Serial.available()) return;
    char cmd = Serial.read();

    switch (cmd) {
        case 'e':
        case 'E':
            Serial.println("[CMD] Starting Explore to Center...");
            navigator.exploreToCenter();
            break;

        case 'n':
        case 'N':
            Serial.println("[CMD] Taking one exploration step...");
            navigator.exploreOneCell();
            break;

        case 'r':
        case 'R':
            Serial.println("[CMD] Returning to Start...");
            navigator.exploreToStart();
            break;

        case 'f':
        case 'F':
            Serial.println("[CMD] Launching Speed Run...");
            navigator.runFastSpeed();
            break;

        case 'a':
        case 'A':
            Serial.println("[CMD] Starting Full Autonomous Sequence...");
            if (navigator.exploreToCenter()) {
                delay(1500);
                if (navigator.exploreToStart()) {
                    delay(2000);
                    navigator.runFastSpeed();
                }
            }
            break;

        case 'c':
        case 'C':
            Serial.println("[CMD] Running In-Cell Sensor Calibration...");
            ir_sensors.calibrateInCell(150);
            break;

        case 'd':
        case 'D':
            continuous_telemetry = !continuous_telemetry;
            Serial.printf("[CMD] Diagnostic Stream: %s\n", continuous_telemetry ? "ENABLED" : "DISABLED");
            break;

        case 'm':
        case 'M':
            navigator.getMaze().printAscii();
            break;

        case 't':
        case 'T':
            Serial.println("[CMD] Testing In-Place Turn (+90 deg)...");
            motion.turnInPlace(90.0f);
            break;

        case 'w':
        case 'W':
            Serial.println("[CMD] Testing Forward 1 Cell (180 mm)...");
            motion.moveForward(CELL_DIMENSION_MM);
            break;

        case '1':
            testSingleWheel(false);
            break;

        case '2':
            testSingleWheel(true);
            break;

        case 's':
        case 'S':
            Serial.println("[CMD] Emergency Stop!");
            motion.emergencyStop();
            break;

        case 'h':
        case 'H':
        case '?':
            printHelp();
            break;

        default:
            break;
    }
}

// Hand gesture mode selector using Front IR Sensor
void gestureModeSelector() {
    Serial.println("\n[READY] Hold a hand in front of the front sensor, then remove it to select a mode:");
    Serial.println("  Hold 0.6-1.8s: Explore to Center");
    Serial.println("  Hold 1.8-3.0s: Full Autonomous Run (Explore + Return + Speed Run)");
    Serial.println("  Hold >3.0s: In-Cell Calibration");
    Serial.println("  Or send [n] over Serial for one manual navigation step.");

    int hand_count = 0;
    uint32_t gesture_start = 0;
    uint32_t last_selector_debug_time = 0;

    while (hand_count == 0) {
        handleSerialCommands();

        ir_sensors.update();
        bool front_detected = ir_sensors.hasFrontWall();

        if (front_detected) {
            if (gesture_start == 0) {
                gesture_start = millis();
            } else {
                uint32_t duration = millis() - gesture_start;
                if (duration > 3000) {
                    digitalWrite(PIN_STATUS_LED, (millis() / 100) % 2);
                } else if (duration > 2000) {
                    digitalWrite(PIN_STATUS_LED, (millis() / 200) % 2);
                } else if (duration > 1000) {
                    digitalWrite(PIN_STATUS_LED, HIGH);
                }
            }
        } else {
            if (gesture_start > 0) {
                uint32_t total_held = millis() - gesture_start;
                gesture_start = 0;
                digitalWrite(PIN_STATUS_LED, LOW);

                if (total_held > 3000) {
                    hand_count = 3;
                } else if (total_held > 1800) {
                    hand_count = 2;
                } else if (total_held > 600) {
                    hand_count = 1;
                }
            }
        }

#ifdef DEBUG_IR_SENSOR_STREAM
        if (millis() - last_selector_debug_time >= 250) {
            last_selector_debug_time = millis();
            streamIRSensorDebug();
        }
#endif
        delay(10);
    }

    Serial.printf("[GESTURE] Selected Mode %d! Starting in 2 seconds...\n", hand_count);
    for (int i = 0; i < hand_count; ++i) {
        digitalWrite(PIN_STATUS_LED, HIGH); delay(200);
        digitalWrite(PIN_STATUS_LED, LOW);  delay(200);
    }
    delay(1000);

    if (hand_count == 1) {
        navigator.exploreToCenter();
    } else if (hand_count == 2) {
        if (navigator.exploreToCenter()) {
            delay(1500);
            if (navigator.exploreToStart()) {
                delay(2000);
                navigator.runFastSpeed();
            }
        }
    } else if (hand_count == 3) {
        ir_sensors.calibrateInCell(150);
    }
}

void runMotorStartupTest() {
    if (!motors.isConnected()) {
        Serial.println("[MOTOR TEST] Skipped: Motoron is not connected.");
        return;
    }

    constexpr int16_t test_speed = 400;
    constexpr uint32_t test_duration_ms = 3000;
    Serial.printf("[MOTOR TEST] Both wheels will start in 5 seconds at command %d for %lu ms.\n",
                  test_speed, static_cast<unsigned long>(test_duration_ms));
    Serial.println("[MOTOR TEST] Keep the robot securely lifted with wheels clear.");
    delay(5000);

    uint32_t test_start_time = millis();
    uint32_t last_status_time = test_start_time;
    while (millis() - test_start_time < test_duration_ms) {
        motors.setSpeeds(test_speed, test_speed);
        if (millis() - last_status_time >= 1000) {
            last_status_time = millis();
            Serial.printf("[MOTOR TEST] current-speed L=%d R=%d\n",
                          motors.getLeftCurrentSpeed(),
                          motors.getRightCurrentSpeed());
        }
        delay(100);
    }
    motors.stop(false);
    Serial.println("[MOTOR TEST] Complete; motors set to coast.");
}

void setup() {
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);

    Serial.begin(115200);
    delay(1500); // Allow USB Serial terminal to attach

#ifdef DEBUG_MANUAL_MAZE_BUILD_MODE
    Serial.println("\n========================================================");
    Serial.println("[MANUAL DEBUG MODE ACTIVE]");
    Serial.println("  - Position reset to (0,0)");
    Serial.println("  - Motors disabled");
    Serial.println("  - IR & IMU active");
    Serial.println("  - Scanning & printing ASCII map every 5 seconds");
    Serial.println("========================================================\n");

    ir_sensors.begin();
    imu.begin();
    navigator.reset(); // Starts at (0, 0) facing DIR_NORTH
    return;
#endif

#ifdef SENSOR_DISTANCE_DIAGNOSTIC_MODE
    Serial.println("[DIAG] Sensor distance diagnostic mode. Motors and navigation are disabled.");
    ir_sensors.begin();
    return;
#endif

    printBanner();

    // Initialize all motion and sensor hardware
    motion.begin();

#ifdef RUN_MOTOR_STARTUP_TEST
    runMotorStartupTest();
#endif

    // Initial self-test LED sequence
    for (int i = 0; i < 4; ++i) {
        digitalWrite(PIN_STATUS_LED, HIGH); delay(80);
        digitalWrite(PIN_STATUS_LED, LOW);  delay(80);
    }

    printHelp();

    // Check motor battery voltage at boot
    float vbat = battery.getVoltage();
    Serial.printf("[SETUP] Motor Battery Status: %4.2f V (%s)\n",
                  vbat, battery.isLow() ? "LOW WARNING!" : "HEALTHY");

    // Enter gesture mode selection or await serial command
    gestureModeSelector();
}

void loop() {
#ifdef DEBUG_MANUAL_MAZE_BUILD_MODE
    static uint32_t last_manual_step = 0;
    static uint32_t last_imu_update = 0;

    // Continuously update IMU integration at 100 Hz while the bot is physically rotated
    if (millis() - last_imu_update >= 10) {
        float dt = (millis() - last_imu_update) / 1000.0f;
        last_imu_update = millis();
        encoders.update(dt);
        imu.update(dt, encoders.getEncoderYawRateDeg_S(), encoders.getForwardSpeedMM_S());
    }

    // Trigger cell scan, wall mapping, and map rendering every 5 seconds
    if (millis() - last_manual_step >= 5000) {
        last_manual_step = millis();

        Serial.println("\n--------------------------------------------------------");
        Serial.println("[MANUAL DEBUG] 5-Second Interval - Scanning Cell & Mapping");
        Serial.println("--------------------------------------------------------");

        // Perform sensor wall scan and update maze structure using IMU orientation
        navigator.scanAndBuildCellManual();

        // Print visual 16x16 maze ASCII map
        navigator.getMaze().printAscii();

        Serial.println("[MANUAL DEBUG] Move bot manually to the next cell within 5 seconds...\n");
    }

    delay(5);
    return;
#endif

#ifdef SENSOR_DISTANCE_DIAGNOSTIC_MODE
    ir_sensors.update();
#else
    // Process incoming serial commands
    handleSerialCommands();

    // 20 Hz diagnostic telemetry stream if enabled
    if (continuous_telemetry && (millis() - last_telemetry_time >= 50)) {
        last_telemetry_time = millis();
        ir_sensors.update();
        encoders.update(0.05f);
        battery.update();
        streamDiagnostics();
    }
#endif

#if defined(DEBUG_IR_SENSOR_STREAM) || defined(SENSOR_DISTANCE_DIAGNOSTIC_MODE)
    static uint32_t last_ir_debug_time = 0;
    if (millis() - last_ir_debug_time >= 250) {
        last_ir_debug_time = millis();
        ir_sensors.update();
        streamIRSensorDebug();
    }
#endif

    delay(5);
}
