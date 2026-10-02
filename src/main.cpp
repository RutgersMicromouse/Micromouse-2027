#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "control/motion_controller.h"
#include "navigation/navigator.h"

// =============================================================================
// Ratatouieee Micromouse Firmware - Main Entry Point
// Rutgers Micromouse 2026-2027
// Target: Teensy 4.0 (NXP i.MX RT1062, ARM Cortex-M7 @ 600MHz)
// =============================================================================

RobotState current_robot_state = STATE_IDLE;
bool continuous_telemetry = false;
uint32_t last_telemetry_time = 0;

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
    Serial.println("  [r] - Return to Start (0, 0)");
    Serial.println("  [f] - Execute High-Speed Speed Run");
    Serial.println("  [a] - Full Autonomy (Explore -> Return -> Speed Run)");
    Serial.println("  [c] - In-Cell IR Auto-Calibration");
    Serial.println("  [d] - Toggle Real-Time Diagnostic Stream");
    Serial.println("  [m] - Print 16x16 Maze ASCII Map");
    Serial.println("  [t] - Test 90-deg in-place turn");
    Serial.println("  [w] - Test 1-cell forward move");
    Serial.println("  [s] - Emergency Stop\n");
}

void streamDiagnostics() {
    DistanceSensors ir = ir_sensors.getReadings();
    Serial.printf("[DIAG] BAT: %4.2fV | ENC: L=%6.1fmm R=%6.1fmm | IMU: %6.1f deg (%5.1f dps) | IR: [L90:%4d L45:%4d F:%4d R45:%4d R90:%4d] Centering:%+4.2f\n",
                  battery.getVoltage(),
                  encoders.getLeftDistanceMM(),
                  encoders.getRightDistanceMM(),
                  imu.getHeadingDeg(),
                  imu.getYawRateDeg_S(),
                  ir.left_90, ir.left_45, ir.front, ir.right_45, ir.right_90,
                  ir.centering_error);
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
    Serial.println("\n[READY] Wave hand in front of front sensor to select mode:");
    Serial.println("  1 blink / hold 1s: Explore to Center");
    Serial.println("  2 blinks / hold 2s: Full Autonomous Run (Explore + Return + Speed Run)");
    Serial.println("  3 blinks / hold 3s: In-Cell Calibration");

    int hand_count = 0;
    uint32_t gesture_start = 0;

    while (hand_count == 0) {
        handleSerialCommands();

        ir_sensors.update();
        uint16_t front_val = ir_sensors.getFront();

        // Front sensor covered by hand (> 350)
        if (front_val > 350) {
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

void setup() {
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);

    Serial.begin(115200);
    delay(1500); // Allow USB Serial terminal to attach

    printBanner();

    // Initialize all motion and sensor hardware
    motion.begin();

    // Initial self-test LED sequence
    for (int i = 0; i < 4; ++i) {
        digitalWrite(PIN_STATUS_LED, HIGH); delay(80);
        digitalWrite(PIN_STATUS_LED, LOW);  delay(80);
    }

    printHelp();

    // Check battery voltage at boot
    float vbat = battery.getVoltage();
    Serial.printf("[SETUP] Battery Status: %4.2f V (%s)\n",
                  vbat, battery.isLow() ? "LOW WARNING!" : "HEALTHY");

    // Enter gesture mode selection or await serial command
    gestureModeSelector();
}

void loop() {
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

    delay(5);
}
