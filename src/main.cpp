#include <Arduino.h>
#include <string.h>
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

// uncomment individual ones to activate
// #define DEBUG_MANUAL_MAZE_BUILD_MODE
// #define SENSOR_DISTANCE_DIAGNOSTIC_MODE
// #define DEBUG_IR_SENSOR_STREAM
// #define DEBUG_IMU_STREAM
// #define DEBUG_LEFT_WALL_FOLLOW
// #define DEBUG_ALTERNATING_TURNS

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
    Serial.printf("  [m] - Print %dx%d Maze ASCII Map\n", MAZE_WIDTH, MAZE_HEIGHT);
    Serial.println("  [t] - Test 90-deg in-place turn");
    Serial.println("  [3] - Test 360-deg in-place turn");
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

    const int16_t left_command = right ? 0 : 250;
    const int16_t right_command = right ? 250 : 0;
    Serial.printf("[MOTOR TEST] Testing %s wheel only for 1 second (L=%d, R=%d). Keep the robot lifted.\n",
                  right ? "right" : "left",
                  left_command,
                  right_command);
    motors.stop(false);
    delay(250);
    motors.stop(true);
    delay(100);
    encoders.reset();
    int32_t start_l = encoders.getLeftTicks();
    int32_t start_r = encoders.getRightTicks();

    motors.setSpeeds(left_command, right_command);
    delay(1000);
    const int16_t left_speed = motors.getLeftCurrentSpeed();
    const int16_t right_speed = motors.getRightCurrentSpeed();
    const uint16_t status = motors.getStatusFlags();
    motors.stop(false);
    delay(100);
    motors.stop(true);

    int32_t delta_l = encoders.getLeftTicks() - start_l;
    int32_t delta_r = encoders.getRightTicks() - start_r;

    Serial.printf("[MOTOR TEST] %s result: Motoron L=%d R=%d status=0x%04X | Encoders delta: L=%ld R=%ld | mm: L=%.1f R=%.1f\n",
                  right ? "Right" : "Left",
                  left_speed, right_speed, status,
                  (long)delta_l, (long)delta_r,
                  (float)delta_l * MM_PER_TICK, (float)delta_r * MM_PER_TICK);
    motors.stop(true);
    Serial.printf("[MOTOR TEST] %s wheel test complete; motors stopped.\n",
                  right ? "Right" : "Left");
}

void runMotorCalibration() {
    if (!motors.isConnected()) {
        Serial.println("[MOTOR CAL] Motor controller is not connected; speed balance test cancelled.");
        return;
    }

    Serial.println("\n[MOTOR CAL] Simultaneous wheel speed test starts in 3 seconds.");
    Serial.println("[MOTOR CAL] Secure the robot with both wheels lifted clear of the floor.");
    Serial.printf("[MOTOR CAL] Motor battery reading: %.2f V\n", battery.getVoltage());
    for (int i = 0; i < 3; ++i) {
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(500);
        digitalWrite(PIN_STATUS_LED, LOW);
        delay(500);
    }

    encoders.reset();
    const int32_t start_left_ticks = encoders.getLeftTicks();
    const int32_t start_right_ticks = encoders.getRightTicks();
    Serial.printf("[MOTOR CAL] Spinning both wheels at command %d for %lu ms.\n",
                  MOTOR_BALANCE_TEST_COMMAND,
                  (unsigned long)MOTOR_BALANCE_TEST_DURATION_MS);
    motors.setSpeeds(MOTOR_BALANCE_TEST_COMMAND, MOTOR_BALANCE_TEST_COMMAND);
    const uint32_t start_time = millis();
    while (millis() - start_time < MOTOR_BALANCE_TEST_DURATION_MS) {
        delay(5);
    }

    const uint32_t elapsed_ms = millis() - start_time;
    const int32_t left_tick_delta = encoders.getLeftTicks() - start_left_ticks;
    const int32_t right_tick_delta = encoders.getRightTicks() - start_right_ticks;
    motors.stop(false);
    delay(100);
    motors.stop(true);
    encoders.reset();

    const float elapsed_seconds = elapsed_ms * 0.001f;
    const float left_ticks_per_second = fabsf(left_tick_delta) / elapsed_seconds;
    const float right_ticks_per_second = fabsf(right_tick_delta) / elapsed_seconds;
    Serial.printf("[MOTOR CAL] Encoder rates: left=%ld ticks/s right=%ld ticks/s (ticks L=%ld R=%ld).\n",
                  (long)left_ticks_per_second,
                  (long)right_ticks_per_second,
                  (long)left_tick_delta,
                  (long)right_tick_delta);

    if (left_tick_delta <= 0 || right_tick_delta <= 0) {
        Serial.printf("[MOTOR CAL] Invalid forward encoder movement (L=%ld, R=%ld); keeping configured motor balance.\n",
                      (long)left_tick_delta,
                      (long)right_tick_delta);
    } else if (left_tick_delta < 20 || right_tick_delta < 20) {
        Serial.println("[MOTOR CAL] Too few encoder ticks to calibrate reliably; keeping configured motor balance.");
    } else if (!motors.calibrateWheelSpeedBalance(left_ticks_per_second,
                                                  right_ticks_per_second)) {
        Serial.println("[MOTOR CAL] Invalid encoder rates; keeping configured motor balance.");
    }

    Serial.println("[MOTOR CAL] Startup wheel test complete.");
}

void runLeftWallFollowDebugStep() {
    ir_sensors.update();
    const bool left_blocked = ir_sensors.hasLeftWall();
    const bool front_blocked = ir_sensors.hasFrontWall();
    const bool right_blocked = ir_sensors.hasRightWall();
    const DistanceSensors readings = ir_sensors.getReadings();

    Serial.printf("[LEFT WALL DEBUG] Walls L=%s F=%s R=%s | raw FL=%u RL=%u F=%u FR=%u RR=%u\n",
                  left_blocked ? "blocked" : "open",
                  front_blocked ? "blocked" : "open",
                  right_blocked ? "blocked" : "open",
                  readings.front_left,
                  readings.rear_left,
                  readings.front,
                  readings.front_right,
                  readings.rear_right);

    if (!left_blocked && !right_blocked) {
        Serial.println("[LEFT WALL DEBUG] Both side openings are clear; turn right first.");
        if (!motion.turnInPlace(-90.0f)) return;
    } else if (!left_blocked) {
        Serial.println("[LEFT WALL DEBUG] Turn left.");
        if (!motion.turnInPlace(90.0f)) return;
    } else if (front_blocked && !right_blocked) {
        Serial.println("[LEFT WALL DEBUG] Front blocked, right open; turn right.");
        if (!motion.turnInPlace(-90.0f)) return;
    } else if (front_blocked) {
        Serial.println("[LEFT WALL DEBUG] Dead end; turn around.");
        if (!motion.turnInPlace(-90.0f)) return;
        delay(50);
        if (!motion.turnInPlace(-90.0f)) return;
    }

    if (!motion.moveForward(CELL_DIMENSION_MM, SEARCH_SPEED_MM_S, 0.0f, true, 0.7f)) {
        Serial.println("[LEFT WALL DEBUG] Forward move failed; waiting for the next stage-1 selection.");
    } else {
        Serial.println("[LEFT WALL DEBUG] One cell complete; waiting for the next stage-1 selection.");
    }
}

void runAlternatingTurnDebug() {
    Serial.println("[TURN DEBUG] Alternating 90-degree turns; first turn starts in 5 seconds. Keep the robot clear.");
    if (!motors.isConnected()) {
        Serial.println("[TURN DEBUG] Motor controller is not connected; no turn commands can run.");
        while (true) {
            digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
            delay(200);
        }
    }

    bool turn_right = true;
    delay(5000);

    while (true) {
        const float angle_deg = turn_right ? -90.0f : 90.0f;
        Serial.printf("[TURN DEBUG] Closed-loop turning %s 90 degrees (Target: %+.1f deg).\n",
                      turn_right ? "right" : "left",
                      angle_deg);

        if (!motion.turnInPlace(angle_deg)) {
            motion.emergencyStop();
            Serial.println("[TURN DEBUG] Turn failed; test halted.");
            while (true) {
                digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
                delay(200);
            }
        }

        digitalWrite(PIN_STATUS_LED, !digitalRead(PIN_STATUS_LED));
        turn_right = !turn_right;
        delay(5000);
    }
}

void updateImuDebugStream() {
    static uint32_t last_update_micros = 0;
    static uint32_t last_print_ms = 0;
    static float previous_heading_deg = 0.0f;
    static float accumulated_heading_deg = 0.0f;

    const uint32_t now_micros = micros();
    if (last_update_micros == 0) {
        last_update_micros = now_micros;
        previous_heading_deg = imu.getHeadingDeg();
        return;
    }

    const float dt_seconds = (now_micros - last_update_micros) * 1e-6f;
    last_update_micros = now_micros;
    encoders.update(dt_seconds);
    imu.update(dt_seconds,
               encoders.getEncoderYawRateDeg_S(),
               encoders.getForwardSpeedMM_S());

    if (millis() - last_print_ms >= 100) {
        last_print_ms = millis();
        const float heading_deg = imu.getHeadingDeg();
        float heading_delta_deg = heading_deg - previous_heading_deg;
        if (heading_delta_deg > 180.0f) heading_delta_deg -= 360.0f;
        if (heading_delta_deg <= -180.0f) heading_delta_deg += 360.0f;
        accumulated_heading_deg += heading_delta_deg;
        previous_heading_deg = heading_deg;

        Serial.printf("[IMU DEBUG] connected=%s heading=%+.1fdeg total=%+.1fdeg yaw=%+.1fdps raw=%+.1fdps bias=%+.1fdps\n",
                      imu.isConnected() ? "YES" : "NO",
                      heading_deg,
                      accumulated_heading_deg,
                      imu.getYawRateDeg_S(),
                      imu.getRawYawRateDeg_S(),
                      imu.getGyroBiasDeg_S());
    }
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

void    streamIRSensorDebug() {
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

        case '3':
            Serial.println("[CMD] Testing In-Place Turn (360 deg)...");
            motion.turnInPlace(-360.0f);
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
    while (true) {
        Serial.println("\n[READY] Hold a hand in front of the front sensor, then remove it to select a mode:");
        Serial.println("  Hold 0.6-1.8s: Explore to Center");
        Serial.println("  Hold 1.8-3.0s: Full Autonomous Run (Explore + Return + Speed Run)");
        Serial.println("  Hold >3.0s: In-Cell Sensor Calibration");

        int hand_count = 0;
        uint32_t gesture_start = 0;
#ifdef DEBUG_IR_SENSOR_STREAM
        uint32_t last_selector_debug_time = 0;
#endif

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
        delay(600);

        if (hand_count == 1 || hand_count == 2) {
            // Re-zero gyro bias, heading, and navigation pose while resting stationary in start cell
            imu.calibrateStaticBias(150);
            imu.resetHeading(0.0f);
            motion.setTargetHeading(0.0f);
            encoders.reset();
            navigator.reset();
        }

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
            return;
        } else if (hand_count == 3) {
            ir_sensors.calibrateInCell(150);
            Serial.println("[GESTURE] Sensor calibration done. Hold a hand in front of the sensor again to choose a run mode.");
        }
    }
}

#ifdef DEBUG_MANUAL_MAZE_BUILD_MODE
const char* manualHeadingName(Direction heading) {
    switch (heading) {
        case DIR_NORTH: return "NORTH";
        case DIR_EAST:  return "EAST";
        case DIR_SOUTH: return "SOUTH";
        case DIR_WEST:  return "WEST";
        default:        return "UNKNOWN";
    }
}

bool readManualMazeCommand(bool& advance, int8_t& quarter_turns) {
    static char command[3];
    static uint8_t command_length = 0;
    static bool command_overflow = false;

    while (Serial.available() > 0) {
        char input = static_cast<char>(Serial.read());
        if (input == '\r' || input == '\n') {
            if (command_length == 0 && !command_overflow) {
                continue;
            }

            command[command_length] = '\0';
            bool valid_command = false;
            if (!command_overflow) {
                for (uint8_t i = 0; i < command_length; ++i) {
                    if (command[i] >= 'A' && command[i] <= 'Z') {
                        command[i] = static_cast<char>(command[i] - 'A' + 'a');
                    }
                }

                if (strcmp(command, "s") == 0) {
                    advance = true;
                    quarter_turns = 0;
                    valid_command = true;
                    Serial.println("[MANUAL INPUT] Advance one cell straight.");
                } else if (strcmp(command, "r") == 0) {
                    advance = false;
                    quarter_turns = 1;
                    valid_command = true;
                    Serial.printf("[MANUAL INPUT] 90 clockwise. Face %s; physically turn before entering this command.\n",
                                  manualHeadingName(turnRight(navigator.getCurrentHeading())));
                } else if (strcmp(command, "l") == 0) {
                    advance = false;
                    quarter_turns = -1;
                    valid_command = true;
                    Serial.printf("[MANUAL INPUT] 90 counterclockwise. Face %s; physically turn before entering this command.\n",
                                  manualHeadingName(turnLeft(navigator.getCurrentHeading())));
                } else if (strcmp(command, "ll") == 0) {
                    advance = false;
                    quarter_turns = 2;
                    valid_command = true;
                    Serial.printf("[MANUAL INPUT] 180 degrees. Face %s; physically turn before entering this command.\n",
                                  manualHeadingName(oppositeDirection(navigator.getCurrentHeading())));
                } else {
                    Serial.println("[MANUAL INPUT] Unknown command. Enter s, r, l, or ll, then press Enter.");
                }
            } else {
                Serial.println("[MANUAL INPUT] Input too long. Enter s, r, l, or ll, then press Enter.");
            }
            command_length = 0;
            command_overflow = false;
            if (valid_command) {
                return true;
            }
        } else if (!command_overflow) {
            if (command_length < sizeof(command) - 1) {
                command[command_length++] = input;
            } else {
                command_overflow = true;
            }
        }
    }
    return false;
}

void printManualMazeScan() {
    static uint16_t manual_scan_number = 0;
    ++manual_scan_number;

    if (manual_scan_number % 2 == 1) {
        Serial.printf("\n/\\/\\/\\/\\/\\/\\/\\/\\/\\ SCAN %u START /\\/\\/\\/\\/\\/\\/\\/\\/\\\n",
                      static_cast<unsigned int>(manual_scan_number));
    } else {
        Serial.printf("\n==================== SCAN %u START ====================\n",
                      static_cast<unsigned int>(manual_scan_number));
    }

    navigator.scanAndBuildCellManual();
    navigator.getMaze().printAscii();

    if (manual_scan_number % 2 == 1) {
        Serial.printf("/\\/\\/\\/\\/\\/\\/\\/\\/\\ SCAN %u COMPLETE /\\/\\/\\/\\/\\/\\/\\/\\/\\\n",
                      static_cast<unsigned int>(manual_scan_number));
    } else {
        Serial.printf("==================== SCAN %u COMPLETE ====================\n",
                      static_cast<unsigned int>(manual_scan_number));
    }
    Serial.println("[MANUAL DEBUG] Move/turn the bot first, then enter s (straight), r (90 CW), l (90 CCW), or ll (180) and press Enter.");
}

void handleManualMazeInput() {
    bool advance = false;
    int8_t quarter_turns = 0;
    if (!readManualMazeCommand(advance, quarter_turns)) {
        return;
    }

    if (advance) {
        if (!navigator.advanceManualCell()) {
            Serial.printf("[MANUAL DEBUG] Cannot advance beyond maze boundary from (%d, %d); position unchanged.\n",
                          navigator.getCurrentPosition().x,
                          navigator.getCurrentPosition().y);
        }
    } else {
        navigator.applyManualTurn(quarter_turns);
    }
    printManualMazeScan();
}
#endif

void setup() {
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);

    Serial.begin(115200);
    delay(1500); // Allow USB Serial terminal to attach

#ifdef DEBUG_MANUAL_MAZE_BUILD_MODE
    Serial.println("\n========================================================");
    Serial.printf("[MANUAL MAZE DEBUG MODE ACTIVE] Build: %dx%d\n", MAZE_WIDTH, MAZE_HEIGHT);
    Serial.println("  - Position reset to (0,0)");
    Serial.println("  - Motors are not initialized; motor battery can remain off");
    Serial.println("  - IMU is not used; enter manual turn commands in Serial Monitor");
    Serial.println("  - Enter s = straight one cell, r = 90 clockwise, l = 90 counterclockwise, ll = 180");
    Serial.println("  - IR sensors active");
    Serial.println("  - Scans only after a valid serial command");
    Serial.println("========================================================\n");

    ir_sensors.begin();
    navigator.reset(); // Starts at (0, 0) facing DIR_NORTH
    Serial.println("[MANUAL DEBUG] First scan starts in 10 seconds.");
    delay(10000);
    printManualMazeScan();
    return;
#endif

#ifdef SENSOR_DISTANCE_DIAGNOSTIC_MODE
    Serial.println("[DIAG] Sensor distance diagnostic mode. Motors and navigation are disabled.");
    ir_sensors.begin();
    return;
#endif

    printBanner();

#ifdef DEBUG_IMU_STREAM
    encoders.begin();
    const bool imu_ready = imu.begin();
    Serial.printf("[IMU DEBUG] Stream active; IMU %s. Hold still until bias calibration completes, then rotate 90 degrees clockwise and counterclockwise.\n",
                  imu_ready ? "ready" : "NOT DETECTED");
    Serial.println("[IMU DEBUG] Compare the total heading change for each rotation; motors and calibration test are disabled.");
    return;
#endif

    // Initialize all motion and sensor hardware
    motion.begin();

#ifdef CALIBRATE_MOTORS
    runMotorCalibration();
#endif

#ifdef DEBUG_ALTERNATING_TURNS
    runAlternatingTurnDebug();
    return;
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
#ifdef DEBUG_IMU_STREAM
    updateImuDebugStream();
    delay(2);
    return;
#endif

#ifdef DEBUG_ALTERNATING_TURNS
    delay(5);
    return;
#endif

#ifdef DEBUG_LEFT_WALL_FOLLOW
    delay(5);
    return;
#endif

#ifdef DEBUG_MANUAL_MAZE_BUILD_MODE
    handleManualMazeInput();
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
