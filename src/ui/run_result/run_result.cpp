#include "robot.h"
#include "ui/status_led/status_led.h"
#include "ui/actions/actions.h"
#include "ui/run_result/run_result.h"

// ==============================================================================
// RUN RESULT
// ==============================================================================

namespace RunResult {

static uint16_t s_count = 0; // Verdicts given since power-on

void report(bool success, const char* note) {
    RobotTelemetry telemetry = {};
    getTelemetry(telemetry);
    const RobotPose pose = g_navigator->getPose();
    const unsigned int n = ++s_count;

    // 1. The verdict, and the state of the robot when it was given
    Serial.printf("[RESULT] #%u %s | note: %s | %s | mode %s | tier %d | maze %dx%d | believes cell (%d, %d) | heading %.1f\n",
                  n, success ? "SUCCESS" : "FAIL", (note && note[0]) ? note : "-",
                  Actions::stateDescription(), Actions::modeName(Actions::getSelectedMode()),
                  (int)Actions::getSpeedTier(), (int)MAZE_ACTIVE_SIZE, (int)MAZE_ACTIVE_SIZE,
                  (int)pose.cell_x, (int)pose.cell_y, telemetry.imu.heading_deg);

    // 2. Health: the things that drift during a session
    Serial.printf("[RESULT] #%u health | motor supply %.1f V | battery %.2f V | IMU %s, %lu failed reads | late handovers %u | encoder faults %u | loop overruns %lu\n",
                  n, g_motors.getSupplyVolts(), telemetry.vbat_volts,
                  !g_imu.isHardwareConnected() ? "MISSING" : (g_imu.isUsingFallback() ? "FAULT" : "OK"),
                  (unsigned long)g_imu.getBadReadCount(), (unsigned int)g_motion_controller.getLateHandovers(),
                  (unsigned int)g_motion_controller.getEncoderFaults(), (unsigned long)telemetry.timing.loop_overruns);

    // 3. The speeds compiled into this firmware
    Serial.printf("[RESULT] #%u speeds | search %.0f known %.0f probe %.0f curve %.0f known-curve %.0f accel %.0f turn %.0f | speed run cruise %.0f curve %.0f curve-cap %.0f accel %.0f\n",
                  n, SEARCH_SPEED_DEFAULT_MM_S, SEARCH_KNOWN_SPEED_MM_S, SEARCH_PROBE_SPEED_MM_S,
                  SEARCH_CURVE_SPEED_MM_S, SEARCH_KNOWN_CURVE_SPEED_MM_S, SEARCH_ACCEL_DEFAULT_MM_S2, SEARCH_TURN_SPEED_DEG_S,
                  SPEEDRUN_CRUISE_SPEED_MM_S, SPEEDRUN_CURVE_SPEED_MM_S, SPEEDRUN_CURVE_MAX_MM_S, SPEEDRUN_ACCEL_MM_S2);

    // 4. The live tuning in force
    Serial.printf("[RESULT] #%u tune |", n);
    for (int i = 0; i < MotionController::TUNE_COUNT; ++i) {
        Serial.printf(" %s=%.6g", MotionController::tuneName(i), g_motion_controller.getTune(i));
    }
    Serial.println();
}

} // namespace RunResult
