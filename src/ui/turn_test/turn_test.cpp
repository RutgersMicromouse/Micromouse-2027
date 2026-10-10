#include "robot.h"
#include "ui/turn_test/turn_test.h"

// ==============================================================================
// TURN TEST
// ==============================================================================

namespace TurnTest {

static const uint32_t TURN_TIMEOUT_MS = 4000;  // A turn that has not finished by then is given up on
static const uint32_t COAST_LOOK_MS   = 300;   // Look again this long after the stop, to catch coasting
static const uint32_t PAUSE_MS        = 400;   // Rest between turns

// One turn: sends it, waits for it to finish, and returns how far past the target it ended
// (+ = turned too far, - = not far enough), measured COAST_LOOK_MS after it stopped.
// Returns false if it was stopped or timed out.
// It also gives `turned_deg`, how far the robot actually rotated from where it started. That is
// the number to tune by when the end nudge is off: the test alternates left and right, so each
// turn starts from wherever the last one ended, and an error in one shows up with the opposite
// sign in the next. How far each turn rotated does not depend on where it started.
static bool oneTurn(bool left, bool fast, int number, float& over_deg, float& turned_deg) {
    const float start = g_imu.getHeadingDeg();
    // The target the controller itself will use: a turn on the spot starts from the nearest grid
    // heading (a multiple of 45 degrees) if the robot is within 18 degrees of one, otherwise from
    // wherever it is pointing (MotionController::executeCommand), plus or minus 90
    const float grid = roundf(start / 45.0f) * 45.0f;
    const float base = (fabsf(normalizeAngle180(start - grid)) <= 18.0f) ? grid : start;
    const float target = normalizeAngle180(base + (left ? 90.0f : -90.0f));
    const uint32_t bad_reads_before = g_imu.getBadReadCount();
    const uint32_t stops_before = stopCount();

    MotionCommand turn = {};
    turn.action = left ? ACTION_TURN_LEFT_90 : ACTION_TURN_RIGHT_90;
    turn.param_value = 90.0f;
    turn.max_speed_mm_s = fast ? SPEEDRUN_TURN_SPEED_DEG_S : SEARCH_TURN_SPEED_DEG_S;  // deg/s for turns
    turn.acceleration = fast ? SPEEDRUN_TURN_ACCEL_DEG_S2 : SEARCH_TURN_ACCEL_DEG_S2;
    xQueueSend(g_motion_cmd_queue, &turn, 0);

    const uint32_t started = millis();
    float peak_rate = 0.0f;
    vTaskDelay(pdMS_TO_TICKS(10)); // Let the motion task pick it up
    while (!g_motion_controller.isCommandFinished()) {
        if (stopCount() != stops_before) {
            Serial.println("[TURNTEST] Stopped.");
            return false;
        }
        if (millis() - started > TURN_TIMEOUT_MS) {
            Serial.printf("[TURNTEST] Turn %d did not finish within %lu ms: given up.\n", number, (unsigned long)TURN_TIMEOUT_MS);
            return false;
        }
        peak_rate = fmaxf(peak_rate, fabsf(g_imu.getGyroZ()));
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    const uint32_t took_ms = millis() - started;
    if (stopCount() != stops_before) { // (a STOP can end the turn before the loop above sees it)
        Serial.println("[TURNTEST] Stopped.");
        return false;
    }
    const float at_stop = g_imu.getHeadingDeg();
    vTaskDelay(pdMS_TO_TICKS(COAST_LOOK_MS));
    const float later = g_imu.getHeadingDeg();

    const float sign = left ? 1.0f : -1.0f; // So that + always means "turned too far"
    const float over_at_stop = sign * normalizeAngle180(at_stop - target);
    over_deg = sign * normalizeAngle180(later - target);
    turned_deg = sign * normalizeAngle180(later - start);

    // A turn that ends this far out is not a tuning result, it is a fault (on 2026-10-09 the IMU
    // came up reading the heading backwards and the robot span wildly through all ten turns).
    // Give up at once rather than send the next turn.
    if (fabsf(over_at_stop) > 30.0f || fabsf(over_deg) > 30.0f) {
        Serial.printf("[TURNTEST] Stopped: turn %d (%s) ended %.0f degrees from its target (heading %.1f, wanted %.1f). "
                      "That is a fault, not tuning: check the IMU (turn the robot left by hand, the heading must go UP).\n",
                      number, left ? "left" : "right", over_deg, later, target);
        requestStop();
        return false;
    }

    Serial.printf("[TURN] #%d %s | turned %.1f | from %.1f, target %.1f | stopped at %.1f (%+.1f) | %.1f s later %.1f (%+.1f) | peak %.0f deg/s | took %.2f s | IMU failed reads %lu\n",
                  number, left ? "LEFT " : "RIGHT", turned_deg, start, target, at_stop, over_at_stop, COAST_LOOK_MS / 1000.0f, later, over_deg,
                  peak_rate, took_ms / 1000.0f, (unsigned long)(g_imu.getBadReadCount() - bad_reads_before));
    return true;
}

// Mean, spread and worst of what one direction did
static void summarise(const char* name, const float* v, int n, char* out, size_t size) {
    if (n == 0) { snprintf(out, size, "%s none", name); return; }
    float sum = 0.0f, worst = 0.0f;
    for (int i = 0; i < n; ++i) { sum += v[i]; if (fabsf(v[i]) > fabsf(worst)) worst = v[i]; }
    const float mean = sum / n;
    float var = 0.0f;
    for (int i = 0; i < n; ++i) var += (v[i] - mean) * (v[i] - mean);
    snprintf(out, size, "%s mean %+.1f sd %.1f worst %+.1f (n=%d)", name, mean, sqrtf(var / n), worst, n);
}

void run(int pairs, bool fast) {
    pairs = constrain(pairs, 1, 10);
    float lefts[10], rights[10];
    int n_left = 0, n_right = 0;
    float turned_left = 0.0f, turned_right = 0.0f; // Sums of how far each turn actually rotated
    // The robot has usually just been put down by hand, pointing somewhere new. Without this the
    // controller still holds the heading it last aimed for and swings back to it before turning
    // (2026-10-10: carried a quarter turn after power-on, the first turn span the wrong way).
    prepareForNewRun();

    const uint32_t bad_reads_before = g_imu.getBadReadCount();
    const uint32_t started = millis();

    Serial.printf("[TURNTEST] Starting: %d left and %d right turns on the spot at %s speed (%.0f deg/s, %.0f deg/s2). + = turned too far, - = not far enough.\n",
                  pairs, pairs, fast ? "SPEED-RUN" : "search", fast ? SPEEDRUN_TURN_SPEED_DEG_S : SEARCH_TURN_SPEED_DEG_S,
                  fast ? SPEEDRUN_TURN_ACCEL_DEG_S2 : SEARCH_TURN_ACCEL_DEG_S2);
    for (int i = 0; i < pairs; ++i) {
        float over = 0.0f, turned = 0.0f;
        if (!oneTurn(true, fast, 2 * i + 1, over, turned)) break;
        lefts[n_left++] = over;
        turned_left += turned;
        vTaskDelay(pdMS_TO_TICKS(PAUSE_MS));
        if (!oneTurn(false, fast, 2 * i + 2, over, turned)) break;
        rights[n_right++] = over;
        turned_right += turned;
        vTaskDelay(pdMS_TO_TICKS(PAUSE_MS));
    }

    char left_text[80], right_text[80];
    summarise("LEFT", lefts, n_left, left_text, sizeof(left_text));
    summarise("RIGHT", rights, n_right, right_text, sizeof(right_text));
    RobotTelemetry telemetry = {};
    getTelemetry(telemetry);
    const MotionController& mc = g_motion_controller;
    Serial.printf("[TURNTEST] turned on average: LEFT %.1f, RIGHT %.1f degrees (90 is right; tune by this when t_push is 0)\n",
                  n_left ? turned_left / n_left : 0.0f, n_right ? turned_right / n_right : 0.0f);
    Serial.printf("[TURNTEST] done | %s | %s | %s | took %.0f s | supply %.1f V battery %.2f V | IMU failed reads %lu | "
                  "h_kp=%.4g h_ki=%.4g h_kd=%.4g turn_ff=%.4g t_push=%.4g t_damp=%.4g t_ka=%.4g imu_a=%.4g\n",
                  fast ? "FAST" : "search speed", left_text, right_text, (millis() - started) / 1000.0f, g_motors.getSupplyVolts(), telemetry.vbat_volts,
                  (unsigned long)(g_imu.getBadReadCount() - bad_reads_before),
                  mc.getTune(MotionController::TUNE_H_KP), mc.getTune(MotionController::TUNE_H_KI),
                  mc.getTune(MotionController::TUNE_H_KD), mc.getTune(MotionController::TUNE_TURN_FF),
                  mc.getTune(MotionController::TUNE_T_PUSH), mc.getTune(MotionController::TUNE_T_DAMP),
                  mc.getTune(MotionController::TUNE_T_KA),
                  mc.getTune(MotionController::TUNE_IMU_A));
}

} // namespace TurnTest
