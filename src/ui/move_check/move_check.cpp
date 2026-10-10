#include "robot.h"
#include "ui/move_check/move_check.h"

// ==============================================================================
// MOVE CHECK
// ==============================================================================

namespace MoveCheck {

static const float    PUSH_EFFORT     = 0.22f;  // Fixed effort on the one pushed wheel
static const uint32_t PUSH_MS         = 220;    // Short: on the floor the robot swings round its other wheel
static const uint32_t REST_MS         = 500;    // Let it come to rest before measuring
static const float    SWUNG_DEG       = 3.0f;   // A swing smaller than this tells nothing
static const float    COUNTED_MM      = 4.0f;   // Nor does an encoder change smaller than this
static const uint32_t MOVE_TIMEOUT_MS = 5000;
static const float    CELL_MM         = 180.0f;

static uint32_t stops_before_ = 0;

// Waits, giving up early on a STOP. Returns false if stopped.
static bool wait(uint32_t ms) {
    const uint32_t started = millis();
    while (millis() - started < ms) {
        if (stopCount() != stops_before_) return false;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return true;
}

// What a forward push on one wheel showed: +1 = the right way, -1 = reversed, 0 = could not tell
struct WheelFinding {
    int motor;
    int encoder;
    float swing_deg;
    float counted_mm;
};

// Pushes one wheel "forward" and works out, from which way the robot swung, which way the motor
// really ran and whether its encoder counted that way. Returns false if stopped.
static bool pushWheel(bool left_wheel, WheelFinding& out) {
    const EncoderState before = g_encoders.getState();
    const float heading_before = g_imu.getHeadingDeg();

    g_motors.setEffort(left_wheel ? PUSH_EFFORT : 0.0f, left_wheel ? 0.0f : PUSH_EFFORT);
    const bool finished = wait(PUSH_MS);
    g_motors.brake();
    if (!finished || !wait(REST_MS)) return false;

    const EncoderState after = g_encoders.getState();
    out.swing_deg = normalizeAngle180(g_imu.getHeadingDeg() - heading_before);
    out.counted_mm = left_wheel ? after.left_dist_mm - before.left_dist_mm
                                : after.right_dist_mm - before.right_dist_mm;

    // The left wheel rolling forward swings the robot right (heading down); the right wheel, left
    const float forward_swing = left_wheel ? -1.0f : 1.0f;
    out.motor = (fabsf(out.swing_deg) < SWUNG_DEG) ? 0 : ((out.swing_deg * forward_swing > 0.0f) ? 1 : -1);
    // The wheel really rolled forward if the motor is the right way round, backward if not; the
    // encoder is right if its count has that sign
    out.encoder = (out.motor == 0 || fabsf(out.counted_mm) < COUNTED_MM) ? 0
                : ((out.counted_mm * (float)out.motor > 0.0f) ? 1 : -1);
    return true;
}

static const char* word(int finding) {
    return finding > 0 ? "right way" : (finding < 0 ? "REVERSED" : "could not tell");
}

// Step 1. Returns false if stopped or if the directions could not be put right.
static bool checkDirections() {
    for (int pass = 1; pass <= 2; ++pass) {
        WheelFinding left = {}, right = {};
        if (!pushWheel(true, left) || !pushWheel(false, right)) return false;

        bool inv_motor_l = false, inv_motor_r = false, inv_enc_l = false, inv_enc_r = false;
        g_motors.getInverted(inv_motor_l, inv_motor_r);
        g_encoders.getInverted(inv_enc_l, inv_enc_r);
        Serial.printf("[MOVECHECK] 1 directions, pass %d | LEFT push: swung %+.1f deg, counted %+.0f mm -> motor %s, encoder %s | "
                      "RIGHT push: swung %+.1f deg, counted %+.0f mm -> motor %s, encoder %s | settings were motors L=%d R=%d, encoders L=%d R=%d\n",
                      pass, left.swing_deg, left.counted_mm, word(left.motor), word(left.encoder),
                      right.swing_deg, right.counted_mm, word(right.motor), word(right.encoder),
                      (int)inv_motor_l, (int)inv_motor_r, (int)inv_enc_l, (int)inv_enc_r);

        if (left.motor == 0 || right.motor == 0 || left.encoder == 0 || right.encoder == 0) {
            Serial.println("[MOVECHECK] FAIL: a wheel did not move the robot or its encoder did not count. "
                           "Is it on the floor with room to swing, motor power on?");
            return false;
        }
        if (left.motor > 0 && right.motor > 0 && left.encoder > 0 && right.encoder > 0) {
            Serial.printf("[MOVECHECK] 1 directions PASS | config.h must have INVERT_LEFT_MOTOR %s, INVERT_RIGHT_MOTOR %s, "
                          "INVERT_LEFT_ENCODER %s, INVERT_RIGHT_ENCODER %s\n",
                          inv_motor_l ? "true" : "false", inv_motor_r ? "true" : "false",
                          inv_enc_l ? "true" : "false", inv_enc_r ? "true" : "false");
            return true;
        }
        if (pass == 2) break;

        // Switch over whatever ran the wrong way, then push again to make sure
        if (left.motor < 0)    inv_motor_l = !inv_motor_l;
        if (right.motor < 0)   inv_motor_r = !inv_motor_r;
        if (left.encoder < 0)  inv_enc_l = !inv_enc_l;
        if (right.encoder < 0) inv_enc_r = !inv_enc_r;
        g_motors.setInverted(inv_motor_l, inv_motor_r);
        g_encoders.setInverted(inv_enc_l, inv_enc_r);
        Serial.printf("[MOVECHECK] Switched over until the next restart: motors L=%d R=%d, encoders L=%d R=%d. Checking again.\n",
                      (int)inv_motor_l, (int)inv_motor_r, (int)inv_enc_l, (int)inv_enc_r);
    }
    Serial.println("[MOVECHECK] FAIL: still the wrong way round after switching over.");
    return false;
}

// Sends one move and waits for it. Returns false if stopped, timed out or aborted by a safety stop.
static bool doMove(const MotionCommand& move) {
    xQueueSend(g_motion_cmd_queue, &move, 0);
    const uint32_t started = millis();
    vTaskDelay(pdMS_TO_TICKS(10)); // Let the motion task pick it up
    while (!g_motion_controller.isCommandFinished()) {
        if (stopCount() != stops_before_) return false;
        if (millis() - started > MOVE_TIMEOUT_MS) {
            Serial.println("[MOVECHECK] FAIL: the move did not finish in time.");
            requestStop();
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return wait(REST_MS);
}

// Step 2: one cell forward
static bool checkForward() {
    const EncoderState before = g_encoders.getState();
    const float heading_before = g_imu.getHeadingDeg();
    MotionCommand move = {};
    move.action = ACTION_MOVE_DISTANCE;
    move.param_value = CELL_MM;
    move.max_speed_mm_s = SEARCH_SPEED_DEFAULT_MM_S;
    move.acceleration = SEARCH_ACCEL_DEFAULT_MM_S2;
    move.enable_wall_centering = false;
    if (!doMove(move)) return false;

    const EncoderState after = g_encoders.getState();
    const float left = after.left_dist_mm - before.left_dist_mm;
    const float right = after.right_dist_mm - before.right_dist_mm;
    const float drift = normalizeAngle180(g_imu.getHeadingDeg() - heading_before);
    const bool pass = fabsf(left - CELL_MM) < 20.0f && fabsf(right - CELL_MM) < 20.0f && fabsf(drift) < 8.0f;
    Serial.printf("[MOVECHECK] 2 forward %.0f mm %s | wheels counted left %+.0f, right %+.0f mm | heading drifted %+.1f deg "
                  "(measure on the floor how far it really went)\n", CELL_MM, pass ? "PASS" : "FAIL", left, right, drift);
    return pass;
}

// Step 3: a 90 degree turn on the spot
static bool checkTurn(bool left) {
    const float heading_before = g_imu.getHeadingDeg();
    MotionCommand move = {};
    move.action = left ? ACTION_TURN_LEFT_90 : ACTION_TURN_RIGHT_90;
    move.param_value = 90.0f;
    move.max_speed_mm_s = SEARCH_TURN_SPEED_DEG_S;  // deg/s for turns
    move.acceleration = SEARCH_TURN_ACCEL_DEG_S2;
    if (!doMove(move)) return false;

    const float turned = normalizeAngle180(g_imu.getHeadingDeg() - heading_before) * (left ? 1.0f : -1.0f);
    const bool pass = fabsf(turned - 90.0f) < 15.0f;
    Serial.printf("[MOVECHECK] 3 turn %s on the spot %s | turned %.1f deg its own way (90 wanted; negative = the wrong way)\n",
                  left ? "LEFT" : "RIGHT", pass ? "PASS" : "FAIL", turned);
    return pass;
}

void run() {
    stops_before_ = stopCount();
    Serial.printf("[MOVECHECK] Starting. Robot on the floor with clear space all round. Supply %.1f V, dist_k %.3g.\n",
                  g_motors.getSupplyVolts(), g_motion_controller.getTune(MotionController::TUNE_DIST_K));

    // Step 1 drives the motors itself, so the motion controller is paused for it
    g_motion_controller.setCalibrating(true);
    const bool directions_ok = checkDirections();
    g_motors.brake();
    g_motion_controller.setCalibrating(false);

    bool all_ok = directions_ok;
    if (directions_ok) {
        requestHeadingReset();
        requestEncoderReset();
        vTaskDelay(pdMS_TO_TICKS(50));
        all_ok = checkForward() && checkTurn(true) && checkTurn(false);
    }
    if (stopCount() != stops_before_) {
        Serial.println("[MOVECHECK] Stopped.");
        return;
    }
    Serial.printf("[MOVECHECK] done | %s\n", all_ok ? "ALL PASS" : "FAILED (see the lines above)");
}

} // namespace MoveCheck
