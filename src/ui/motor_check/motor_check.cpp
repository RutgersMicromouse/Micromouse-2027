#include "robot.h"
#include "ui/motor_check/motor_check.h"

// ==============================================================================
// MOTOR CHECK
// ==============================================================================

namespace MotorCheck {

static const float    EFFORT        = 0.25f;  // Fixed effort on the one driven wheel
static const uint32_t ANNOUNCE_MS   = 1500;   // Time to read what is about to happen
static const uint32_t DRIVE_MS      = 600;    // How long the wheel is driven
static const uint32_t REST_MS       = 400;    // Let it come to rest before measuring
static const float    MOVED_MM      = 15.0f;  // An encoder that changed less than this did not move
static const float    TURNED_DEG    = 5.0f;   // A heading that changed less than this did not turn

// What one step found
struct Step {
    float left_mm;      // What the left encoder says its wheel went (+ = forward)
    float right_mm;
    float heading_deg;  // + = the robot swung left
};

// Waits, giving up early on a STOP. Returns false if stopped.
static bool wait(uint32_t ms, uint32_t stops_before) {
    const uint32_t started = millis();
    while (millis() - started < ms) {
        if (stopCount() != stops_before) return false;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return true;
}

// Drives one wheel one way and measures. Returns false if stopped.
static bool oneStep(bool left_wheel, bool forward, int number, Step& out, uint32_t stops_before) {
    const char* side = left_wheel ? "LEFT" : "RIGHT";
    const char* way  = forward ? "FORWARD" : "BACKWARD";
    Serial.printf("[MOTOR] Step %d of 4: the %s wheel should now roll %s (only that wheel).\n", number, side, way);
    if (!wait(ANNOUNCE_MS, stops_before)) return false;

    const EncoderState before = g_encoders.getState();
    const float heading_before = g_imu.getHeadingDeg();

    const float effort = forward ? EFFORT : -EFFORT;
    g_motors.setEffort(left_wheel ? effort : 0.0f, left_wheel ? 0.0f : effort);
    const bool finished = wait(DRIVE_MS, stops_before);
    g_motors.brake();
    if (!finished) return false;
    if (!wait(REST_MS, stops_before)) return false;

    const EncoderState after = g_encoders.getState();
    out.left_mm = after.left_dist_mm - before.left_dist_mm;
    out.right_mm = after.right_dist_mm - before.right_dist_mm;
    out.heading_deg = normalizeAngle180(g_imu.getHeadingDeg() - heading_before);

    // The encoders: did the driven wheel's own encoder move, and the right way?
    const float own = left_wheel ? out.left_mm : out.right_mm;
    const float other = left_wheel ? out.right_mm : out.left_mm;
    const float wanted = forward ? 1.0f : -1.0f;
    const char* encoder_says;
    if (fabsf(own) < MOVED_MM && fabsf(other) >= MOVED_MM) encoder_says = "the OTHER side's encoder moved: the two sides are swapped";
    else if (fabsf(own) < MOVED_MM)                         encoder_says = "no encoder moved: motor not turning, or encoder dead";
    else if (own * wanted > 0.0f)                           encoder_says = "its encoder agrees";
    else                                                    encoder_says = "its encoder counted the OPPOSITE way";

    // The heading (only means something on the floor): the left wheel going forward swings the
    // robot right (heading down); the right wheel going forward swings it left (heading up)
    const float wanted_swing = (left_wheel ? -1.0f : 1.0f) * wanted;
    const char* heading_says;
    if (fabsf(out.heading_deg) < TURNED_DEG)          heading_says = "heading did not move (wheels off the floor?)";
    else if (out.heading_deg * wanted_swing > 0.0f)   heading_says = "heading agrees";
    else                                              heading_says = "heading went the OPPOSITE way";

    Serial.printf("[MOTOR] Step %d %s wheel %s | encoders: left %+.0f mm, right %+.0f mm | heading %+.1f deg | %s; %s\n",
                  number, side, way, out.left_mm, out.right_mm, out.heading_deg, encoder_says, heading_says);
    return true;
}

void run() {
    const uint32_t stops_before = stopCount();
    bool inv_motor_l = false, inv_motor_r = false, inv_enc_l = false, inv_enc_r = false;
    g_motors.getInverted(inv_motor_l, inv_motor_r);
    g_encoders.getInverted(inv_enc_l, inv_enc_r);
    Serial.printf("[MOTORCHECK] Starting: each wheel alone, forward then backward, effort %.2f for %.1f s. Watch which wheel rolls, and which way. "
                  "Settings now: motors reversed L=%d R=%d, encoders reversed L=%d R=%d, supply %.1f V.\n",
                  EFFORT, DRIVE_MS / 1000.0f, (int)inv_motor_l, (int)inv_motor_r, (int)inv_enc_l, (int)inv_enc_r, g_motors.getSupplyVolts());

    // Take the motors away from the motion controller for the length of the test
    g_motion_controller.setCalibrating(true);
    Step steps[4] = {};
    int done = 0;
    for (; done < 4; ++done) {
        if (!oneStep(done < 2, (done % 2) == 0, done + 1, steps[done], stops_before)) break;
    }
    g_motors.brake();
    g_motion_controller.setCalibrating(false);

    if (done < 4) {
        Serial.println("[MOTORCHECK] Stopped.");
        return;
    }
    Serial.printf("[MOTORCHECK] done | LEFT forward: enc L %+.0f R %+.0f, heading %+.1f | LEFT backward: enc L %+.0f R %+.0f, heading %+.1f | "
                  "RIGHT forward: enc L %+.0f R %+.0f, heading %+.1f | RIGHT backward: enc L %+.0f R %+.0f, heading %+.1f\n",
                  steps[0].left_mm, steps[0].right_mm, steps[0].heading_deg, steps[1].left_mm, steps[1].right_mm, steps[1].heading_deg,
                  steps[2].left_mm, steps[2].right_mm, steps[2].heading_deg, steps[3].left_mm, steps[3].right_mm, steps[3].heading_deg);
}

} // namespace MotorCheck
