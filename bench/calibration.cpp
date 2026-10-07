// Extra text commands that exist only in the `calibration` firmware build
// (pio run -e calibration -t upload). The normal robot firmware contains none of this.
//
//   motorcal            Balance the two motors and save the trims
//   motorrpm [duty]     Tachometer benchmark at a fixed duty (default 0.5)
//   dyno ...            Protocol spoken by the dyno test stand (tools/dyno_station)

#include <functional>
#include "robot.h"
#include "ui.h"

// ==============================================================================
// DYNO TEST-STAND PROTOCOL (tools/dyno_station), commands start with "dyno"
// ==============================================================================

#include <Arduino.h>
#include <functional>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Text protocol spoken by the optical dyno test stand (tools/dyno_station) while it auto-tunes
// the robot. All commands start with "dyno".
namespace DynoProtocol {

// Process a dyno command. Returns true if it was one.
bool handleCommand(const String& cmd, Motors& motors, MotionController& motion,
                   QueueHandle_t motion_queue, std::function<void(const char*)> reply);

} // namespace DynoProtocol

namespace DynoProtocol {

bool handleCommand(const String& cmd, Motors& motors, MotionController& motion,
                   QueueHandle_t motion_queue, std::function<void(const char*)> reply) {
    if (!cmd.startsWith("dyno")) return false;

    if (cmd == "dyno ping") {
        reply("DYNO_ACK READY");
        return true;
    }

    if (cmd.startsWith("dyno step ")) {
        float dl = 0.0f, dr = 0.0f;
        int dur_ms = 1000;
        if (sscanf(cmd.c_str() + 10, "%f %f %d", &dl, &dr, &dur_ms) >= 2) {
            motion.setCalibrating(true);
            motors.setRawEffort(dl, dr);
            delay(dur_ms);
            motors.brake();
            motion.setCalibrating(false);
            reply("DYNO_ACK STEP_DONE");
        } else {
            reply("ERR: USAGE 'dyno step <dl> <dr> <ms>'");
        }
        return true;
    }

    if (cmd.startsWith("dyno trap ")) {
        float spd = 300.0f, acc = 2000.0f, dist = 500.0f;
        if (sscanf(cmd.c_str() + 10, "%f %f %f", &spd, &acc, &dist) >= 3) {
            MotionCommand mc;
            mc.action = ACTION_MOVE_DISTANCE;
            mc.param_value = dist;
            mc.max_speed_mm_s = spd;
            mc.acceleration = acc;
            mc.enable_wall_centering = false;
            mc.entry_speed_mm_s = 0.0f;
            mc.exit_speed_mm_s = 0.0f;
            mc.start_offset_mm = 0.0f;
            xQueueSend(motion_queue, &mc, 0);
            reply("DYNO_ACK TRAP_STARTED");
        } else {
            reply("ERR: USAGE 'dyno trap <spd> <acc> <dist>'");
        }
        return true;
    }

    if (cmd.startsWith("dyno trim ")) {
        float tl = 1.0f, tr = 1.0f;
        if (sscanf(cmd.c_str() + 10, "%f %f", &tl, &tr) == 2) {
            motors.setTrim(tl, tr);
            reply("DYNO_ACK TRIM_SET");
        }
        return true;
    }

    if (cmd.startsWith("dyno deadband ")) {
        float dl = 0.02f, dr = 0.02f;
        if (sscanf(cmd.c_str() + 14, "%f %f", &dl, &dr) == 2) {
            motors.setDeadband(dl, dr);
            reply("DYNO_ACK DEADBAND_SET");
        }
        return true;
    }

    if (cmd.startsWith("dyno sync ")) {
        float ks = 0.0004f;
        if (sscanf(cmd.c_str() + 10, "%f", &ks) == 1) {
            motion.setSyncGain(ks);
            reply("DYNO_ACK SYNC_SET");
        }
        return true;
    }

    if (cmd.startsWith("dyno pid ")) {
        float kp = 0.0025f, ki = 0.0005f, kd = 0.00005f;
        if (sscanf(cmd.c_str() + 9, "%f %f %f", &kp, &ki, &kd) == 3) {
            motion.setLinearVelGains(kp, ki, kd);
            reply("DYNO_ACK PID_SET");
        }
        return true;
    }

    if (cmd == "dyno save") {
        motors.saveToNVS();
        motion.saveToNVS();
        reply("DYNO_ACK SAVED_TO_NVS");
        return true;
    }

    if (cmd == "dyno get") {
        float tl = 1.0f, tr = 1.0f, dl = 0.02f, dr = 0.02f;
        motors.getTrim(tl, tr);
        motors.getDeadband(dl, dr);
        float ks = motion.getSyncGain();
        float kp = 0, ki = 0, kd = 0;
        motion.getLinearVelGains(kp, ki, kd);
        char buf[128];
        snprintf(buf, sizeof(buf), "DYNO_PARAMS: Trim=[%.3f,%.3f] Dead=[%.3f,%.3f] Sync=%.6f PID=[%.5f,%.5f,%.6f]",
                 tl, tr, dl, dr, ks, kp, ki, kd);
        reply(buf);
        return true;
    }

    return false;
}

} // namespace DynoProtocol

// ==============================================================================
// MOTOR BALANCING (robot on a stand, wheels free)
// ==============================================================================

#include <Arduino.h>

// Bench routines for balancing the two motors. Run with the robot on a stand, wheels free.
namespace MotorCalibration {

// Spins both motors at 25 / 50 / 75 %, measures the speed mismatch with the encoders, and saves
// trim multipliers to flash so both wheels match. Returns false if a wheel did not turn.
bool calibrateMotors();

// Spins both motors at a fixed duty and prints each wheel's RPM twice a second, for checking
// against an external tachometer.
void runTachometerBenchmark(float duty = 0.5f, uint16_t duration_ms = 4000);

} // namespace MotorCalibration

namespace MotorCalibration {

bool calibrateMotors() {
    Serial.println("\n=======================================================");
    Serial.println("  AUTOMATED MOTOR SPEED & TRIM BALANCING BENCHMARK");
    Serial.println("=======================================================");
    Serial.println("[CALIB] Place robot on a stand/box with wheels freewheeling.");
    Serial.println("[CALIB] Starting multi-point optical tachometer routine in 1.5s...");
    delay(1500);

    g_motion_controller.setCalibrating(true);
    g_motors.coast();
    delay(200);

    // Test points: 25%, 50%, 75% raw duty
    const float test_duties[] = {0.25f, 0.50f, 0.75f};
    const int num_points = 3;
    float ratios[3] = {1.0f, 1.0f, 1.0f};

    for (int i = 0; i < num_points; i++) {
        float duty = test_duties[i];
        Serial.printf("\n[CALIB] Step %d/%d: Testing raw duty %2.0f%%...\n", i + 1, num_points, duty * 100.0f);

        // Spin up with raw effort (bypassing current trim)
        g_motors.setRawEffort(duty, duty);
        delay(400); // Wait for motors to reach steady-state velocity

        // Measure encoder tick deltas over 1000ms
        uint32_t t_start = millis();
        EncoderState enc_start = g_encoders.getState();
        int32_t start_ticks_l = enc_start.left_ticks_total;
        int32_t start_ticks_r = enc_start.right_ticks_total;

        delay(1000);

        uint32_t t_end = millis();
        EncoderState enc_end = g_encoders.getState();
        int32_t end_ticks_l = enc_end.left_ticks_total;
        int32_t end_ticks_r = enc_end.right_ticks_total;

        float dt_s = (float)(t_end - t_start) / 1000.0f;
        int32_t delta_l = abs(end_ticks_l - start_ticks_l);
        int32_t delta_r = abs(end_ticks_r - start_ticks_r);

        // Safety check: verify wheels are actually turning
        if (delta_l < 30 || delta_r < 30) {
            Serial.printf("[CALIB ERROR] Wheel rotation not detected! (L_ticks=%d, R_ticks=%d)\n", delta_l, delta_r);
            Serial.println("[CALIB ERROR] Ensure 12V boost is active and robot wheels are not physically jammed.");
            g_motors.coast();
            g_motion_controller.setCalibrating(false);
            g_motion_controller.resetTracking();
            return false;
        }

        // RPM = (Delta_Ticks / ENCODER_TOTAL_CPR) / dt_s * 60
        float rpm_l = ((float)delta_l / ENCODER_TOTAL_CPR) / dt_s * 60.0f;
        float rpm_r = ((float)delta_r / ENCODER_TOTAL_CPR) / dt_s * 60.0f;

        ratios[i] = rpm_l / rpm_r;
        Serial.printf("[CALIB] -> Left: %6.1f RPM | Right: %6.1f RPM | Ratio (L/R): %.4f\n", rpm_l, rpm_r, ratios[i]);
    }

    g_motors.coast();
    delay(300);

    // Compute average ratio across test points
    float avg_ratio = (ratios[0] + ratios[1] + ratios[2]) / 3.0f;

    // Calculate trims: scale down the faster motor so both match the slower one
    float trim_left = 1.0f;
    float trim_right = 1.0f;

    if (avg_ratio > 1.002f) {
        // Left is faster -> trim Left down
        trim_left = 1.0f / avg_ratio;
        trim_right = 1.0f;
    } else if (avg_ratio < 0.998f) {
        // Right is faster -> trim Right down
        trim_left = 1.0f;
        trim_right = avg_ratio;
    }

    // Clamp trim values between 0.60 and 1.00 for safety
    trim_left = constrain(trim_left, 0.60f, 1.0f);
    trim_right = constrain(trim_right, 0.60f, 1.0f);

    // Apply and persist to Flash NVS
    g_motors.setTrim(trim_left, trim_right);
    g_motors.saveToNVS();

    // Verification test: run at 50% duty with trims applied
    Serial.println("\n[CALIB] Running post-trim verification at 50% duty...");
    g_motors.setEffort(0.50f, 0.50f);
    delay(400);

    uint32_t v_start = millis();
    EncoderState v_enc_start = g_encoders.getState();
    int32_t v_ticks_l_start = v_enc_start.left_ticks_total;
    int32_t v_ticks_r_start = v_enc_start.right_ticks_total;

    delay(1000);

    uint32_t v_end = millis();
    EncoderState v_enc_end = g_encoders.getState();
    float v_dt = (float)(v_end - v_start) / 1000.0f;
    int32_t v_delta_l = abs(v_enc_end.left_ticks_total - v_ticks_l_start);
    int32_t v_delta_r = abs(v_enc_end.right_ticks_total - v_ticks_r_start);

    g_motors.coast();

    float v_rpm_l = ((float)v_delta_l / ENCODER_TOTAL_CPR) / v_dt * 60.0f;
    float v_rpm_r = ((float)v_delta_r / ENCODER_TOTAL_CPR) / v_dt * 60.0f;
    float diff_rpm = v_rpm_l - v_rpm_r;
    float diff_pct = (fabsf(diff_rpm) / ((v_rpm_l + v_rpm_r) * 0.5f)) * 100.0f;

    Serial.println("=======================================================");
    Serial.println("         CALIBRATION RESULTS & TRIM SAVED              ");
    Serial.println("=======================================================");
    Serial.printf(" Raw Speed Ratio (L/R) : %.4f\n", avg_ratio);
    Serial.printf(" Trim Multipliers      : Left = %.4f | Right = %.4f\n", trim_left, trim_right);
    Serial.printf(" Verified 50%% RPM      : Left = %6.1f | Right = %6.1f\n", v_rpm_l, v_rpm_r);
    Serial.printf(" RPM Discrepancy       : %+.1f RPM (%.2f%% difference)\n", diff_rpm, diff_pct);
    Serial.println("=======================================================\n");

    g_motion_controller.setCalibrating(false);
    g_motion_controller.resetTracking();
    return true;
}

void runTachometerBenchmark(float duty, uint16_t duration_ms) {
    g_motion_controller.setCalibrating(true);
    duty = constrain(duty, 0.1f, 1.0f);

    float trim_l = 1.0f, trim_r = 1.0f;
    g_motors.getTrim(trim_l, trim_r);

    Serial.println("\n-------------------------------------------------------");
    Serial.printf(" [TACH] RUNNING BENCHMARK @ %2.0f%% DUTY (Duration: %d ms)\n", duty * 100.0f, duration_ms);
    Serial.printf(" [TACH] Active Trims: Left=%.4f, Right=%.4f\n", trim_l, trim_r);
    Serial.println(" [TACH] Point your external optical RPM tachometer gun now!");
    Serial.println("-------------------------------------------------------");

    g_motors.setEffort(duty, duty);
    delay(300); // Settle time

    uint32_t start_time = millis();
    uint32_t last_sample_time = millis();
    int32_t last_ticks_l = g_encoders.getState().left_ticks_total;
    int32_t last_ticks_r = g_encoders.getState().right_ticks_total;

    while (millis() - start_time < duration_ms) {
        delay(500);

        uint32_t now = millis();
        float dt_s = (float)(now - last_sample_time) / 1000.0f;
        EncoderState enc = g_encoders.getState();
        int32_t delta_l = abs(enc.left_ticks_total - last_ticks_l);
        int32_t delta_r = abs(enc.right_ticks_total - last_ticks_r);

        last_sample_time = now;
        last_ticks_l = enc.left_ticks_total;
        last_ticks_r = enc.right_ticks_total;

        float rpm_l = ((float)delta_l / ENCODER_TOTAL_CPR) / dt_s * 60.0f;
        float rpm_r = ((float)delta_r / ENCODER_TOTAL_CPR) / dt_s * 60.0f;
        float diff = rpm_l - rpm_r;

        Serial.printf("[TACH +%4d ms] Left: %6.1f RPM | Right: %6.1f RPM | Diff: %+5.1f RPM\n",
                      (int)(now - start_time), rpm_l, rpm_r, diff);
    }

    g_motors.coast();
    Serial.println("[TACH] Benchmark complete. Motors stopped.\n");

    g_motion_controller.setCalibrating(false);
    g_motion_controller.resetTracking();
}

} // namespace MotorCalibration

// ==============================================================================
// COMMAND DISPATCH
// ==============================================================================


namespace Calibration {

bool handleCommand(const String& cmd, std::function<void(const char*)> reply) {
    if (cmd == "motorcal") {
        if (Actions::isRunActive()) {
            reply("ERR: CANNOT_CALIB_WHILE_RUNNING");
            return true;
        }
        reply("Starting automated motor speed calibration (wheels must be freewheeling)...");
        StatusLED::set(StatusLED::YELLOW);
        if (MotorCalibration::calibrateMotors()) {
            StatusLED::flash(StatusLED::GREEN, 4, 100);
            float trim_left = 1.0f, trim_right = 1.0f;
            g_motors.getTrim(trim_left, trim_right);
            char buf[64];
            snprintf(buf, sizeof(buf), "ACK: MOTOR CALIB SUCCESS (L=%.4f, R=%.4f)", trim_left, trim_right);
            reply(buf);
        } else {
            StatusLED::flash(StatusLED::RED, 4, 100);
            reply("ERR: MOTOR CALIB FAILED (check wheels or battery)");
        }
        Actions::showSelectedMode();
        return true;
    }

    if (cmd.startsWith("motorrpm")) {
        if (Actions::isRunActive()) {
            reply("ERR: CANNOT_BENCHMARK_WHILE_RUNNING");
            return true;
        }
        float duty = 0.5f;
        int space_idx = cmd.indexOf(' ');
        if (space_idx > 0) {
            float parsed = cmd.substring(space_idx + 1).toFloat();
            if (parsed > 0.05f && parsed <= 1.0f) duty = parsed;
        }
        char buf[64];
        snprintf(buf, sizeof(buf), "Running tachometer benchmark at %.0f%% duty for 4000ms...", duty * 100.0f);
        reply(buf);
        StatusLED::set(StatusLED::CYAN);
        MotorCalibration::runTachometerBenchmark(duty, 4000);
        reply("ACK: TACH BENCHMARK FINISHED");
        Actions::showSelectedMode();
        return true;
    }

    return DynoProtocol::handleCommand(cmd, g_motors, g_motion_controller, g_motion_cmd_queue, reply);
}

} // namespace Calibration
