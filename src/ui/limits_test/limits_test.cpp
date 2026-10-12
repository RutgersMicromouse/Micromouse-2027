#include <Preferences.h>
#include "robot.h"
#include "ui/status_led/status_led.h"
#include "ui/actions/actions.h"
#include "ui/limits_test/limits_test.h"

// ==============================================================================
// LIMITS TEST
// ==============================================================================

namespace LimitsTest {

static const uint8_t kSteps[] = LIMITS_STEPS;

static constexpr bool passes(float stop_error_mm, float heading_error_deg, float turn_error_deg, float tilt_deg, bool braked) {
    return !braked && tilt_deg <= LIMITS_TILT_TOLERANCE_DEG &&
           stop_error_mm     <= LIMITS_STOP_TOLERANCE_MM     && stop_error_mm     >= -LIMITS_STOP_TOLERANCE_MM &&
           heading_error_deg <= LIMITS_HEADING_TOLERANCE_DEG && heading_error_deg >= -LIMITS_HEADING_TOLERANCE_DEG &&
           turn_error_deg    <= LIMITS_TURN_TOLERANCE_DEG    && turn_error_deg    >= -LIMITS_TURN_TOLERANCE_DEG;
}
static_assert(passes(0.0f, 0.0f, 0.0f, 0.0f, false), "a perfect run passes");
static_assert(!passes(0.0f, 0.0f, 0.0f, 0.0f, true), "a run the safety supervisor braked fails");
static_assert(!passes(-LIMITS_STOP_TOLERANCE_MM - 1.0f, 0.0f, 0.0f, 0.0f, false), "stopping too far past the centre fails");
static_assert(!passes(0.0f, 0.0f, LIMITS_TURN_TOLERANCE_DEG + 1.0f, 0.0f, false), "a turn too far off fails");
static_assert(!passes(0.0f, 0.0f, 0.0f, LIMITS_TILT_TOLERANCE_DEG + 1.0f, false), "tipping under braking or acceleration fails");

// Sends one move and waits for it. False if a STOP came, it took too long, or the motors are held.
// `tilt_max_deg`, if given, comes back as the most the robot tipped during the move.
static bool drive(const MotionCommand& move, uint32_t timeout_ms, uint32_t stops_before, float* tilt_max_deg = nullptr) {
    xQueueSend(g_motion_cmd_queue, &move, 0);
    const uint32_t started = millis();
    // (wait for the motion task to take it up before waiting for it to finish)
    while (g_motion_controller.isCommandFinished() && millis() - started < 300) vTaskDelay(pdMS_TO_TICKS(2));
    while (!g_motion_controller.isCommandFinished()) {
        if (stopCount() != stops_before) return false;
        if (millis() - started > timeout_ms) { requestStop(); return false; }
        if (tilt_max_deg) *tilt_max_deg = fmaxf(*tilt_max_deg, g_imu.getTiltDeg());
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelay(pdMS_TO_TICKS(300)); // Come to rest, and let the readings settle
    return stopCount() == stops_before;
}

int run(int cells) {
    cells = constrain(cells, 2, 15);
    const float run_mm = (float)(cells - 1) * MAZE_CELL_SIZE_MM;
    uint32_t stops_before = stopCount();

    if (!g_ir_sensors.isFrontLevelMeasured()) {
        Serial.println("[LIMITS] Not started: calibrate first (the front level has to be measured against a wall, or it cannot tell where it stopped).");
        return 0;
    }
    prepareForNewRun(); // The robot has just been put down by hand
    vTaskDelay(pdMS_TO_TICKS(60));
    Serial.printf("[LIMITS] Starting in a closed corridor of %d cells: %.0f mm each way, harder each time. Passing = stop within %.0f mm, straight within %.0f deg, turn within %.0f deg, no safety braking.\n",
                  cells, run_mm, (float)LIMITS_STOP_TOLERANCE_MM, (float)LIMITS_HEADING_TOLERANCE_DEG, (float)LIMITS_TURN_TOLERANCE_DEG);

    int best = 0;
    const char* ended = "every level passed";
    for (uint8_t percent : kSteps) {
        bool level_ok = true;
        for (int leg = 0; leg < 2 && level_ok; ++leg) {
            const float scale = (float)percent / 100.0f;
            const uint16_t brakings_before = g_safety.getStatus().brakings;

            // The straight, as planSpeedRun sends one: distance by the wheels, no front-wall stop
            MotionCommand straight = {};
            straight.action = ACTION_MOVE_DISTANCE;
            straight.param_value = run_mm;
            straight.max_speed_mm_s = SPEEDRUN_CRUISE_SPEED_MM_S * scale;
            straight.acceleration = SPEEDRUN_ACCEL_MM_S2 * scale;
            straight.enable_wall_centering = true;
            float tilt = 0.0f; // Hard braking tips it onto its nose; hard acceleration lifts the nose
            const bool finished = drive(straight, 4000 + (uint32_t)(run_mm * 20.0f), stops_before, &tilt);
            const bool braked = g_safety.getStatus().brakings != brakings_before;
            if (braked) stops_before = stopCount(); // (a safety braking counts as a stop; it is this level's failure, not a STOP by hand)
            // (stopped by hand: nothing it found is applied)
            if (!finished && !braked) { ended = "stopped by hand"; level_ok = false; best = -1; break; }

            const float heading_error = g_motion_controller.getHeadingErrorDeg();
            const IRReadings ir = g_ir_sensors.getReadings();
            if (!ir.wall_front) {
                Serial.printf("[LIMITS] %d%% run %d: no wall in front at the end. The corridor must be closed at both ends and exactly %d cells long. Stopped.\n",
                              (int)percent, leg + 1, cells);
                ended = "no end wall seen";
                level_ok = false;
                break;
            }
            const float stop_error = ir.front_offset_mm; // + = short of the cell centre, - = past it

            // Let the safety supervisor let go, then step onto the cell centre so the next run starts true
            const uint32_t held_since = millis();
            while (g_motors.isInhibited() && millis() - held_since < 4000) vTaskDelay(pdMS_TO_TICKS(20));
            float turn_error = 0.0f;
            bool moved_on = !g_motors.isInhibited();
            if (moved_on && fabsf(stop_error) > 3.0f) {
                MotionCommand nudge = {};
                nudge.action = ACTION_MOVE_DISTANCE;
                nudge.param_value = constrain(stop_error, -40.0f, 40.0f);
                nudge.max_speed_mm_s = 80.0f;
                nudge.acceleration = 400.0f;
                moved_on = drive(nudge, 3000, stops_before);
            }
            if (moved_on) {
                MotionCommand turn = {};
                turn.action = ACTION_TURN_AROUND_180;
                turn.param_value = 180.0f;
                turn.max_speed_mm_s = SEARCH_TURN_SPEED_DEG_S;
                turn.acceleration = SEARCH_TURN_ACCEL_DEG_S2;
                moved_on = drive(turn, 6000, stops_before);
                turn_error = g_motion_controller.getHeadingErrorDeg();
            }

            const bool ok = moved_on && passes(stop_error, heading_error, turn_error, tilt, braked);
            Serial.printf("[LIMITS] %d%% run %d | %.0f mm/s, %.0f mm/s2 | stopped %+.0f mm from the centre (+ short, - past) | heading %+.1f deg off | turn %+.1f deg off | tipped %.1f deg | safety %s | %s\n",
                          (int)percent, leg + 1, straight.max_speed_mm_s, straight.acceleration, stop_error,
                          heading_error, turn_error, tilt, braked ? "BRAKED" : "quiet", ok ? "pass" : "FAIL");
            if (!moved_on) { ended = "stopped by hand"; level_ok = false; best = -1; break; }
            if (!ok) { ended = "a level failed"; level_ok = false; }
        }
        if (!level_ok) break;
        best = percent;
    }

    if (best > 0) {
        Actions::setSpeedLimitPercent((uint8_t)best);
        Serial.printf("[LIMITS] done | %s | best level that passed: %d%% | speed runs are now held to %d%% (saved; \"limits clear\" removes it)\n",
                      ended, best, best);
    } else {
        Serial.printf("[LIMITS] done | %s | no level passed, so the ceiling is unchanged (%d%%)\n", ended, (int)Actions::getSpeedLimitPercent());
    }
    return best;
}

// ------------------------------------------------------------------------------
// Braking, measured
// ------------------------------------------------------------------------------

void begin() {
    Preferences prefs;
    prefs.begin("hand_cfg", true);
    const float saved = prefs.getFloat("brake", 0.0f);
    const float delay_s = prefs.getFloat("lat", SAFETY_LATENCY_S);
    prefs.end();
    if (saved > 0.0f) {
        SafetySupervisor::setBrakeModel(saved, delay_s);
        Serial.printf("[LIMITS] Braking measured on this robot: the safety sum counts on %.0f mm/s2 after %.0f ms.\n", saved, delay_s * 1000.0f);
    }
}

// Mean of the two wheels' distance
static float wheelsMm() {
    const EncoderState enc = g_encoders.getState();
    return 0.5f * (enc.left_dist_mm + enc.right_dist_mm);
}

void measureBraking() {
    static const uint8_t kLevels[] = { 40, 70, 100 };
    Serial.println("[BRAKE] Needs 1.2 m of clear floor straight ahead. Three runs, each a little further on: up to speed, then the brake the safety supervisor uses.");
    float speeds[3], stops[3];
    int measured = 0;
    for (uint8_t percent : kLevels) {
        prepareForNewRun();
        vTaskDelay(pdMS_TO_TICKS(60));
        const uint32_t stops_before = stopCount();
        const float target = SPEEDRUN_CRUISE_SPEED_MM_S * (float)percent / 100.0f;
        const float brake_at_mm = target * target / (2.0f * SPEEDRUN_ACCEL_MM_S2) + 80.0f; // Up to speed, and 80 mm at it

        MotionCommand go = {};
        go.action = ACTION_MOVE_DISTANCE;
        go.param_value = brake_at_mm + 250.0f;
        go.max_speed_mm_s = target;
        go.acceleration = SPEEDRUN_ACCEL_MM_S2;
        xQueueSend(g_motion_cmd_queue, &go, 0);

        // The speed is taken from the distance covered in the last 20 ms: the speed the encoders
        // report is smoothed too heavily to say what it was at the moment the brake went on
        const uint32_t started = millis();
        float before_mm = 0.0f, now_mm = 0.0f;
        while (now_mm < brake_at_mm && millis() - started < 4000 && stopCount() == stops_before) {
            before_mm = now_mm;
            vTaskDelay(pdMS_TO_TICKS(20));
            now_mm = wheelsMm();
        }
        if (stopCount() != stops_before) { Serial.println("[BRAKE] Stopped."); return; }
        const float speed = (now_mm - before_mm) / 0.020f;
        float gap_before = 0.0f, gap_after = 0.0f;
        const bool gap_known = g_ir_sensors.getFrontGapMm(gap_before);
        requestStop(); // Brakes on the motion task's next tick, as a safety stop does

        // Standing still = the wheels have not moved for a tenth of a second
        float last_mm = now_mm;
        const uint32_t braked_at = millis(); // (and never wait for it longer than 3 s, whatever the encoders say)
        for (int still = 0; still < 5 && millis() - braked_at < 3000; ) {
            vTaskDelay(pdMS_TO_TICKS(20));
            const float mm = wheelsMm();
            still = (fabsf(mm - last_mm) < 0.3f) ? still + 1 : 0;
            last_mm = mm;
        }
        float stop_mm = last_mm - now_mm;
        // Locked wheels slide while their encoders stand still: where a front wall is in view, the
        // front sensors say how far the robot really went, and the longer of the two is believed
        const bool by_sensors = gap_known && g_ir_sensors.getFrontGapMm(gap_after) && (gap_before - gap_after) > stop_mm;
        if (by_sensors) stop_mm = gap_before - gap_after;
        if (speed < 0.6f * target || stop_mm < 1.0f) {
            Serial.printf("[BRAKE] %d%%: only reached %.0f mm/s of %.0f, or did not move while braking: not counted.\n", (int)percent, speed, target);
            continue;
        }
        Serial.printf("[BRAKE] %d%% | from %.0f mm/s | stopped in %.0f mm (%s)\n",
                      (int)percent, speed, stop_mm, by_sensors ? "by the front sensors" : "by the wheels");
        speeds[measured] = speed;
        stops[measured] = stop_mm;
        measured++;
    }
    if (measured < 2) { Serial.println("[BRAKE] done | fewer than two speeds measured, the safety sum is unchanged"); return; }
    // A stop is a delay (the robot runs on at speed until the brake bites) and then the braking:
    //   distance = speed x delay + speed^2 / (2 x deceleration)
    // so distance / speed against speed is a straight line: it starts at the delay and climbs by
    // 1 / (2 x deceleration). The line through the measured points gives both.
    float sum_v = 0.0f, sum_y = 0.0f, sum_vv = 0.0f, sum_vy = 0.0f;
    for (int i = 0; i < measured; ++i) {
        const float y = stops[i] / speeds[i];
        sum_v += speeds[i]; sum_y += y; sum_vv += speeds[i] * speeds[i]; sum_vy += speeds[i] * y;
    }
    const float spread = measured * sum_vv - sum_v * sum_v;
    float climb = spread > 1.0f ? (measured * sum_vy - sum_v * sum_y) / spread : 0.0f;
    climb = fmaxf(climb, 1.0f / (2.0f * 8000.0f)); // Never credit it with more than 8000 mm/s2
    const float decel = 1.0f / (2.0f * climb);
    const float delay_s = fmaxf(0.0f, (sum_y - climb * sum_v) / measured);
    // A fifth taken off the braking and added to the delay, for a tired battery or a dusty floor
    const float use_decel = constrain(0.8f * decel, 500.0f, 8000.0f);
    const float use_delay = constrain(1.2f * delay_s, 0.010f, 0.200f);
    const float was = SafetySupervisor::stoppingDistanceMm(500.0f);
    SafetySupervisor::setBrakeModel(use_decel, use_delay);
    Preferences prefs;
    prefs.begin("hand_cfg", false);
    prefs.putFloat("brake", use_decel);
    prefs.putFloat("lat", use_delay);
    prefs.end();
    Serial.printf("[BRAKE] done | measured: brakes at %.0f mm/s2 after a delay of %.0f ms | the safety sum now counts on %.0f mm/s2 after %.0f ms | "
                  "from 500 mm/s that allows %.0f mm (it was %.0f) | saved\n",
                  decel, delay_s * 1000.0f, use_decel, use_delay * 1000.0f, SafetySupervisor::stoppingDistanceMm(500.0f), was);
}

} // namespace LimitsTest
