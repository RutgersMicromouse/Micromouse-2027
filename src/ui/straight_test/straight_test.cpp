#include "robot.h"
#include "ui/straight_test/straight_test.h"

// ==============================================================================
// STRAIGHT TEST
// ==============================================================================

namespace StraightTest {

static const uint32_t SAMPLE_MS      = 20;     // 50 readings a second
static const float    SWING_DEG_S    = 15.0f;  // A turn rate below this is not counted as a swing
static const float    MIN_STRETCH_MM = 25.0f;  // A change of walls shorter than this is a flicker
static const int      MAX_STRETCHES  = 8;

// One stretch of the straight over which the same side walls were in view
struct Stretch {
    bool left, right;
    float from_mm, to_mm;
    int samples;
    float sum_dev, sum_dev_sq, most_dev;   // Heading away from the start heading, degrees
    float sum_centre, sum_centre_sq;       // Wall-centring error
};

static void addSample(Stretch& s, float dev, float centre) {
    s.samples++;
    s.sum_dev += dev;
    s.sum_dev_sq += dev * dev;
    if (fabsf(dev) > fabsf(s.most_dev)) s.most_dev = dev;
    s.sum_centre += centre;
    s.sum_centre_sq += centre * centre;
}

static float spread(float sum, float sum_sq, int n) {
    if (n < 2) return 0.0f;
    const float mean = sum / n;
    return sqrtf(fmaxf(0.0f, sum_sq / n - mean * mean));
}

static const char* wallsName(bool left, bool right) {
    return left ? (right ? "BOTH walls" : "LEFT wall only") : (right ? "RIGHT wall only" : "NO side walls");
}

void run(int cells) {
    cells = constrain(cells, 1, 15);
    const uint32_t stops_before = stopCount();
    // The robot has just been put down by hand: start from where it stands and points now
    prepareForNewRun();

    // (the heading and the wheel counters are zeroed for the new run a moment after that call
    // returns, so the starting values are read only after a short wait: read at once, the first
    // test on the robot took 180 degrees as its start and every heading figure came out wrong)
    vTaskDelay(pdMS_TO_TICKS(60));
    const float start_heading = g_imu.getHeadingDeg();
    const EncoderState enc_start = g_encoders.getState();
    Serial.printf("[STRAIGHTTEST] Starting: %d cell(s) forward as the search drives them (%.0f mm/s, wall centring on).\n",
                  cells, SEARCH_SPEED_DEFAULT_MM_S);

    MotionCommand move = {};
    move.action = ACTION_MOVE_FORWARD_CELLS;
    move.param_value = (float)cells;
    move.max_speed_mm_s = SEARCH_SPEED_DEFAULT_MM_S;
    move.acceleration = SEARCH_ACCEL_DEFAULT_MM_S2;
    move.enable_wall_centering = true;
    move.stop_at_front_wall = true;
    move.steer_by_side_sensors = true;
    xQueueSend(g_motion_cmd_queue, &move, 0);

    Stretch stretches[MAX_STRETCHES] = {};
    int n = 0;
    Stretch whole = {};
    int swings = 0, last_swing_sign = 0;
    float most_rate = 0.0f;
    const uint32_t started = millis();
    const uint32_t timeout_ms = 3000 + (uint32_t)cells * 2500;
    bool stopped = false;

    vTaskDelay(pdMS_TO_TICKS(10)); // Let the motion task pick it up
    while (!g_motion_controller.isCommandFinished()) {
        if (stopCount() != stops_before) { stopped = true; break; }
        if (millis() - started > timeout_ms) {
            Serial.println("[STRAIGHTTEST] The move did not finish in time: stopped.");
            requestStop();
            stopped = true;
            break;
        }
        const IRReadings ir = g_ir_sensors.getReadings();
        const EncoderState enc = g_encoders.getState();
        const float travelled = ((enc.left_dist_mm - enc_start.left_dist_mm) + (enc.right_dist_mm - enc_start.right_dist_mm)) * 0.5f;
        const float dev = normalizeAngle180(g_imu.getHeadingDeg() - start_heading);
        const float rate = g_imu.getGyroZ();

        // A swing = the turn rate changing from clearly one way to clearly the other
        most_rate = fmaxf(most_rate, fabsf(rate));
        const int sign = (rate > SWING_DEG_S) ? 1 : ((rate < -SWING_DEG_S) ? -1 : 0);
        if (sign != 0 && sign != last_swing_sign) {
            if (last_swing_sign != 0) swings++;
            last_swing_sign = sign;
        }

        // A new stretch when the side walls in view change (a short flicker is folded back in below)
        if (n == 0 || stretches[n - 1].left != ir.wall_left || stretches[n - 1].right != ir.wall_right) {
            if (n > 0 && (stretches[n - 1].to_mm - stretches[n - 1].from_mm) < MIN_STRETCH_MM) n--;
            if (n > 0 && stretches[n - 1].left == ir.wall_left && stretches[n - 1].right == ir.wall_right) {
                // Back to the walls seen before the flicker: carry that stretch on
            } else if (n < MAX_STRETCHES) {
                stretches[n] = Stretch{};
                stretches[n].left = ir.wall_left;
                stretches[n].right = ir.wall_right;
                stretches[n].from_mm = travelled;
                n++;
            }
        }
        if (n > 0) {
            stretches[n - 1].to_mm = travelled;
            addSample(stretches[n - 1], dev, ir.centering_error);
        }
        addSample(whole, dev, ir.centering_error);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }
    if (stopped) {
        Serial.println("[STRAIGHTTEST] Stopped.");
        return;
    }
    const float took_s = (millis() - started) / 1000.0f;
    vTaskDelay(pdMS_TO_TICKS(300)); // Let it come to rest before the end readings

    for (int i = 0; i < n; ++i) {
        const Stretch& s = stretches[i];
        if (s.samples == 0) continue;
        Serial.printf("[STRAIGHT] %4.0f to %4.0f mm | %s | heading off start: mean %+.1f, spread %.1f, most %+.1f deg | centring error: mean %+.3f, spread %.3f\n",
                      s.from_mm, s.to_mm, wallsName(s.left, s.right), s.sum_dev / s.samples,
                      spread(s.sum_dev, s.sum_dev_sq, s.samples), s.most_dev,
                      s.sum_centre / s.samples, spread(s.sum_centre, s.sum_centre_sq, s.samples));
    }

    const IRReadings ir = g_ir_sensors.getReadings();
    const EncoderState enc = g_encoders.getState();
    const MotionController& mc = g_motion_controller;
    char front[48] = "no wall in front";
    if (ir.wall_front) snprintf(front, sizeof(front), "front wall: %+.0f mm from the cell centre", ir.front_offset_mm);
    Serial.printf("[STRAIGHTTEST] done | %d cells in %.2f s | wheels left %+.0f, right %+.0f mm (%.0f wanted) | "
                  "heading off start: mean %+.1f, spread %.1f, most %+.1f, at the end %+.1f deg | swings %d, fastest %.0f deg/s | %s | "
                  "end IR L90=%d R90=%d | supply %.1f V | h_kp=%.4g h_kd=%.4g w_kp=%.4g w_kd=%.4g w_max=%.4g w_gyro=%.4g s_damp=%.4g v_kp=%.4g k_sync=%.4g dist_k=%.4g\n",
                  cells, took_s, enc.left_dist_mm - enc_start.left_dist_mm, enc.right_dist_mm - enc_start.right_dist_mm,
                  cells * MAZE_CELL_SIZE_MM,
                  whole.samples ? whole.sum_dev / whole.samples : 0.0f, spread(whole.sum_dev, whole.sum_dev_sq, whole.samples),
                  whole.most_dev, normalizeAngle180(g_imu.getHeadingDeg() - start_heading), swings, most_rate, front,
                  (int)ir.left_90, (int)ir.right_90, g_motors.getSupplyVolts(),
                  mc.getTune(MotionController::TUNE_H_KP), mc.getTune(MotionController::TUNE_H_KD),
                  mc.getTune(MotionController::TUNE_W_KP), mc.getTune(MotionController::TUNE_W_KD),
                  mc.getTune(MotionController::TUNE_W_MAX), mc.getTune(MotionController::TUNE_W_GYRO),
                  mc.getTune(MotionController::TUNE_S_DAMP), mc.getTune(MotionController::TUNE_V_KP),
                  mc.getTune(MotionController::TUNE_K_SYNC), mc.getTune(MotionController::TUNE_DIST_K));
}

} // namespace StraightTest
