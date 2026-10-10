#include "navigation/navigator/navigator.h"

// ==============================================================================
// NAVIGATOR: SPEED-RUN PLANNER
// ==============================================================================

// Time a straight of `dist` mm takes when entered at v0 and left at v1, never faster than v_max
static float moveTime(float dist, float v_max, float accel, float v0, float v1) {
    if (dist <= 0.0f) return 0.0f;
    float d_up   = (v_max * v_max - v0 * v0) / (2.0f * accel);
    float d_down = (v_max * v_max - v1 * v1) / (2.0f * accel);
    if (d_up + d_down > dist) {
        // Too short to reach v_max: accelerate to a lower peak, then brake
        float v_peak = sqrtf((2.0f * accel * dist + v0 * v0 + v1 * v1) * 0.5f);
        v_peak = fmaxf(v_peak, fmaxf(v0, v1));
        return (v_peak - v0) / accel + (v_peak - v1) / accel;
    }
    return (v_max - v0) / accel + (v_max - v1) / accel + (dist - d_up - d_down) / v_max;
}

// Plans a whole speed run: turns the cell path into one unbroken chain of moves.
//
// The path is read as a list of turns, one per cell the path bends in:
//   a turn on its own                 -> smooth 90° curve, cell edge to cell edge
//   turns that alternate (L R L R..)  -> one diagonal, entered and left with smooth 45° curves
//   two turns the same way in a row   -> the diagonal doubles back: a smooth 90° "V" turn from
//       inside a diagonal                one diagonal onto the next
// The robot never stops between the start cell and the centre (unless the path reverses on
// itself, which a shortest path does not do).
//
// Distances: `pending` is straight-line distance still to be driven before the next turn. Every
// cell step adds 180 mm; each turn takes over part of the steps either side of it.
float Navigator::planSpeedRun(const Coordinate* path, uint8_t path_len, bool use_diagonals) {
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;
    sub_cmd_overflow_ = false;

    const float cruise     = SPEEDRUN_CRUISE_SPEED_MM_S * speed_scale_;
    const float accel      = SPEEDRUN_ACCEL_MM_S2       * speed_scale_;
    const float diag_speed = SPEEDRUN_DIAG_SPEED_MM_S   * speed_scale_;
    // Curves do not speed up with the rest beyond SPEEDRUN_CURVE_MAX_MM_S: each straight brakes
    // down to the curve speed before its curve, and the next one accelerates away again.
    const float turn_speed = fminf(SPEEDRUN_CURVE_SPEED_MM_S * speed_scale_, SPEEDRUN_CURVE_MAX_MM_S);
    const float v_speed    = turn_speed * V90_SPEED_RATIO;
    const float spin_speed = SPEEDRUN_TURN_SPEED_DEG_S  * speed_scale_;
    const float spin_accel = SPEEDRUN_TURN_ACCEL_DEG_S2 * speed_scale_;

    // Every run begins in the start cell facing into the maze
    pose_.cell_x = path[0].x;
    pose_.cell_y = path[0].y;
    pose_.current_dir = DIR_NORTH;

    static Direction dirs[256];
    const uint8_t steps = path_len - 1;
    for (uint8_t i = 0; i < steps; ++i) {
        dirs[i] = directionBetween(path[i], path[i + 1]);
    }
    // The turn made in cell i: 0 = straight on, 1 = right, 3 = left, 2 = back the way it came
    auto turnAt = [&](uint8_t i) -> int8_t { return (int8_t)(((int8_t)dirs[i] - (int8_t)dirs[i - 1] + 4) % 4); };

    float time_s  = 0.0f;
    float pending = 0.0f; // Straight distance not yet turned into a move
    float speed   = 0.0f; // Speed of the robot at the point reached so far

    // Emits the pending straight, ending at `exit_speed`
    auto driveStraight = [&](float exit_speed) {
        if (pending > 1.0f) {
            pushSubCommand(ACTION_MOVE_DISTANCE, pending, cruise, accel, true, speed, exit_speed);
            time_s += moveTime(pending, cruise, accel, speed, exit_speed);
        }
        pending = 0.0f;
        speed = exit_speed;
    };
    auto driveCurve = [&](MotionAction action, float length, float v) {
        pushSubCommand(action, length, v, accel, false, v, v);
        time_s += length / v;
        speed = v;
    };
    auto spinOnSpot = [&](MotionAction action, float degrees) {
        pushSubCommand(action, degrees, spin_speed, spin_accel, false);
        time_s += degrees / spin_speed + spin_speed / spin_accel;
    };

    // Face the first step (normally it already does)
    int8_t first_turn = ((int8_t)dirs[0] - (int8_t)pose_.current_dir + 4) % 4;
    if (first_turn == 1) spinOnSpot(ACTION_TURN_RIGHT_90, 90.0f);
    if (first_turn == 3) spinOnSpot(ACTION_TURN_LEFT_90, 90.0f);
    if (first_turn == 2) spinOnSpot(ACTION_TURN_AROUND_180, 180.0f);

    pending = MAZE_CELL_SIZE_MM; // The first step, start cell centre to the next cell centre
    uint8_t i = 1;
    while (i < steps) {
        const int8_t turn = turnAt(i);

        if (turn == 0) { // Straight on through cell i
            pending += MAZE_CELL_SIZE_MM;
            i++;
            continue;
        }
        if (turn == 2) { // The path reverses: stop at the cell centre and turn around
            driveStraight(0.0f);
            spinOnSpot(ACTION_TURN_AROUND_180, 180.0f);
            pending = MAZE_CELL_SIZE_MM;
            i++;
            continue;
        }

        // Cells i..j: a run of left / right turns with no straight cell in between
        uint8_t j = i;
        bool alternates = false;
        while (j + 1 < steps && (turnAt(j + 1) == 1 || turnAt(j + 1) == 3)) {
            if (turnAt(j + 1) != turnAt(j)) alternates = true;
            j++;
        }

        if (!use_diagonals || !alternates) {
            // Smooth 90° curves, one per cell, each from the cell's entry edge to its exit edge
            for (uint8_t k = i; k <= j; ++k) {
                pending -= HALF_CELL_SIZE_MM;
                driveStraight(turn_speed);
                driveCurve(turnAt(k) == 1 ? ACTION_CURVE_RIGHT_90 : ACTION_CURVE_LEFT_90, CURVE_90_LENGTH_MM, turn_speed);
                pending = HALF_CELL_SIZE_MM; // From the exit edge to the next cell centre
            }
            i = j + 1;
            continue;
        }

        // Diagonal: leave the cell centreline with a 45° curve...
        pending += DIAG_LEAD_MM - MAZE_CELL_SIZE_MM;
        driveStraight(turn_speed);
        driveCurve(turnAt(i) == 1 ? ACTION_CURVE_RIGHT_45 : ACTION_CURVE_LEFT_45, CURVE_45_LENGTH_MM, turn_speed);

        // ...then one diagonal straight per stretch of alternating turns, with a V turn wherever
        // two turns in a row go the same way...
        float trim_in = DIAG_TRIM_MM;
        uint8_t k = i;
        while (k <= j) {
            uint8_t m = k;
            while (m < j && turnAt(m + 1) != turnAt(m)) m++;

            const bool last_stretch = (m == j);
            const float trim_out   = last_stretch ? DIAG_TRIM_MM : V90_TRIM_MM;
            const float exit_speed = last_stretch ? turn_speed : v_speed;
            const float length     = (float)(m - k + 1) * DIAG_HALF_STEP_MM - trim_in - trim_out;

            pushSubCommand(ACTION_MOVE_DIAGONAL_HALF, length / DIAG_HALF_STEP_MM, diag_speed, accel, false, speed, exit_speed);
            time_s += moveTime(length, diag_speed, accel, speed, exit_speed);
            speed = exit_speed;

            if (!last_stretch) {
                driveCurve(turnAt(m) == 1 ? ACTION_CURVE_RIGHT_90 : ACTION_CURVE_LEFT_90, CURVE_V90_LENGTH_MM, v_speed);
                trim_in = V90_TRIM_MM;
            }
            k = m + 1;
        }

        // ...and back onto a cell centreline with another 45° curve
        driveCurve(turnAt(j) == 1 ? ACTION_CURVE_RIGHT_45 : ACTION_CURVE_LEFT_45, CURVE_45_LENGTH_MM, turn_speed);
        pending = DIAG_LEAD_MM; // From the end of that curve to the next cell centre
        i = j + 1;
    }

    driveStraight(0.0f); // Into the last cell, and stop

    pose_.cell_x = path[steps].x;
    pose_.cell_y = path[steps].y;
    pose_.current_dir = dirs[steps - 1];

    return sub_cmd_overflow_ ? -1.0f : time_s;
}

void Navigator::startSpeedRun(SpeedrunStrategy strategy) {
    current_strategy_ = strategy;
    state_ = NAV_STATE_SPEED_RUNNING;
    waiting_for_motion_ = false;

    static Coordinate path[256];
    uint8_t path_len = dijkstra_.findFastestPathToCenter(0, 0, DIR_NORTH, path, 255);
    if (path_len < 2) {
        Serial.println("[NAV] Error: No speedrun path found!");
        state_ = NAV_STATE_ERROR;
        return;
    }

    bool use_diagonals = (strategy != SPEEDRUN_CURVES_ONLY);
    if (strategy == SPEEDRUN_HYBRID_AUTO) {
        // Plan it both ways and keep whichever is quicker
        float t_curves = planSpeedRun(path, path_len, false);
        float t_diags  = planSpeedRun(path, path_len, true);
        use_diagonals = (t_diags >= 0.0f) && (t_curves < 0.0f || t_diags < t_curves);
        Serial.printf("[NAV] Hybrid: curves %.2f s, diagonals %.2f s -> %s\n",
                      t_curves, t_diags, use_diagonals ? "diagonals" : "curves");
    }

    float run_time = planSpeedRun(path, path_len, use_diagonals);
    if (run_time < 0.0f) {
        Serial.println("[NAV] Error: speed run has too many moves to plan!");
        sub_cmd_count_ = 0;
        state_ = NAV_STATE_ERROR;
        return;
    }
    Serial.printf("[NAV] Speed run planned: %d cells, %d moves, about %.2f s.\n",
                  (int)path_len, (int)sub_cmd_count_, run_time);

    processSubcommandQueue();
}

bool Navigator::startPathTest(const Coordinate* path, uint8_t path_len, bool use_diagonals) {
    if (path_len < 2) return false;
    current_strategy_ = use_diagonals ? SPEEDRUN_DIAGONALS_ONLY : SPEEDRUN_CURVES_ONLY;
    waiting_for_motion_ = false;

    float run_time = planSpeedRun(path, path_len, use_diagonals);
    if (run_time < 0.0f || sub_cmd_count_ == 0) {
        sub_cmd_count_ = 0;
        return false;
    }
    Serial.printf("[NAV] Turn test planned: %d cells, %d moves, about %.2f s.\n",
                  (int)path_len, (int)sub_cmd_count_, run_time);
    state_ = NAV_STATE_SPEED_RUNNING;
    processSubcommandQueue();
    return true;
}
