#include "navigator.h"

Navigator::Navigator(QueueHandle_t motion_cmd_queue, QueueHandle_t telemetry_queue)
    : motion_cmd_queue_(motion_cmd_queue),
      telemetry_queue_(telemetry_queue),
      floodfill_(maze_),
      dijkstra_(maze_),
      state_(NAV_STATE_IDLE),
      current_strategy_(SPEEDRUN_HYBRID_AUTO),
      waiting_for_motion_(false),
      current_search_speed_(0.0f),
      sub_cmd_count_(0),
      sub_cmd_idx_(0),
      segment_count_(0),
      segment_idx_(0) {
    memset(&pose_, 0, sizeof(pose_));
    pose_.cell_x = 0;
    pose_.cell_y = 0;
    pose_.current_dir = DIR_NORTH;
}

void Navigator::begin() {
    maze_.reset();
    if (maze_.loadFromNVS()) {
        Serial.println("[NAV] Found previously saved maze in Flash! Loaded successfully for instant Speedrun.");
    }
    floodfill_.setGoalToCenter();
    floodfill_.recalculate();
    state_ = NAV_STATE_IDLE;
    waiting_for_motion_ = false;
    current_search_speed_ = 0.0f;
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;
    segment_count_ = 0;
    segment_idx_ = 0;
}

void Navigator::startSearchRun() {
    maze_.reset();
    maze_.setVisited(0, 0); // Start cell (0,0) is visited
    floodfill_.setGoalToCenter();
    floodfill_.recalculate();
    pose_.cell_x = 0;
    pose_.cell_y = 0;
    pose_.current_dir = DIR_NORTH;
    state_ = NAV_STATE_EXPLORING_TO_CENTER;
    waiting_for_motion_ = false;
    current_search_speed_ = 0.0f;
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;
    segment_count_ = 0;
    segment_idx_ = 0;
    Serial.println("[NAV] Starting Continuous High-Speed Flying Search to Maze Center!");
}

void Navigator::startReturnRun() {
    state_ = NAV_STATE_RETURNING_TO_START;
    waiting_for_motion_ = false;
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;
    Serial.printf("[NAV] Center reached at (%d, %d)! Planning FASTEST DIAGONAL RETURN to (0,0)...\n",
                  pose_.cell_x, pose_.cell_y);

    // Auto-save discovered maze grid to NVS Flash immediately upon reaching the Goal!
    maze_.saveToNVS();
    Serial.println("[NAV] Maze successfully auto-saved to Flash NVS at Goal!");

    static Coordinate path[256];
    uint8_t path_len = dijkstra_.findFastestPathToStart(pose_.cell_x, pose_.cell_y, pose_.current_dir, path, 255);

    if (path_len < 2) {
        // Fallback to floodfill if Dijkstra has no visited path
        floodfill_.setGoalToStart();
        floodfill_.recalculate();
        path[0] = { pose_.cell_x, pose_.cell_y };
        path_len = 1;
        int8_t cur_x = pose_.cell_x;
        int8_t cur_y = pose_.cell_y;
        Direction cur_h = pose_.current_dir;
        for (int step = 0; step < 254; ++step) {
            Direction nd = floodfill_.getNextDirection(cur_x, cur_y, cur_h);
            if (nd == DIR_INVALID) break;
            cur_h = nd;
            if (nd == DIR_NORTH) cur_y++;
            else if (nd == DIR_EAST) cur_x++;
            else if (nd == DIR_SOUTH) cur_y--;
            else if (nd == DIR_WEST) cur_x--;
            path[path_len++] = { cur_x, cur_y };
            if (cur_x == 0 && cur_y == 0) break;
        }
    }

    segment_count_ = Decomposer::decompose(path, path_len, segment_queue_, MAX_SEGMENTS);
    segment_idx_ = 0;

    Serial.printf("[NAV] Return Route: %d cells decomposed into %d high-speed segments.\n",
                  path_len, segment_count_);

    if (segment_count_ > 0) {
        queueSegment(segment_queue_[segment_idx_++], RETURN_CRUISE_SPEED_MM_S, RETURN_ACCEL_MM_S2);
    }
}

void Navigator::startSpeedRun(SpeedrunStrategy strategy) {
    current_strategy_ = strategy;
    state_ = NAV_STATE_SPEED_RUNNING;
    waiting_for_motion_ = false;
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;

    static Coordinate path[256];
    uint8_t path_len = dijkstra_.findFastestPathToCenter(pose_.cell_x, pose_.cell_y, pose_.current_dir, path, 255);

    if (path_len < 2) {
        Serial.println("[NAV] Error: No speedrun path found!");
        state_ = NAV_STATE_ERROR;
        return;
    }

    if (strategy == SPEEDRUN_HYBRID_AUTO) {
        Serial.println("\n[NAV-OPTIMIZER] ⚡ Benchmarking Continuous Curves vs Diagonal Sprints...");
        static PathSegment segs_curves[MAX_SEGMENTS];
        static PathSegment segs_diags[MAX_SEGMENTS];

        uint8_t count_curves = Decomposer::decompose(path, path_len, segs_curves, MAX_SEGMENTS, false);
        uint8_t count_diags  = Decomposer::decompose(path, path_len, segs_diags, MAX_SEGMENTS, true);

        // Estimate total traversal time for Curves
        float t_curves = 0.0f;
        for (uint8_t i = 0; i < count_curves; ++i) {
            float dist = segs_curves[i].count * 180.0f;
            t_curves += (dist / SPEEDRUN_CRUISE_SPEED_MM_S) + 0.18f; // straight cruise + 90 deg curve
        }

        // Estimate total traversal time for Diagonals
        float t_diags = 0.0f;
        for (uint8_t i = 0; i < count_diags; ++i) {
            if (segs_diags[i].type == SEG_DIAGONAL) {
                float dist = 180.0f + (segs_diags[i].count - 1) * 127.28f;
                t_diags += (dist / SPEEDRUN_DIAG_SPEED_MM_S) + 0.22f; // diagonal sprint + 45 deg curves
            } else if (segs_diags[i].type == SEG_SLALOM) {
                float dist = 180.0f + segs_diags[i].count * 127.28f;
                t_diags += (dist / (SPEEDRUN_DIAG_SPEED_MM_S * 0.9f)) + 0.35f;
            } else {
                float dist = segs_diags[i].count * 180.0f;
                t_diags += (dist / SPEEDRUN_CRUISE_SPEED_MM_S) + 0.18f;
            }
        }

        Serial.printf("  • Continuous Curves Estimate: %5.2fs (%d segments)\n", t_curves, count_curves);
        Serial.printf("  • Diagonal Sprints Estimate:  %5.2fs (%d segments)\n", t_diags, count_diags);

        if (t_diags < t_curves) {
            Serial.printf("  ⚡ AUTO-SELECTED: DIAGONALS (saves %5.2fs, %.1f%% faster)!\n",
                          t_curves - t_diags, (t_curves - t_diags) / t_curves * 100.0f);
            segment_count_ = count_diags;
            for (uint8_t i = 0; i < count_diags; ++i) segment_queue_[i] = segs_diags[i];
        } else {
            Serial.printf("  ⚡ AUTO-SELECTED: CONTINUOUS CURVES (saves %5.2fs)!\n", t_diags - t_curves);
            segment_count_ = count_curves;
            for (uint8_t i = 0; i < count_curves; ++i) segment_queue_[i] = segs_curves[i];
        }

    } else if (strategy == SPEEDRUN_DIAGONALS_ONLY) {
        Serial.println("\n[NAV] 📐 Launching PURE DIAGONAL SPECIALIST Speedrun (Max Sprint Speed)!");
        segment_count_ = Decomposer::decompose(path, path_len, segment_queue_, MAX_SEGMENTS, true);
    } else { // SPEEDRUN_CURVES_ONLY
        Serial.println("\n[NAV] 🏎 Launching PURE CONTINUOUS CURVES Speedrun (Zero Diagonals)!");
        segment_count_ = Decomposer::decompose(path, path_len, segment_queue_, MAX_SEGMENTS, false);
    }

    segment_idx_ = 0;
    Serial.printf("[NAV] Championship Path: %d cells, %d high-speed segments.\n",
                  path_len, segment_count_);

    if (segment_count_ > 0) {
        queueSegment(segment_queue_[segment_idx_++], SPEEDRUN_CRUISE_SPEED_MM_S, SPEEDRUN_ACCEL_MM_S2);
    }
}

void Navigator::stop() {
    state_ = NAV_STATE_IDLE;
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;
    segment_count_ = 0;
    segment_idx_ = 0;
    sendMotionCommand(ACTION_EMERGENCY_STOP, 0.0f, 0.0f, 0.0f);
}

void Navigator::notifyMotionComplete() {
    waiting_for_motion_ = false;
    processSubcommandQueue();
}

void Navigator::sendMotionCommand(MotionAction action, float param, float max_speed, float accel,
                                  bool wall_centering, float entry_speed, float exit_speed) {
    MotionCommand cmd;
    cmd.action = action;
    cmd.param_value = param;
    cmd.max_speed_mm_s = max_speed;
    cmd.acceleration = accel;
    cmd.enable_wall_centering = wall_centering;
    cmd.entry_speed_mm_s = entry_speed;
    cmd.exit_speed_mm_s = exit_speed;

    xQueueSend(motion_cmd_queue_, &cmd, portMAX_DELAY);
    waiting_for_motion_ = true;
}

void Navigator::processSubcommandQueue() {
    if (sub_cmd_idx_ < sub_cmd_count_) {
        // Send next sub-command in current segment
        MotionCommand next_cmd = sub_cmd_queue_[sub_cmd_idx_++];
        xQueueSend(motion_cmd_queue_, &next_cmd, portMAX_DELAY);
        waiting_for_motion_ = true;
        return;
    }

    // Current segment finished! Check if more segments are queued
    if (segment_idx_ < segment_count_) {
        float speed = (state_ == NAV_STATE_SPEED_RUNNING) ? SPEEDRUN_CRUISE_SPEED_MM_S : RETURN_CRUISE_SPEED_MM_S;
        float accel = (state_ == NAV_STATE_SPEED_RUNNING) ? SPEEDRUN_ACCEL_MM_S2 : RETURN_ACCEL_MM_S2;
        queueSegment(segment_queue_[segment_idx_++], speed, accel);
        return;
    }

    // All segments finished in Return or Speedrun!
    if (state_ == NAV_STATE_RETURNING_TO_START) {
        Serial.println("[NAV] 🚀 START REACHED at (0,0)! Maze fully explored and saved.");
        maze_.saveToNVS();
        state_ = NAV_STATE_PREPARING_SPEED_RUN;
        Serial.println("[NAV] Staged in Start Cell. Press CONFIRM button to execute Speedrun!");
    } else if (state_ == NAV_STATE_SPEED_RUNNING) {
        Serial.println("[NAV] 🏆 CHAMPIONSHIP SPEED RUN COMPLETE! Center reached at maximum velocity!");
        state_ = NAV_STATE_FINISHED;
    }
}

void Navigator::queueSegment(const PathSegment& seg, float cruise_speed, float accel) {
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;

    const float turn_speed = (state_ == NAV_STATE_SPEED_RUNNING) ? SPEEDRUN_TURN_SPEED_DEG_S : SEARCH_TURN_SPEED_DEG_S;
    const float turn_accel = (state_ == NAV_STATE_SPEED_RUNNING) ? SPEEDRUN_TURN_ACCEL_DEG_S2 : SEARCH_TURN_ACCEL_DEG_S2;
    const float diag_speed = (state_ == NAV_STATE_SPEED_RUNNING) ? SPEEDRUN_DIAG_SPEED_MM_S : (cruise_speed * 1.10f);

    if (seg.type == SEG_STRAIGHT) {
        // 1. Turn to sprint heading if needed
        if (pose_.current_dir != seg.dir) {
            int8_t diff = ((int8_t)seg.dir - (int8_t)pose_.current_dir + 4) % 4;
            if (current_strategy_ == SPEEDRUN_CURVES_ONLY && state_ == NAV_STATE_SPEED_RUNNING) {
                // High-speed continuous tangent circular arcs
                if (diff == 1) {
                    sub_cmd_queue_[sub_cmd_count_++] = { ACTION_CURVE_RIGHT_90, 80.0f, cruise_speed, accel, false };
                } else if (diff == 3) {
                    sub_cmd_queue_[sub_cmd_count_++] = { ACTION_CURVE_LEFT_90, 80.0f, cruise_speed, accel, false };
                } else if (diff == 2) {
                    sub_cmd_queue_[sub_cmd_count_++] = { ACTION_TURN_AROUND_180, 180.0f, turn_speed, turn_accel, false };
                }
            } else {
                if (diff == 1) {
                    sub_cmd_queue_[sub_cmd_count_++] = { ACTION_TURN_RIGHT_90, 90.0f, turn_speed, turn_accel, false };
                } else if (diff == 3) {
                    sub_cmd_queue_[sub_cmd_count_++] = { ACTION_TURN_LEFT_90, 90.0f, turn_speed, turn_accel, false };
                } else if (diff == 2) {
                    sub_cmd_queue_[sub_cmd_count_++] = { ACTION_TURN_AROUND_180, 180.0f, turn_speed, turn_accel, false };
                }
            }
            pose_.current_dir = seg.dir;
        }

        // 2. High-speed straight sprint
        sub_cmd_queue_[sub_cmd_count_++] = { ACTION_MOVE_FORWARD_CELLS, (float)seg.count, cruise_speed, accel, true };
        pose_.cell_x = seg.end_x;
        pose_.cell_y = seg.end_y;

    } else if (seg.type == SEG_DIAGONAL) {
        Direction d1 = seg.dir;
        Direction d2 = seg.secondary_dir;
        Direction d_last = seg.last_dir;

        // Turn in place to d1 if not facing it
        if (pose_.current_dir != d1) {
            int8_t diff = ((int8_t)d1 - (int8_t)pose_.current_dir + 4) % 4;
            if (diff == 1) {
                sub_cmd_queue_[sub_cmd_count_++] = { ACTION_TURN_RIGHT_90, 90.0f, turn_speed, turn_accel, false };
            } else if (diff == 3) {
                sub_cmd_queue_[sub_cmd_count_++] = { ACTION_TURN_LEFT_90, 90.0f, turn_speed, turn_accel, false };
            } else if (diff == 2) {
                sub_cmd_queue_[sub_cmd_count_++] = { ACTION_TURN_AROUND_180, 180.0f, turn_speed, turn_accel, false };
            }
            pose_.current_dir = d1;
        }

        int8_t turn_in_diff = ((int8_t)d2 - (int8_t)d1 + 4) % 4;
        uint8_t diag_half_steps = seg.count - 1;

        // Determine exit turn direction
        Direction d_prev = (seg.count % 2 == 1) ? d2 : d1;
        int8_t turn_out_diff = ((int8_t)d_last - (int8_t)d_prev + 4) % 4;

        // Approach cell edge (90mm)
        sub_cmd_queue_[sub_cmd_count_++] = { ACTION_MOVE_HALF_CELL, 1.0f, cruise_speed, accel, true };

        // Turn 45° into diagonal
        sub_cmd_queue_[sub_cmd_count_++] = {
            (turn_in_diff == 1) ? ACTION_TURN_RIGHT_45 : ACTION_TURN_LEFT_45,
            45.0f, turn_speed, turn_accel, false
        };

        // Big continuous diagonal sprint (K * 127.28mm)
        sub_cmd_queue_[sub_cmd_count_++] = { ACTION_MOVE_DIAGONAL_HALF, (float)diag_half_steps, diag_speed, accel, false };

        // Turn 45° to align with exit direction
        sub_cmd_queue_[sub_cmd_count_++] = {
            (turn_out_diff == 1) ? ACTION_TURN_RIGHT_45 : ACTION_TURN_LEFT_45,
            45.0f, turn_speed, turn_accel, false
        };

        // Move into destination cell center (90mm)
        sub_cmd_queue_[sub_cmd_count_++] = { ACTION_MOVE_HALF_CELL, 1.0f, cruise_speed, accel, true };

        pose_.cell_x = seg.end_x;
        pose_.cell_y = seg.end_y;
        pose_.current_dir = d_last;

    } else if (seg.type == SEG_SLALOM) {
        Direction d_prog = seg.dir;
        Direction d_c1 = seg.secondary_dir;
        uint8_t num_waves = (seg.count - 1) / 2;

        if (pose_.current_dir != d_prog) {
            int8_t diff = ((int8_t)d_prog - (int8_t)pose_.current_dir + 4) % 4;
            if (diff == 1) {
                sub_cmd_queue_[sub_cmd_count_++] = { ACTION_TURN_RIGHT_90, 90.0f, turn_speed, turn_accel, false };
            } else if (diff == 3) {
                sub_cmd_queue_[sub_cmd_count_++] = { ACTION_TURN_LEFT_90, 90.0f, turn_speed, turn_accel, false };
            } else if (diff == 2) {
                sub_cmd_queue_[sub_cmd_count_++] = { ACTION_TURN_AROUND_180, 180.0f, turn_speed, turn_accel, false };
            }
            pose_.current_dir = d_prog;
        }

        int8_t t_in_diff = ((int8_t)d_c1 - (int8_t)d_prog + 4) % 4;

        // 1. Approach entrance edge
        sub_cmd_queue_[sub_cmd_count_++] = { ACTION_MOVE_HALF_CELL, 1.0f, cruise_speed, accel, true };

        // 2. Turn 45° into first wave
        sub_cmd_queue_[sub_cmd_count_++] = {
            (t_in_diff == 1) ? ACTION_TURN_RIGHT_45 : ACTION_TURN_LEFT_45,
            45.0f, turn_speed, turn_accel, false
        };

        // 3. Weave through all waves
        for (uint8_t w = 0; w < num_waves; ++w) {
            sub_cmd_queue_[sub_cmd_count_++] = { ACTION_MOVE_DIAGONAL_HALF, 2.0f, diag_speed, accel, false };
            if (w < num_waves - 1) {
                bool turn_left = (t_in_diff == 1 && w % 2 == 0) || (t_in_diff == 3 && w % 2 == 1);
                sub_cmd_queue_[sub_cmd_count_++] = {
                    turn_left ? ACTION_TURN_LEFT_90 : ACTION_TURN_RIGHT_90,
                    90.0f, turn_speed, turn_accel, false
                };
            }
        }

        // 4. Align with exit heading (d_prog)
        bool exit_left = (t_in_diff == 1 && (num_waves - 1) % 2 == 0) || (t_in_diff == 3 && (num_waves - 1) % 2 == 1);
        sub_cmd_queue_[sub_cmd_count_++] = {
            exit_left ? ACTION_TURN_LEFT_45 : ACTION_TURN_RIGHT_45,
            45.0f, turn_speed, turn_accel, false
        };

        // 5. Enter destination cell center
        sub_cmd_queue_[sub_cmd_count_++] = { ACTION_MOVE_HALF_CELL, 1.0f, cruise_speed, accel, true };

        pose_.cell_x = seg.end_x;
        pose_.cell_y = seg.end_y;
        pose_.current_dir = d_prog;
    }

    // Launch first sub-command in queue
    processSubcommandQueue();
}

void Navigator::step(const IRReadings& ir) {
    if (waiting_for_motion_) {
        return;
    }

    if (state_ == NAV_STATE_EXPLORING_TO_CENTER) {
        // 1. Update walls in current cell based on reliable 90° and front sensor readings
        maze_.updateCellWalls(pose_.cell_x, pose_.cell_y, pose_.current_dir,
                              ir.wall_left, ir.wall_front, ir.wall_right);

        // 2. Center reached?
        if (floodfill_.isAtGoal(pose_.cell_x, pose_.cell_y)) {
            current_search_speed_ = 0.0f;
            startReturnRun();
            return;
        }

        // 3. Recalculate floodfill
        floodfill_.recalculate();

        // 4. Extract path ahead
        static Coordinate path[256];
        path[0] = { pose_.cell_x, pose_.cell_y };
        uint8_t path_len = 1;
        int8_t cur_x = pose_.cell_x;
        int8_t cur_y = pose_.cell_y;
        Direction cur_h = pose_.current_dir;

        for (int step_i = 0; step_i < 254; ++step_i) {
            Direction nd = floodfill_.getNextDirection(cur_x, cur_y, cur_h);
            if (nd == DIR_INVALID) break;
            cur_h = nd;
            if (nd == DIR_NORTH) cur_y++;
            else if (nd == DIR_EAST)  cur_x++;
            else if (nd == DIR_SOUTH) cur_y--;
            else if (nd == DIR_WEST)  cur_x--;
            path[path_len++] = { cur_x, cur_y };
            if (floodfill_.isAtGoal(cur_x, cur_y)) break;
        }

        if (path_len < 2) {
            Serial.println("[NAV] Error: Trapped! No valid paths.");
            state_ = NAV_STATE_ERROR;
            return;
        }

        const float search_speed = SEARCH_SPEED_DEFAULT_MM_S;
        const float search_accel = SEARCH_ACCEL_DEFAULT_MM_S2;
        const float curve_speed  = SEARCH_CURVE_SPEED_MM_S;
        const float turn_speed   = SEARCH_TURN_SPEED_DEG_S;
        const float turn_accel   = SEARCH_TURN_ACCEL_DEG_S2;

        Coordinate next_cell = path[1];
        Direction d0 = Decomposer::getDirection({ pose_.cell_x, pose_.cell_y }, next_cell);
        int8_t diff = ((int8_t)d0 - (int8_t)pose_.current_dir + 4) % 4;

        if (diff == 0) {
            // --- CONTINUOUS FLYING STRAIGHT SPRINT ---
            // Drive forward through unexplored cell at cruise speed without stopping!
            float entry_v = current_search_speed_;
            float exit_v = search_speed;
            sendMotionCommand(ACTION_MOVE_FORWARD_CELLS, 1.0f, search_speed, search_accel, true, entry_v, exit_v);
            current_search_speed_ = search_speed;
            pose_.cell_x = next_cell.x;
            pose_.cell_y = next_cell.y;

        } else if (diff == 1 || diff == 3) {
            // --- 90° CORNER TURN ---
            if (current_search_speed_ > 50.0f) {
                // Smooth continuous 90° circular arc at safe curve speed!
                MotionAction curve_act = (diff == 1) ? ACTION_CURVE_RIGHT_90 : ACTION_CURVE_LEFT_90;
                sendMotionCommand(curve_act, 80.0f, curve_speed, search_accel, false, current_search_speed_, curve_speed);
                current_search_speed_ = curve_speed;
            } else {
                // In-place turn if starting from stationary
                sendMotionCommand((diff == 1) ? ACTION_TURN_RIGHT_90 : ACTION_TURN_LEFT_90,
                                  90.0f, turn_speed, turn_accel, false, 0.0f, 0.0f);
                current_search_speed_ = 0.0f;
            }
            pose_.current_dir = d0;
            pose_.cell_x = next_cell.x;
            pose_.cell_y = next_cell.y;

        } else if (diff == 2) {
            // --- DEAD END: OPTICAL FRONT SQUARING + 180° TURNAROUND ---
            Serial.printf("[NAV] Dead end reached at (%d, %d). Squaring optically against front wall...\n",
                          pose_.cell_x, pose_.cell_y);

            sub_cmd_count_ = 0;
            sub_cmd_idx_ = 0;

            // 1. Optically square against front wall using FL and FR sensor symmetry
            sub_cmd_queue_[sub_cmd_count_++] = { ACTION_SQUARE_FRONT_OPTICAL, 0.0f, 0.0f, 0.0f, false };

            // 2. High-precision 180° turnaround from a freshly zeroed heading baseline
            sub_cmd_queue_[sub_cmd_count_++] = { ACTION_TURN_AROUND_180, 180.0f, turn_speed, turn_accel, false };

            processSubcommandQueue();
            pose_.current_dir = d0;
            current_search_speed_ = 0.0f;
        }
    }
}

NavState Navigator::getState() const {
    return state_;
}

RobotPose Navigator::getPose() const {
    return pose_;
}

const Maze& Navigator::getMaze() const {
    return maze_;
}

Maze& Navigator::getMaze() {
    return maze_;
}

void Navigator::clearSavedMaze() {
    maze_.clearNVS();
    maze_.reset();
    floodfill_.setGoalToCenter();
    floodfill_.recalculate();
    Serial.println("[NAV] Flash NVS maze cleared and grid reset.");
}
