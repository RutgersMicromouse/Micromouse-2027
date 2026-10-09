#include "navigation/navigator/navigator.h"

// ==============================================================================
// NAVIGATOR: SEARCH STEPPING
// ==============================================================================

// Search run: one decision per cell, taken at the cell centre where the wall sensors are reliable.
// The same stepping drives both legs: out to the centre, then back to the start cell. Unexplored
// cells count as open on both legs, so the way back naturally explores new ground.
//
// The robot rolls straight through cells without stopping whenever the way ahead is clear:
// at full search speed through cells it already knows, at SEARCH_PROBE_SPEED into cells it has
// never seen (so it can stop within a few millimetres if a wall turns up). It stops and turns on
// the spot only where the path actually turns.
void Navigator::step(const IRReadings& ir, const WallPreview& preview) {
    if (waiting_for_motion_ ||
        (state_ != NAV_STATE_EXPLORING_TO_CENTER && state_ != NAV_STATE_RETURNING_TO_START)) {
        return;
    }

    // One move at a time: stand still at the cell centre until the next move is asked for
    if (single_step_ && !step_allowed_) {
        if (!step_paused_) {
            static const char* const kCompass[4] = { "north", "east", "south", "west" };
            Serial.printf("[NAV] Waiting in cell (%d, %d) facing %s. Walls seen here: %s%s%s. Ask for the next move.\n",
                          pose_.cell_x, pose_.cell_y, kCompass[pose_.current_dir % 4],
                          ir.wall_left ? "left " : "", ir.wall_front ? "front " : "", ir.wall_right ? "right" : "");
        }
        step_paused_ = true;
        return;
    }
    step_paused_ = false;

    // Part-way through a look-ahead move: the robot is on a cell edge, not a centre
    if (search_phase_ == PHASE_AT_EDGE)     { stepAtCellEdge(ir, preview); return; }
    if (search_phase_ == PHASE_AFTER_CURVE) { stepAfterCurve(ir, preview); return; }

    const float search_speed = SEARCH_SPEED_DEFAULT_MM_S;
    const float search_accel = SEARCH_ACCEL_DEFAULT_MM_S2;
    const float turn_speed   = SEARCH_TURN_SPEED_DEG_S;
    const float turn_accel   = SEARCH_TURN_ACCEL_DEG_S2;

    // 1. Update walls in current cell based on reliable 90° and front sensor readings
    if (!maze_.isVisited(pose_.cell_x, pose_.cell_y)) maze_changed_ = true;
    maze_.updateCellWalls(pose_.cell_x, pose_.cell_y, pose_.current_dir,
                          ir.wall_left, ir.wall_front, ir.wall_right);

    // A wall the front sensors see right now, from the cell centre, is never driven into on the
    // strength of older readings that said the way was open
    if (ir.wall_front) {
        for (int i = 0; i < 2 * WALL_VOTE_LIMIT && !maze_.hasWall(pose_.cell_x, pose_.cell_y, pose_.current_dir); ++i) {
            maze_.observeWall(pose_.cell_x, pose_.cell_y, pose_.current_dir, true);
        }
    }

    // A move that rolled in at speed comes to rest by itself if it met a wall ahead
    // (MotionCommand::stop_at_front_wall), so with a wall in front the robot is standing still.
    if (ir.wall_front) current_search_speed_ = 0.0f;

#if ENABLE_FRONT_WALL_DISTANCE_FIX
    // 1b. Standing in front of a wall: check the distance to it with the front sensors and, if the
    // robot is not in the middle of the cell, shuffle there before turning. Done once per stop;
    // step() runs again when the shuffle has finished and then carries on below.
    if (ir.wall_front && !front_fix_done_ && fabsf(ir.front_offset_mm) > FRONT_FIX_MIN_MM) {
        front_fix_done_ = true;
        const float shuffle_mm = fmaxf(-FRONT_FIX_MAX_MM, fminf(FRONT_FIX_MAX_MM, ir.front_offset_mm * FRONT_FIX_GAIN));
        Serial.printf("[NAV] Front wall check: %.0f mm %s the cell centre. Moving %.0f mm %s.\n",
                      fabsf(ir.front_offset_mm), ir.front_offset_mm > 0.0f ? "short of" : "past",
                      fabsf(shuffle_mm), shuffle_mm > 0.0f ? "forward" : "back");
        sendMotionCommand(ACTION_MOVE_DISTANCE, shuffle_mm, SEARCH_PROBE_SPEED_MM_S, search_accel, false);
        return;
    }
    front_fix_done_ = false;
#endif

    // 2. Work out which way to go next
    const bool at_goal = floodfill_.isAtGoal(pose_.cell_x, pose_.cell_y);
    static Coordinate path[256];
    uint8_t path_len = 1;
    int8_t diff = 0;
    Direction d0 = pose_.current_dir;

    if (!at_goal) {
        floodfill_.recalculate();

        path[0] = { pose_.cell_x, pose_.cell_y };
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
            // The map says the robot is walled in. Mazes are never built like that, so the map is
            // wrong: first drop the walls that rest on a single reading, then, if that is not
            // enough, forget the whole map and carry on exploring from here.
            if (trap_recovery_ < 2) {
                trap_recovery_++;
                if (trap_recovery_ == 1) {
                    Serial.println("[NAV] Map says walled in. Dropping walls seen only once and looking again.");
                    maze_.forgetWeakWalls();
                } else {
                    Serial.println("[NAV] Still walled in. Forgetting the map and exploring afresh from here.");
                    maze_.reset();
                }
                maze_.setVisited(pose_.cell_x, pose_.cell_y);
                maze_changed_ = true;
                floodfill_.recalculate();
                step(ir, preview);
                return;
            }
            Serial.println("[NAV] Error: Trapped! No valid paths.");
            state_ = NAV_STATE_ERROR;
            return;
        }
        trap_recovery_ = 0;

        d0 = directionBetween({ pose_.cell_x, pose_.cell_y }, path[1]);
        diff = ((int8_t)d0 - (int8_t)pose_.current_dir + 4) % 4;
    }

    // 3. Anything other than "straight on" happens from a standstill at the cell centre. If the
    // robot rolled in at speed, brake, then back up the few millimetres it overshot, so that
    // turning on the spot never leaves it off-centre. Then decide again.
    // (Not on reaching the centre, though: the search rolls straight on into its return leg, and
    // the recursive step() below brakes only if the way back starts with a turn.)
    const bool reached_centre = at_goal && state_ == NAV_STATE_EXPLORING_TO_CENTER;
    if ((at_goal || diff != 0) && current_search_speed_ > 0.0f && !reached_centre) {
        float v = current_search_speed_;
        float brake_mm = (v * v) / (2.0f * search_accel) + 2.0f;
        sub_cmd_count_ = 0;
        sub_cmd_idx_ = 0;
        pushSubCommand(ACTION_MOVE_DISTANCE,  brake_mm, v, search_accel, false, v, 0.0f);
        pushSubCommand(ACTION_MOVE_DISTANCE, -brake_mm, SEARCH_PROBE_SPEED_MM_S, search_accel, false);
        processSubcommandQueue();
        current_search_speed_ = 0.0f;
        return;
    }

    if (at_goal) {
        if (current_search_speed_ <= 0.0f) saveMazeIfChanged(); // Flash is only written standing still

        if (state_ == NAV_STATE_EXPLORING_TO_CENTER) {
            // Centre reached: now explore back to the start cell
            Serial.printf("[NAV] Centre reached at (%d, %d). Exploring back to the start.\n", pose_.cell_x, pose_.cell_y);
            floodfill_.setGoalToStart();
            floodfill_.recalculate();
            state_ = NAV_STATE_RETURNING_TO_START;
            step(ir, preview);
            return;
        }

        // Back in the start cell: the search is over
        floodfill_.setGoalToCenter();
        floodfill_.recalculate();
        best_route_explored_ = checkBestRouteExplored();
        state_ = NAV_STATE_PREPARING_SPEED_RUN;

        // Turn to face into the maze again, so the next run can start without picking the robot
        // up. Every run begins in the start cell facing north. (Directions count clockwise, so
        // this is the number of quarter-turns to the right that brings it back to north.)
        const uint8_t right_turns = (uint8_t)(DIR_NORTH - pose_.current_dir) & 3;
        if (right_turns == 1) sendMotionCommand(ACTION_TURN_RIGHT_90,   90.0f,  SEARCH_TURN_SPEED_DEG_S, SEARCH_TURN_ACCEL_DEG_S2, false);
        if (right_turns == 2) sendMotionCommand(ACTION_TURN_AROUND_180, 180.0f, SEARCH_TURN_SPEED_DEG_S, SEARCH_TURN_ACCEL_DEG_S2, false);
        if (right_turns == 3) sendMotionCommand(ACTION_TURN_LEFT_90,    90.0f,  SEARCH_TURN_SPEED_DEG_S, SEARCH_TURN_ACCEL_DEG_S2, false);
        pose_.current_dir = DIR_NORTH;

        Serial.println(best_route_explored_
            ? "[NAV] Search complete. The shortest possible route is fully explored: ready for speed runs."
            : "[NAV] Search complete. A shorter route may still be hiding in unexplored cells: search again to look for it.");
        return;
    }

    if (diff == 0) {
        // --- STRAIGHT ON: one cell forward, without stopping if the plan keeps going straight ---
        Coordinate next_cell = path[1];
        float exit_v = 0.0f;
        // (one move at a time: every move ends at rest in a cell centre)
        // Into the centre the plan ends, so ask where the return leg would go from there.
        const bool next_is_goal = floodfill_.isAtGoal(next_cell.x, next_cell.y);
        bool plan_continues_straight = !single_step_ &&
            (next_is_goal ? returnLegGoesStraightOn(next_cell, d0)
                          : (path_len >= 3 && directionBetween(path[1], path[2]) == d0));
        if (plan_continues_straight) {
            exit_v = maze_.isVisited(next_cell.x, next_cell.y) ? knownStraightSpeed(path, path_len, d0)
                                                               : SEARCH_PROBE_SPEED_MM_S;
        }

        // About to drive through the opening ahead, which settles that it is one
        maze_.confirmOpen(pose_.cell_x, pose_.cell_y, d0);

#if ENABLE_SEARCH_LOOKAHEAD
        // Look-ahead: if the next cell is new, or the plan turns there, drive only as far as its
        // edge. By then the 45° sensors have seen its side walls, and stepAtCellEdge() can curve
        // straight through it instead of stopping at its centre to turn on the spot.
        const bool next_is_known = maze_.isVisited(next_cell.x, next_cell.y);
        if (!single_step_ && !next_is_goal && (!next_is_known || !plan_continues_straight)) {
            // The robot may curve from that edge, so it arrives there no faster than a curve is taken
            // (a curve through a cell it has visited before may be taken a little quicker)
            float edge_v = next_is_known ? SEARCH_KNOWN_CURVE_SPEED_MM_S
                                         : fminf(SEARCH_PROBE_SPEED_MM_S, SEARCH_CURVE_SPEED_MM_S);
            sendMotionCommand(ACTION_MOVE_DISTANCE, HALF_CELL_SIZE_MM, search_speed, search_accel, true,
                              current_search_speed_, edge_v);
            current_search_speed_ = edge_v;
            pose_.cell_x = next_cell.x;
            pose_.cell_y = next_cell.y;
            search_phase_ = PHASE_AT_EDGE;
            return;
        }
#endif
        // Into a cell it has visited before, the robot may go faster than the search speed in
        // between (it still arrives at exit_v); into a new one it keeps to the search speed.
        const float cruise = maze_.isVisited(next_cell.x, next_cell.y)
                             ? fmaxf(SEARCH_KNOWN_SPEED_MM_S, search_speed) : search_speed;
        sendMotionCommand(ACTION_MOVE_FORWARD_CELLS, 1.0f, fmaxf(cruise, exit_v), search_accel, true,
                          current_search_speed_, exit_v, true);
        current_search_speed_ = exit_v;
        pose_.cell_x = next_cell.x;
        pose_.cell_y = next_cell.y;

    } else if (diff == 1 || diff == 3) {
        saveMazeIfChanged();

        // --- TURN: on the spot at the cell centre. The robot stays in this cell; the next step
        // re-reads the walls facing the new way and then drives straight on.
        sendMotionCommand((diff == 1) ? ACTION_TURN_RIGHT_90 : ACTION_TURN_LEFT_90,
                          90.0f, turn_speed, turn_accel, false);
        pose_.current_dir = d0;

    } else {
        saveMazeIfChanged();

        // --- DEAD END: 180° TURNAROUND (after squaring on the front wall, if that is switched on) ---
        Serial.printf("[NAV] Dead end reached at (%d, %d). Turning round.\n",
                      pose_.cell_x, pose_.cell_y);

        sub_cmd_count_ = 0;
        sub_cmd_idx_ = 0;

#if ENABLE_FRONT_SQUARING
        // 1. Optically square against front wall using FL and FR sensor symmetry
        pushSubCommand(ACTION_SQUARE_FRONT_OPTICAL, 0.0f, 0.0f, 0.0f, false);
#endif

        // 2. 180° turnaround (measured from the grid heading, like every turn on the spot)
        pushSubCommand(ACTION_TURN_AROUND_180, 180.0f, turn_speed, turn_accel, false);

        processSubcommandQueue();
        pose_.current_dir = d0;
    }
}

// The robot is on the entry edge of pose_.cell, still rolling. Curve through the cell if the path
// turns there and the turn is certain to be clear; otherwise carry on to its centre and decide
// there in the normal way.
void Navigator::stepAtCellEdge(const IRReadings& ir, const WallPreview& preview) {
    const int8_t x = pose_.cell_x;
    const int8_t y = pose_.cell_y;
    const Direction heading = pose_.current_dir;
    const Direction left  = Maze::getAbsoluteDirection(heading, -1);
    const Direction right = Maze::getAbsoluteDirection(heading, 1);

    const bool known = maze_.isVisited(x, y);
    bool sides_certain = known;

    if (!known) {
        // The 45° sensors look ahead: on the way here they were already reading this cell's side
        // walls, and every sample in their window had to agree for a verdict (see getWallPreview).
        // The 90° sensors look straight sideways, so at this moment they may still be beside the
        // PREVIOUS cell's wall or the post between the two cells. Asking them to agree here is
        // only right if they sit well ahead of the wheel axle (SEARCH_CONFIRM_WITH_90).
#if SEARCH_CONFIRM_WITH_90
        const bool left_wall  = preview.left_wall  && ir.wall_left;
        const bool left_open  = preview.left_open  && !ir.wall_left;
        const bool right_wall = preview.right_wall && ir.wall_right;
        const bool right_open = preview.right_open && !ir.wall_right;
#else
        const bool left_wall  = preview.left_wall,  left_open  = preview.left_open;
        const bool right_wall = preview.right_wall, right_open = preview.right_open;
#endif
        sides_certain = (left_wall || left_open) && (right_wall || right_open);

        if (sides_certain) {
            maze_.observeWall(x, y, left,  left_wall);
            maze_.observeWall(x, y, right, right_wall);
            floodfill_.recalculate();
        }
    }

    // The front wall is not known yet, so the floodfill treats it as open. If turning is still the
    // best way out of this cell, it is the best way whatever the front wall turns out to be.
    Direction best = sides_certain ? floodfill_.getNextDirection(x, y, heading) : DIR_INVALID;
    edge_note_ = !sides_certain ? "drove to the centre: the look-ahead sensors were not sure about the side walls"
               : (best == left || best == right) ? "curved through the corner without stopping"
               : "went straight on to the centre";

    // Whatever the map says, never curve toward a side unless it is positively clear: no sensor
    // that can see it reports a wall, and either the look-ahead saw the opening just now or the
    // robot has driven through it before. (An opening the map merely has no wall for is not
    // enough: that is also what a wall dropped by forgetWeakWalls() looks like.)
    auto clearToCurve = [&](Direction side, bool seen_wall, bool seen_open, bool wall_at_90) {
        if (seen_wall) return false;
#if SEARCH_CONFIRM_WITH_90
        (void)side; (void)seen_open;
        return !wall_at_90;
#else
        (void)wall_at_90;
        return seen_open || maze_.isKnownOpen(x, y, side);
#endif
    };
    if ((best == left  && !clearToCurve(left,  preview.left_wall,  preview.left_open,  ir.wall_left)) ||
        (best == right && !clearToCurve(right, preview.right_wall, preview.right_open, ir.wall_right))) {
        best = DIR_INVALID;
        edge_note_ = "drove to the centre: the side the route turns to was not positively clear";
    }

    if (best == left || best == right) {
        maze_.confirmOpen(x, y, best); // About to drive through it
        // Curves are where the robot is most likely to slip, so they are driven at the curve
        // speed from start to finish; it speeds up again on the straight that follows.
        // A cell it has visited before is curved through a little quicker.
        const float v = fminf(current_search_speed_, known ? SEARCH_KNOWN_CURVE_SPEED_MM_S : SEARCH_CURVE_SPEED_MM_S);
        curve_cell_x_ = x;
        curve_cell_y_ = y;
        curve_entry_dir_ = heading;
        curve_cell_was_known_ = known;

        sendMotionCommand((best == right) ? ACTION_CURVE_RIGHT_90 : ACTION_CURVE_LEFT_90,
                          CURVE_90_LENGTH_MM, v, SEARCH_ACCEL_DEFAULT_MM_S2, false, current_search_speed_, v);
        current_search_speed_ = v;

        // The curve ends on the entry edge of the neighbouring cell
        pose_.current_dir = best;
        if (best == DIR_NORTH) pose_.cell_y++;
        else if (best == DIR_EAST)  pose_.cell_x++;
        else if (best == DIR_SOUTH) pose_.cell_y--;
        else if (best == DIR_WEST)  pose_.cell_x--;
        search_phase_ = PHASE_AFTER_CURVE;
        return;
    }

    driveToCellCentre();
}

// The curve through (curve_cell_x_, curve_cell_y_) has finished. Half-way round it the outer 45°
// sensor was facing that cell's front wall, which completes what is known about the cell.
void Navigator::stepAfterCurve(const IRReadings& ir, const WallPreview& preview) {
    if (!curve_cell_was_known_) {
        // Only count the cell as explored if the front wall reading was clear either way;
        // otherwise it stays unexplored and the speed run will not be routed through it on trust.
        if (preview.front_wall || preview.front_open) {
            maze_.observeWall(curve_cell_x_, curve_cell_y_, curve_entry_dir_, preview.front_wall);
            maze_.setVisited(curve_cell_x_, curve_cell_y_);
            maze_changed_ = true;
        }
    }

#if ENABLE_SEARCH_CHAINED_CURVES
    // The robot is now on the entry edge of the next cell, exactly where it would be after a
    // straight, and as the curve ended its 45° sensors read that cell's side walls. So decide
    // here like at any other cell edge: if the route turns again and that side is positively
    // clear it curves again at once (two in a row the same way is a U-turn in one motion);
    // otherwise stepAtCellEdge() carries on to the cell centre as before. The goal cell is
    // always entered to its centre, as it is from a straight.
    if (!floodfill_.isAtGoal(pose_.cell_x, pose_.cell_y)) {
#if !SEARCH_CHAIN_CURVES_BY_SENSORS
        // ...but only where the map already knows the way. The side readings taken as a curve
        // ends are not good enough to curve again on: the robot is still swinging, and on the
        // real robot they called a walled side open and it curved into the wall. So a cell
        // it has never visited is driven to its centre and read there, and in a visited cell
        // an "open" from those readings counts for nothing (a "wall" still vetoes the curve).
        if (!maze_.isVisited(pose_.cell_x, pose_.cell_y)) {
            edge_note_ = "drove to the centre: new cell straight after a curve, so it is read from the centre";
            driveToCellCentre();
            return;
        }
        WallPreview walls_only = preview;
        walls_only.left_open = false;
        walls_only.right_open = false;
        stepAtCellEdge(ir, walls_only);
        return;
#endif
        stepAtCellEdge(ir, preview);
        return;
    }
#else
    (void)ir;
#endif
    driveToCellCentre();
}

// The robot is about to roll into the centre cell `centre` heading `heading`. Once there the search
// turns round and explores its way back to the start: would that begin by going straight on? If
// so there is no reason to stop in the centre at all.
bool Navigator::returnLegGoesStraightOn(Coordinate centre, Direction heading) {
    if (state_ != NAV_STATE_EXPLORING_TO_CENTER) return false; // The start cell is where the search ends
    floodfill_.setGoalToStart();
    floodfill_.recalculate();
    const Direction onward = floodfill_.getNextDirection(centre.x, centre.y, heading);
    floodfill_.setGoalToCenter();
    floodfill_.recalculate();
    return onward == heading;
}

// Dynamic search speed. The robot is about to drive one cell along `heading` into path[1], which
// it has visited, and the plan carries straight on from there. Returns the speed it may have at
// path[1]'s centre: the normal search speed, plus whatever it can still shed, braking gently,
// over the further cells of this straight that are just as well known. So it speeds up along a
// known straight and is back at the normal search speed one cell before anything new or a turn.
float Navigator::knownStraightSpeed(const Coordinate* path, uint8_t path_len, Direction heading) const {
    uint8_t clear_cells = 0;
    for (uint8_t i = 1; i + 2 < path_len; ++i) {
        if (directionBetween(path[i + 1], path[i + 2]) != heading) break;   // The route turns there
        if (!maze_.isVisited(path[i + 1].x, path[i + 1].y)) break;          // New ground
        if (!maze_.isKnownOpen(path[i].x, path[i].y, heading)) break;       // Opening never confirmed
        clear_cells++;
    }
    const float brake_accel = 0.8f * SEARCH_ACCEL_DEFAULT_MM_S2; // Leave a margin on the braking
    const float v = sqrtf(SEARCH_SPEED_DEFAULT_MM_S * SEARCH_SPEED_DEFAULT_MM_S +
                          2.0f * brake_accel * MAZE_CELL_SIZE_MM * clear_cells);
    return fminf(v, fmaxf(SEARCH_KNOWN_SPEED_MM_S, SEARCH_SPEED_DEFAULT_MM_S));
}

// Second half of a look-ahead move: from the entry edge of pose_.cell to its centre, where the
// next decision is taken with all the sensors in the normal way.
void Navigator::driveToCellCentre() {
    const Direction best = floodfill_.getNextDirection(pose_.cell_x, pose_.cell_y, pose_.current_dir);
    const bool keep_rolling = floodfill_.isAtGoal(pose_.cell_x, pose_.cell_y)
                              ? returnLegGoesStraightOn({ pose_.cell_x, pose_.cell_y }, pose_.current_dir)
                              : (best == pose_.current_dir);
    const float exit_v = keep_rolling ? SEARCH_PROBE_SPEED_MM_S : 0.0f;

    sendMotionCommand(ACTION_MOVE_DISTANCE, HALF_CELL_SIZE_MM, SEARCH_SPEED_DEFAULT_MM_S, SEARCH_ACCEL_DEFAULT_MM_S2,
                      true, current_search_speed_, exit_v, true);
    current_search_speed_ = exit_v;
    search_phase_ = PHASE_AT_CENTRE;
}

void Navigator::continueOneMove(const IRReadings& ir, const WallPreview& preview) {
    if (!step_paused_) return;
    step_allowed_ = true;
    step(ir, preview);
}
