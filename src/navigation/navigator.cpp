#include "navigator.h"

// Comment this out to stop instead of trying an available opening when the
// current wall map contains no route to the goal.
#define ENABLE_NO_ROUTE_OPENING_RECOVERY

// Comment this out to silence per-step sensor and route-decision diagnostics.
#define DEBUG_NAV_DECISIONS

Navigator navigator;

Navigator::Navigator()
    : solver_(maze_),
      current_pos_{0, 0},
      current_heading_(DIR_NORTH),
      in_speed_run_(false) {
}

void Navigator::reset() {
    current_pos_ = {0, 0};
    current_heading_ = DIR_NORTH;
    maze_.reset();
    solver_.setGoalToCenter();
    solver_.recalculate();
    motion.setTargetHeading(0.0f);
}

void Navigator::applyManualTurn(int8_t quarter_turns) {
    if (quarter_turns > 0) {
        for (int8_t i = 0; i < quarter_turns; ++i) {
            current_heading_ = turnRight(current_heading_);
        }
    } else {
        for (int8_t i = 0; i > quarter_turns; --i) {
            current_heading_ = turnLeft(current_heading_);
        }
    }
}

bool Navigator::advanceManualCell() {
    int8_t next_x = current_pos_.x + dxFromDir(current_heading_);
    int8_t next_y = current_pos_.y + dyFromDir(current_heading_);
    if (!Maze::isValidCoordinate(next_x, next_y)) {
        return false;
    }
    current_pos_ = {next_x, next_y};
    return true;
}

void Navigator::scanAndBuildCellManual() {
    // 1. Refresh IR sensor readings; heading is controlled only by serial turn commands.
    ir_sensors.update();

    // 3. Map relative walls to absolute compass directions based on current_heading_
    Direction front_dir = current_heading_;
    Direction left_dir  = turnLeft(current_heading_);
    Direction right_dir = turnRight(current_heading_);

    bool front_wall = ir_sensors.hasFrontWall();
    bool left_wall  = ir_sensors.hasLeftWall();
    bool right_wall = ir_sensors.hasRightWall();

    // 4. Update the maze structure at the current coordinates
    maze_.setWall(current_pos_.x, current_pos_.y, front_dir, front_wall);
    maze_.setWall(current_pos_.x, current_pos_.y, left_dir,  left_wall);
    maze_.setWall(current_pos_.x, current_pos_.y, right_dir, right_wall);
    maze_.setVisited(current_pos_.x, current_pos_.y, true);

    Serial.printf("[MANUAL DEBUG] Cell (%d, %d), facing %d | Walls -> F: %s | L: %s | R: %s\n",
                  current_pos_.x, current_pos_.y, current_heading_,
                  front_wall ? "WALL" : "OPEN",
                  left_wall  ? "WALL" : "OPEN",
                  right_wall ? "WALL" : "OPEN");
}

void Navigator::scanCurrentCell() {
    // Determine absolute wall directions from robot's current heading
    Direction front_dir = current_heading_;
    Direction left_dir  = turnLeft(current_heading_);
    Direction right_dir = turnRight(current_heading_);

    bool front_wall = ir_sensors.hasFrontWall();
    bool left_wall  = ir_sensors.hasLeftWall();
    bool right_wall = ir_sensors.hasRightWall();
    DistanceSensors readings = ir_sensors.getReadings();

    maze_.setWall(current_pos_.x, current_pos_.y, front_dir, front_wall);
    maze_.setWall(current_pos_.x, current_pos_.y, left_dir,  left_wall);
    maze_.setWall(current_pos_.x, current_pos_.y, right_dir, right_wall);

    maze_.setVisited(current_pos_.x, current_pos_.y, true);

    Serial.printf("[NAV] Cell (%d,%d) Walls -> F(%s):%d, L(%s):%d, R(%s):%d | raw F=%u FL=%u RL=%u FR=%u RR=%u | mm F=%.1f FL=%.1f RL=%.1f FR=%.1f RR=%.1f | align=%+.1fdeg\n",
                  current_pos_.x, current_pos_.y,
                  front_wall ? "YES" : "NO", front_dir,
                  left_wall  ? "YES" : "NO", left_dir,
                  right_wall ? "YES" : "NO", right_dir,
                  readings.front, readings.front_left, readings.rear_left,
                  readings.front_right, readings.rear_right,
                  readings.front_mm, readings.front_left_mm, readings.rear_left_mm,
                  readings.front_right_mm, readings.rear_right_mm,
                  readings.wall_alignment_error_deg);
}

Navigator::StepResult Navigator::stepExplore() {
    // 1. Scan and register walls at current cell
    scanCurrentCell();

    // 2. Recalculate floodfill distance map
    solver_.recalculate();

#ifdef DEBUG_NAV_DECISIONS
    DistanceSensors readings = ir_sensors.getReadings();
    Serial.printf("[NAV DEBUG] Front: raw=%u (wall if >=%u), distance=%.1fmm, sensor=%s; mapped=%s\n",
                  readings.front,
                  static_cast<unsigned int>(IR_WALL_DETECT_FRONT),
                  readings.front_mm,
                  ir_sensors.hasFrontWall() ? "WALL" : "OPEN",
                  maze_.hasWall(current_pos_.x, current_pos_.y, current_heading_) ? "WALL" : "OPEN");

    const int8_t debug_turns[4] = {0, 1, -1, 2};
    const char* debug_names[4] = {"straight", "right", "left", "reverse"};
    for (int i = 0; i < 4; ++i) {
        Direction candidate = relativeToAbsolute(current_heading_, debug_turns[i]);
        int8_t nx = current_pos_.x + dxFromDir(candidate);
        int8_t ny = current_pos_.y + dyFromDir(candidate);
        bool in_bounds = Maze::isValidCoordinate(nx, ny);
        bool mapped_wall = maze_.hasWall(current_pos_.x, current_pos_.y, candidate);
        uint16_t distance = in_bounds ? solver_.getDistance(nx, ny) : DIST_INFINITY;
        Serial.printf("[NAV DEBUG] Candidate %s dir=%d: map=%s, neighbor=(%d,%d) %s, flood-distance=%u\n",
                      debug_names[i], candidate, mapped_wall ? "WALL" : "OPEN",
                      nx, ny, in_bounds ? "inside" : "outside",
                      static_cast<unsigned int>(distance));
    }
#endif

    // 3. Check if we have arrived at the goal
    if (solver_.isAtGoal(current_pos_.x, current_pos_.y)) {
        Serial.printf("[NAV] Arrived at Goal Cell (%d, %d)!\n", current_pos_.x, current_pos_.y);
        return StepResult::GoalReached;
    }

    // 4. Flood-fill already prefers straight movement when it has the lowest
    // cost.  Do not override it in open cells, or the mouse can drive away
    // from a shorter branch simply because it happened to enter heading north.
    Direction next_dir = DIR_INVALID;
    const char* decision_source = "flood-fill";
    if (next_dir == DIR_INVALID) {
        next_dir = solver_.getNextDirection(current_pos_.x, current_pos_.y, current_heading_);
    }

    if (next_dir == DIR_INVALID) {
#ifdef ENABLE_NO_ROUTE_OPENING_RECOVERY
        const int8_t turn_preference[4] = {0, 1, -1, 2};
        for (uint8_t pass = 0; pass < 2 && next_dir == DIR_INVALID; ++pass) {
            for (int i = 0; i < 4; ++i) {
                Direction candidate = relativeToAbsolute(current_heading_, turn_preference[i]);
                int8_t nx = current_pos_.x + dxFromDir(candidate);
                int8_t ny = current_pos_.y + dyFromDir(candidate);

                if (!maze_.hasWall(current_pos_.x, current_pos_.y, candidate) &&
                    Maze::isValidCoordinate(nx, ny) &&
                    (pass != 0 || !maze_.isVisited(nx, ny))) {
                    next_dir = candidate;
                    break;
                }
            }
        }

        if (next_dir != DIR_INVALID) {
            decision_source = "open-opening recovery";
            Serial.printf("[NAV] No route to goal in current map; recovery trying open %s opening.\n",
                          maze_.isVisited(current_pos_.x + dxFromDir(next_dir),
                                          current_pos_.y + dyFromDir(next_dir))
                              ? "previously visited"
                              : "unvisited");
        } else
#endif
        {
#ifdef DEBUG_NAV_DECISIONS
            Serial.println("[NAV DEBUG] Decision: STOP; no mapped opening has a reachable flood-fill distance.");
#endif
            Serial.printf("[NAV] No route to goal in current wall map from cell (%d,%d), heading=%d, walls=0x%02X, distance=%u\n",
                          current_pos_.x, current_pos_.y, current_heading_,
                          static_cast<unsigned int>(maze_.getCellRaw(current_pos_.x, current_pos_.y) & WALL_MASK),
                          static_cast<unsigned int>(solver_.getDistance(current_pos_.x, current_pos_.y)));
            motion.emergencyStop();
            return StepResult::NoRoute;
        }
    }

#ifdef DEBUG_NAV_DECISIONS
    Serial.printf("[NAV DEBUG] Decision: move %s (absolute dir=%d), source=%s\n",
                  next_dir == current_heading_ ? "straight" : "turn then forward",
                  next_dir, decision_source);
#endif

    // 5. Determine relative turn needed
    int8_t turn_code = (next_dir - current_heading_ + 4) & 0x03;

    if (turn_code == 1) {
        // 90° Turn Right
        if (!motion.turnInPlace(-90.0f, TURN_SPEED_DEG_S)) return StepResult::MotionFailed;
    } else if (turn_code == 3) {
        // 90° Turn Left
        if (!motion.turnInPlace(90.0f, TURN_SPEED_DEG_S)) return StepResult::MotionFailed;
    } else if (turn_code == 2) {
        // 180° Turn Around
        if (!motion.turnInPlace(180.0f, TURN_SPEED_DEG_S)) return StepResult::MotionFailed;
    }

    current_heading_ = next_dir;

    // 6. Move forward 1 cell
    if (!motion.moveForward(CELL_DIMENSION_MM, SEARCH_SPEED_MM_S, 0.0f, true)) {
        Serial.println("[NAV] Forward move did not complete; navigation position was not advanced.");
        motion.emergencyStop();
        return StepResult::MotionFailed;
    }

    // Update coordinates
    current_pos_.x += dxFromDir(current_heading_);
    current_pos_.y += dyFromDir(current_heading_);

    return StepResult::Continue;
}

bool Navigator::exploreToCenter() {
    Serial.println("\n[NAV] =======================================");
    Serial.println("[NAV] Starting Center Exploration Run...");
    Serial.println("[NAV] =======================================");

    solver_.setGoalToCenter();
    solver_.recalculate();

    while (!solver_.isAtGoal(current_pos_.x, current_pos_.y)) {
        StepResult result = stepExplore();
        if (result == StepResult::GoalReached) break;
        if (result != StepResult::Continue) return false;
    }

    // Goal reached! Scan the center cell walls
    scanCurrentCell();
    maze_.printAscii();

    // Success indicator: blink LED rapidly
    for (int i = 0; i < 6; ++i) {
        digitalWrite(PIN_STATUS_LED, HIGH); delay(100);
        digitalWrite(PIN_STATUS_LED, LOW);  delay(100);
    }

    return true;
}

bool Navigator::exploreOneCell() {
    solver_.setGoalToCenter();
    StepResult result = stepExplore();
    if (result == StepResult::Continue) {
        Serial.printf("[NAV] Step complete; now at cell (%d,%d), heading=%d. Waiting for next step command.\n",
                      current_pos_.x, current_pos_.y, current_heading_);
        return true;
    }
    if (result == StepResult::GoalReached) {
        Serial.println("[NAV] Already at the center goal; no cell movement made.");
        return true;
    }
    return false;
}

bool Navigator::exploreToStart() {
    Serial.println("\n[NAV] Starting Return Journey to Start (0, 0)...");

    solver_.setGoalToStart();
    solver_.recalculate();

    while (!solver_.isAtGoal(current_pos_.x, current_pos_.y)) {
        StepResult result = stepExplore();
        if (result == StepResult::GoalReached) break;
        if (result != StepResult::Continue) return false;
    }

    Serial.println("[NAV] Successfully Returned to Start (0, 0)!");

    // Turn to face North in start cell
    if (current_heading_ != DIR_NORTH) {
        int8_t turn_code = (DIR_NORTH - current_heading_ + 4) & 0x03;
        bool turned = (turn_code == 1) ? motion.turnInPlace(-90.0f) :
                      (turn_code == 3) ? motion.turnInPlace(90.0f) :
                                         motion.turnInPlace(180.0f);
        if (!turned) return false;
        current_heading_ = DIR_NORTH;
    }

    return true;
}

bool Navigator::runFastSpeed() {
    Serial.println("\n[NAV] =======================================");
    Serial.println("[NAV] Generating Optimized Speed Run Path...");
    Serial.println("[NAV] =======================================");

    if (!optimizer_.generatePath(maze_, solver_, current_pos_, current_heading_)) {
        Serial.println("[NAV] Failed to generate speed run path! Please explore maze first.");
        return false;
    }

    optimizer_.printPath();

    Serial.println("\n[NAV] Executing High-Speed Speed Run in 3 seconds...");
    for (int i = 3; i > 0; --i) {
        Serial.printf("[NAV] %d...\n", i);
        digitalWrite(PIN_STATUS_LED, HIGH); delay(200);
        digitalWrite(PIN_STATUS_LED, LOW);  delay(800);
    }

    uint16_t total_segs = optimizer_.getSegmentCount();
    for (uint16_t i = 0; i < total_segs; ++i) {
        const PathSegment& seg = optimizer_.getSegment(i);

        if (seg.action == ACTION_FORWARD) {
            if (!motion.moveForward(seg.value, seg.speed_mm_s, seg.end_speed_mm_s, true)) {
                Serial.println("[NAV] Speed-run forward move failed; stopping the run.");
                motion.emergencyStop();
                return false;
            }
        } else if (seg.action == ACTION_TURN_LEFT) {
            if (!motion.turnInPlace(seg.value, TURN_SPEED_DEG_S)) return false;
        } else if (seg.action == ACTION_TURN_RIGHT) {
            if (!motion.turnInPlace(seg.value, TURN_SPEED_DEG_S)) return false;
        } else if (seg.action == ACTION_TURN_AROUND) {
            if (!motion.turnInPlace(seg.value, TURN_SPEED_DEG_S)) return false;
        } else if (seg.action == ACTION_STOP) {
            motion.emergencyStop();
            break;
        }
    }

    Serial.println("[NAV] Speed Run Completed!");
    return true;
}
