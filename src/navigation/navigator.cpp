#include "navigator.h"

// Per-step sensor and route-decision diagnostics.
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
    // 1. Refresh IR sensor readings with flush; heading is controlled only by serial turn commands.
    ir_sensors.flushFilter(4);

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
    // Flush IR filter so readings reflect the current stationary position and heading
    ir_sensors.flushFilter(4);

    // Determine absolute wall directions from robot's current heading
    Direction front_dir = current_heading_;
    Direction left_dir  = turnLeft(current_heading_);
    Direction right_dir = turnRight(current_heading_);

    bool front_wall = ir_sensors.hasFrontWall();
    bool left_wall  = ir_sensors.hasLeftWall();
    bool right_wall = ir_sensors.hasRightWall();
    DistanceSensors readings = ir_sensors.getReadings();

    // Front-side open-space confirmation:
    // If a front-side sensor reads open space (< IR_WALL_DETECT_SIDE),
    // the forward portion of that side is unequivocally open.
    if (readings.front_left < IR_WALL_DETECT_SIDE) {
        left_wall = false;
    }
    if (readings.front_right < IR_WALL_DETECT_SIDE) {
        right_wall = false;
    }

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

    // Visual confirmation: Print evolving ASCII map and distance matrix at each step
    maze_.printAscii(current_pos_.x, current_pos_.y, current_heading_);
    solver_.printDistanceMatrix();

    // 3. Check if we have arrived at the goal
    if (solver_.isAtGoal(current_pos_.x, current_pos_.y)) {
        Serial.printf("[NAV] Arrived at Goal Cell (%d, %d)!\n", current_pos_.x, current_pos_.y);
        return StepResult::GoalReached;
    }

    // 4. Decision: strictly 100% determined by Floodfill shortest-path gradient
    uint16_t curr_dist = solver_.getDistance(current_pos_.x, current_pos_.y);
    Serial.printf("[FLOODFILL] Cell (%d,%d), Heading=%d, Dist-to-Goal=%u\n",
                  current_pos_.x, current_pos_.y, current_heading_, curr_dist);

    const int8_t debug_turns[4] = {0, 1, -1, 2};
    const char* debug_names[4] = {"STRAIGHT", "RIGHT", "LEFT", "REVERSE"};
    for (int i = 0; i < 4; ++i) {
        Direction candidate = relativeToAbsolute(current_heading_, debug_turns[i]);
        int8_t nx = current_pos_.x + dxFromDir(candidate);
        int8_t ny = current_pos_.y + dyFromDir(candidate);
        bool in_bounds = Maze::isValidCoordinate(nx, ny);
        bool mapped_wall = maze_.hasWall(current_pos_.x, current_pos_.y, candidate);
        if (mapped_wall) {
            Serial.printf("  Candidate %s (dir %d): WALL [BLOCKED]\n", debug_names[i], candidate);
        } else if (!in_bounds) {
            Serial.printf("  Candidate %s (dir %d): OUT OF BOUNDS\n", debug_names[i], candidate);
        } else {
            uint16_t d = solver_.getDistance(nx, ny);
            Serial.printf("  Candidate %s (dir %d -> %d,%d): DIST = %u%s\n",
                          debug_names[i], candidate, nx, ny, d,
                          (d == DIST_INFINITY) ? " [UNREACHABLE]" : "");
        }
    }

    Direction next_dir = solver_.getNextDirection(current_pos_.x, current_pos_.y, current_heading_);

    // Front sensor safety check: if robot intended to drive straight into an unmapped front wall,
    // update the map immediately, recalculate floodfill, and re-query
    if (next_dir == current_heading_ && ir_sensors.hasFrontWall()) {
        Serial.println("[FLOODFILL] Physical front wall confirmed ahead; updating map & re-running floodfill.");
        maze_.setWall(current_pos_.x, current_pos_.y, current_heading_, true);
        solver_.recalculate();
        next_dir = solver_.getNextDirection(current_pos_.x, current_pos_.y, current_heading_);
    }

    if (next_dir == DIR_INVALID) {
        Serial.printf("[NAV ERROR] No route to goal in current wall map from cell (%d,%d), heading=%d, distance=%u\n",
                      current_pos_.x, current_pos_.y, current_heading_, curr_dist);
        motion.emergencyStop();
        return StepResult::NoRoute;
    }

    int8_t target_nx = current_pos_.x + dxFromDir(next_dir);
    int8_t target_ny = current_pos_.y + dyFromDir(next_dir);
    uint16_t target_dist = solver_.getDistance(target_nx, target_ny);
    Serial.printf("[FLOODFILL 100%% SOLVER DECISION] Move %s (dir %d -> %d,%d, target flood-distance=%u)\n",
                  next_dir == current_heading_ ? "STRAIGHT" : "TURN",
                  next_dir, target_nx, target_ny, target_dist);


    // 5. Determine relative turn needed
    int8_t turn_code = (next_dir - current_heading_ + 4) & 0x03;

    if (turn_code != 0) {
        Serial.printf("[NAV] Executing %s (turn_code=%d) from heading %d to heading %d...\n",
                      (turn_code == 1) ? "90-deg RIGHT turn" :
                      (turn_code == 3) ? "90-deg LEFT turn" : "180-deg TURN AROUND",
                      turn_code, current_heading_, next_dir);
        if (turn_code == 1) {
            // 90° Turn Right
            motion.turnInPlace(-90.0f);
        } else if (turn_code == 3) {
            // 90° Turn Left
            motion.turnInPlace(90.0f);
        } else if (turn_code == 2) {
            // 180° Turn Around: two consecutive proven 90° right turns
            motion.turnInPlace(-90.0f);
            delay(50);
            motion.turnInPlace(-90.0f);
        }

        current_heading_ = next_dir;

        // Post-turn settling pause: brief, crisp stabilization before immediate forward motion
        delay(25);
        ir_sensors.flushFilter(4);

        // Post-turn sensor check (informational; moveForward will brake safely if an obstacle is encountered)
        if (ir_sensors.getFront() >= 500) {
            Serial.printf("[NAV] Post-turn notice: front sensor close (raw=%u, %.1f mm) facing heading %d.\n",
                          ir_sensors.getFront(), ir_sensors.getFrontMM(), current_heading_);
        }
    } else {
        current_heading_ = next_dir;
    }

    // 6. Move forward 1 cell
    if (!motion.moveForward(CELL_DIMENSION_MM, SEARCH_SPEED_MM_S, 0.0f, true)) {
        Serial.println("[NAV] Forward move did not complete; navigation position was not advanced.");
        motion.emergencyStop();
        return StepResult::MotionFailed;
    }

    // Settling pause after cell arrival: allows chassis to stop completely,
    // sensors to stabilize, and heading to align before scanning the new cell.
    delay(100);
    ir_sensors.flushFilter(8);

    // Update coordinates
    current_pos_.x += dxFromDir(current_heading_);
    current_pos_.y += dyFromDir(current_heading_);

    Serial.printf("[NAV] Arrived at Cell (%d, %d), facing %d\n",
                  current_pos_.x, current_pos_.y, current_heading_);

    return StepResult::Continue;
}

bool Navigator::exploreToCenter() {
    Serial.println("\n[NAV] =======================================");
    Serial.println("[NAV] Starting Center Exploration Run (Stage 1)...");
    Serial.println("[NAV] =======================================");

    solver_.setGoalToCenter();
    solver_.recalculate();

    // Diagnostic in-place 360-degree turn in the start square (0, 0)
    // before moving forward with the Stage 1 search algorithm.
    if (current_pos_.x == 0 && current_pos_.y == 0) {
        Serial.println("\n[DIAGNOSTIC] Executing 360-degree in-place turn in start square (0, 0)...");
        digitalWrite(PIN_STATUS_LED, HIGH);
        delay(400);

        motion.turnInPlace(-360.0f);

        digitalWrite(PIN_STATUS_LED, LOW);
        delay(250);

        // Stabilize start cell heading and sensors facing North (0.0 deg)
        imu.resetHeading(0.0f);
        motion.setTargetHeading(0.0f);
        encoders.reset();
        ir_sensors.flushFilter(8);

        Serial.println("[DIAGNOSTIC] 360-degree turn complete! Proceeding with Stage 1 search forward...\n");
    }

    while (!solver_.isAtGoal(current_pos_.x, current_pos_.y)) {
        StepResult result = stepExplore();
        if (result == StepResult::GoalReached) break;
        if (result != StepResult::Continue) return false;
        delay(60); // Deliberate square-by-square cadence
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
        delay(60); // Deliberate square-by-square cadence
    }

    Serial.println("[NAV] Successfully Returned to Start (0, 0)!");

    // Turn to face North in start cell
    if (current_heading_ != DIR_NORTH) {
        int8_t turn_code = (DIR_NORTH - current_heading_ + 4) & 0x03;
        bool turned = false;
        if (turn_code == 1) {
            turned = motion.turnInPlace(-90.0f);
        } else if (turn_code == 3) {
            turned = motion.turnInPlace(90.0f);
        } else if (turn_code == 2) {
            turned = motion.turnInPlace(-90.0f);
            delay(50);
            turned = turned && motion.turnInPlace(-90.0f);
        }
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
            if (!motion.turnInPlace(seg.value)) return false;
        } else if (seg.action == ACTION_TURN_RIGHT) {
            if (!motion.turnInPlace(seg.value)) return false;
        } else if (seg.action == ACTION_TURN_AROUND) {
            if (!motion.turnInPlace(seg.value)) return false;
        } else if (seg.action == ACTION_STOP) {
            motion.emergencyStop();
            break;
        }
    }

    Serial.println("[NAV] Speed Run Completed!");
    return true;
}
