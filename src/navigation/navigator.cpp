#include "navigator.h"

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

void Navigator::scanCurrentCell() {
    // Determine absolute wall directions from robot's current heading
    Direction front_dir = current_heading_;
    Direction left_dir  = turnLeft(current_heading_);
    Direction right_dir = turnRight(current_heading_);

    bool front_wall = ir_sensors.hasFrontWall();
    bool left_wall  = ir_sensors.hasLeftWall();
    bool right_wall = ir_sensors.hasRightWall();

    maze_.setWall(current_pos_.x, current_pos_.y, front_dir, front_wall);
    maze_.setWall(current_pos_.x, current_pos_.y, left_dir,  left_wall);
    maze_.setWall(current_pos_.x, current_pos_.y, right_dir, right_wall);

    maze_.setVisited(current_pos_.x, current_pos_.y, true);

    Serial.printf("[NAV] Cell (%d,%d) Walls -> F(%s):%d, L(%s):%d, R(%s):%d\n",
                  current_pos_.x, current_pos_.y,
                  front_wall ? "YES" : "NO", front_dir,
                  left_wall  ? "YES" : "NO", left_dir,
                  right_wall ? "YES" : "NO", right_dir);
}

bool Navigator::stepExplore() {
    // 1. Scan and register walls at current cell
    scanCurrentCell();

    // 2. Recalculate floodfill distance map
    solver_.recalculate();

    // 3. Check if we have arrived at the goal
    if (solver_.isAtGoal(current_pos_.x, current_pos_.y)) {
        Serial.printf("[NAV] Arrived at Goal Cell (%d, %d)!\n", current_pos_.x, current_pos_.y);
        return true;
    }

    // 4. Select next direction
    Direction next_dir = solver_.getNextDirection(current_pos_.x, current_pos_.y, current_heading_);
    if (next_dir == DIR_INVALID) {
        Serial.println("[NAV] ERROR: Trapped! No accessible paths from current cell!");
        return false;
    }

    // 5. Determine relative turn needed
    int8_t turn_code = (next_dir - current_heading_ + 4) & 0x03;

    if (turn_code == 1) {
        // 90° Turn Right
        motion.turnInPlace(-90.0f, TURN_SPEED_DEG_S);
    } else if (turn_code == 3) {
        // 90° Turn Left
        motion.turnInPlace(90.0f, TURN_SPEED_DEG_S);
    } else if (turn_code == 2) {
        // 180° Turn Around
        motion.turnInPlace(180.0f, TURN_SPEED_DEG_S);
    }

    current_heading_ = next_dir;

    // 6. Move forward 1 cell
    motion.moveForward(CELL_DIMENSION_MM, SEARCH_SPEED_MM_S, 0.0f, true);

    // Update coordinates
    current_pos_.x += dxFromDir(current_heading_);
    current_pos_.y += dyFromDir(current_heading_);

    return false; // Still exploring
}

bool Navigator::exploreToCenter() {
    Serial.println("\n[NAV] =======================================");
    Serial.println("[NAV] Starting Center Exploration Run...");
    Serial.println("[NAV] =======================================");

    solver_.setGoalToCenter();
    solver_.recalculate();

    while (!solver_.isAtGoal(current_pos_.x, current_pos_.y)) {
        if (battery.isCritical()) {
            Serial.println("[NAV] Aborting exploration due to critical battery!");
            return false;
        }

        bool goal_reached = stepExplore();
        if (goal_reached) break;
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

bool Navigator::exploreToStart() {
    Serial.println("\n[NAV] Starting Return Journey to Start (0, 0)...");

    solver_.setGoalToStart();
    solver_.recalculate();

    while (!solver_.isAtGoal(current_pos_.x, current_pos_.y)) {
        if (battery.isCritical()) return false;
        bool goal_reached = stepExplore();
        if (goal_reached) break;
    }

    Serial.println("[NAV] Successfully Returned to Start (0, 0)!");

    // Turn to face North in start cell
    if (current_heading_ != DIR_NORTH) {
        int8_t turn_code = (DIR_NORTH - current_heading_ + 4) & 0x03;
        if (turn_code == 1)      motion.turnInPlace(-90.0f);
        else if (turn_code == 3) motion.turnInPlace(90.0f);
        else if (turn_code == 2) motion.turnInPlace(180.0f);
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

        if (battery.isCritical()) {
            motion.emergencyStop();
            return false;
        }

        if (seg.action == ACTION_FORWARD) {
            motion.moveForward(seg.value, seg.speed_mm_s, seg.end_speed_mm_s, true);
        } else if (seg.action == ACTION_TURN_LEFT) {
            motion.turnInPlace(seg.value, TURN_SPEED_DEG_S);
        } else if (seg.action == ACTION_TURN_RIGHT) {
            motion.turnInPlace(seg.value, TURN_SPEED_DEG_S);
        } else if (seg.action == ACTION_TURN_AROUND) {
            motion.turnInPlace(seg.value, TURN_SPEED_DEG_S);
        } else if (seg.action == ACTION_STOP) {
            motion.emergencyStop();
            break;
        }
    }

    Serial.println("[NAV] Speed Run Completed!");
    return true;
}
