#include "optimizer.h"
#include "config.h"

PathOptimizer::PathOptimizer()
    : segment_count_(0) {
}

bool PathOptimizer::generatePath(const Maze& maze, Floodfill& solver, Coordinate start_pos, Direction start_heading) {
    segment_count_ = 0;
    solver.setGoalToCenter();
    solver.recalculate();

    Coordinate cur = start_pos;
    Direction heading = start_heading;

    if (solver.isAtGoal(cur.x, cur.y)) {
        return false;
    }

    uint16_t straight_cells = 0;

    for (int step = 0; step < TOTAL_CELLS; ++step) {
        if (solver.isAtGoal(cur.x, cur.y)) {
            // Flush any remaining straight movement
            if (straight_cells > 0) {
                PathSegment seg;
                seg.action = ACTION_FORWARD;
                seg.value = (float)straight_cells * CELL_DIMENSION_MM;
                seg.speed_mm_s = FAST_SPEED_MM_S;
                seg.end_speed_mm_s = 0.0f;
                segments_[segment_count_++] = seg;
            }
            PathSegment stop_seg;
            stop_seg.action = ACTION_STOP;
            stop_seg.value = 0.0f;
            stop_seg.speed_mm_s = 0.0f;
            stop_seg.end_speed_mm_s = 0.0f;
            segments_[segment_count_++] = stop_seg;
            return true;
        }

        Direction next_dir = solver.getNextDirection(cur.x, cur.y, heading);
        if (next_dir == DIR_INVALID) {
            Serial.println("[OPTIMIZER] Path broken, no valid next cell!");
            return false;
        }

        int8_t rel = (next_dir - heading + 4) & 0x03;

        if (rel == 0) {
            // Continue straight
            straight_cells++;
        } else {
            // Need a turn! First flush straight cells accumulated so far
            if (straight_cells > 0) {
                PathSegment seg;
                seg.action = ACTION_FORWARD;
                seg.value = (float)straight_cells * CELL_DIMENSION_MM;
                seg.speed_mm_s = FAST_SPEED_MM_S;
                seg.end_speed_mm_s = 0.0f;
                segments_[segment_count_++] = seg;
                straight_cells = 0;
            }

            // Append turn segment
            PathSegment turn_seg;
            if (rel == 1) {
                turn_seg.action = ACTION_TURN_RIGHT;
                turn_seg.value = -90.0f;
            } else if (rel == 3) {
                turn_seg.action = ACTION_TURN_LEFT;
                turn_seg.value = 90.0f;
            } else {
                turn_seg.action = ACTION_TURN_AROUND;
                turn_seg.value = 180.0f;
            }
            turn_seg.speed_mm_s = TURN_SPEED_DEG_S;
            turn_seg.end_speed_mm_s = 0.0f;
            segments_[segment_count_++] = turn_seg;

            heading = next_dir;
            straight_cells = 1;
        }

        cur.x += dxFromDir(next_dir);
        cur.y += dyFromDir(next_dir);
    }

    return false;
}

void PathOptimizer::printPath() const {
    Serial.printf("\n--- OPTIMIZED FAST RUN PATH (%d segments) ---\n", segment_count_);
    for (uint16_t i = 0; i < segment_count_; ++i) {
        const PathSegment& s = segments_[i];
        if (s.action == ACTION_FORWARD) {
            Serial.printf("[%02d] FORWARD: %5.1f mm @ %4.0f mm/s\n", i, s.value, s.speed_mm_s);
        } else if (s.action == ACTION_TURN_LEFT) {
            Serial.printf("[%02d] TURN LEFT: %5.1f deg\n", i, s.value);
        } else if (s.action == ACTION_TURN_RIGHT) {
            Serial.printf("[%02d] TURN RIGHT: %5.1f deg\n", i, s.value);
        } else if (s.action == ACTION_TURN_AROUND) {
            Serial.printf("[%02d] TURN AROUND: %5.1f deg\n", i, s.value);
        } else if (s.action == ACTION_STOP) {
            Serial.printf("[%02d] STOP\n", i);
        }
    }
}
