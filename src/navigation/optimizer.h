#pragma once

#include <Arduino.h>
#include "maze_constants.h"
#include "types.h"
#include "maze.h"
#include "floodfill.h"

// =============================================================================
// Speed-Run Path Optimizer
// Compresses cell-by-cell path into high-speed multi-cell sprints
// =============================================================================

enum ActionType : uint8_t {
    ACTION_FORWARD = 0,
    ACTION_TURN_LEFT,
    ACTION_TURN_RIGHT,
    ACTION_TURN_AROUND,
    ACTION_STOP
};

struct PathSegment {
    ActionType action;
    float value;           // distance in mm for FORWARD, angle in deg for turns
    float speed_mm_s;      // top cruise speed
    float end_speed_mm_s;  // speed at end of segment
};

class PathOptimizer {
public:
    PathOptimizer();

    // Compute optimized trajectory from start to center
    bool generatePath(const Maze& maze, Floodfill& solver, Coordinate start_pos, Direction start_heading);

    uint16_t getSegmentCount() const { return segment_count_; }
    const PathSegment& getSegment(uint16_t index) const { return segments_[index]; }

    void printPath() const;

private:
    // A worst-case cell path can alternate a turn and a forward segment.
    PathSegment segments_[TOTAL_CELLS * 2 + 1];
    uint16_t segment_count_;
};
