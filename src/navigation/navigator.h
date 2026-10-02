#pragma once

#include <Arduino.h>
#include "maze_constants.h"
#include "types.h"
#include "maze.h"
#include "floodfill.h"
#include "optimizer.h"
#include "control/motion_controller.h"

// =============================================================================
// Top-Level Autonomous Navigation & State Machine
// =============================================================================

class Navigator {
public:
    Navigator();

    void reset();

    // Exploration to Center (7,7)-(8,8)
    bool exploreToCenter();

    // Exploration return journey back to Start (0,0)
    bool exploreToStart();

    // High-speed speed-run based on mapped walls
    bool runFastSpeed();

    // Update walls at the current robot position using IR sensors
    void scanCurrentCell();

    // Status queries
    Coordinate getCurrentPosition() const { return current_pos_; }
    Direction getCurrentHeading() const   { return current_heading_; }

    Maze& getMaze() { return maze_; }

private:
    // Move 1 cell in current or new direction
    bool stepExplore();

    Maze maze_;
    Floodfill solver_;
    PathOptimizer optimizer_;

    Coordinate current_pos_;
    Direction current_heading_;
    bool in_speed_run_;
};

extern Navigator navigator;
