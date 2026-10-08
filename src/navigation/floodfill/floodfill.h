#pragma once

// Search-run solver: explore towards the goal

#include "config.h"
#include "types.h"
#include "navigation/maze/maze.h"

// ==============================================================================
// FLOODFILL SOLVER
// ==============================================================================

#include <Arduino.h>

class Floodfill {
public:
    Floodfill(const Maze& maze);

    // Set destination goals: Center (7,7 - 8,8) or Start (0,0)
    void setGoalToCenter();
    void setGoalToStart();

    // Recompute floodfill distance matrix
    void recalculate();

    // Determine the optimal next direction to move from current cell
    Direction getNextDirection(int8_t current_x, int8_t current_y, Direction current_heading);

    // Get calculated distance value for a cell
    uint16_t getDistance(int8_t x, int8_t y) const;

    // Check if the robot has reached its goal
    bool isAtGoal(int8_t x, int8_t y) const;

private:
    const Maze& maze_;
    uint16_t distance_[MAZE_WIDTH][MAZE_HEIGHT];
    bool is_goal_[MAZE_WIDTH][MAZE_HEIGHT];
};
