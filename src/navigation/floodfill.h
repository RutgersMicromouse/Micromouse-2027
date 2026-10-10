#pragma once

#include <Arduino.h>
#include "maze_constants.h"
#include "maze.h"
#include "types.h"

// =============================================================================
// Wavefront Floodfill Maze Solver
// Supports center, start, and custom waypoint goals
// =============================================================================

class Floodfill {
public:
    explicit Floodfill(const Maze& maze);

    // Goal Configuration
    void setGoalToCenter();
    void setGoalToStart();
    void setCustomGoal(int8_t x, int8_t y);

    // Recalculate distance matrix across all 256 cells
    // Exploration may treat unknown edges as passable; speed runs must use
    // only edges that have been explicitly observed open.
    void recalculate(bool known_edges_only = false);

    // Query distance metric
    uint16_t getDistance(int8_t x, int8_t y) const;
    bool isAtGoal(int8_t x, int8_t y) const;

    // Determine optimal direction to move next
    Direction getNextDirection(int8_t current_x, int8_t current_y, Direction current_heading);

    // Print complete distance matrix to Serial console for real-time verification
    void printDistanceMatrix() const;

private:
    const Maze& maze_;
    uint16_t distance_[MAZE_WIDTH][MAZE_HEIGHT];
    bool is_goal_[MAZE_WIDTH][MAZE_HEIGHT];
    bool known_edges_only_;
};
