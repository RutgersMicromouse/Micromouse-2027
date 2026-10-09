#pragma once

// The wall map (saved to flash)

#include "config.h"
#include "types.h"

// ==============================================================================
// MAZE MAP
// ==============================================================================

#include <Arduino.h>

// The robot's memory of the maze.
//
// Walls are not simply on or off: each one carries a tally of sensor readings (+1 each time it
// was seen, -1 each time the gap was seen open). That makes the map self-correcting, which
// matters because it is kept between attempts: a single false reading is outvoted next time the
// robot passes, instead of blocking a route for good.
class Maze {
public:
    Maze();

    // Forget everything except the outer walls
    void reset();

    // Is this cell inside the maze? (0 .. MAZE_ACTIVE_SIZE-1 both ways)
    static bool inMaze(int8_t x, int8_t y);

    // Is this one of the cells the robot is trying to reach?
    static bool isGoalCell(int8_t x, int8_t y);

    // Believed to be a wall: seen as a wall more often than as a gap, or an outer wall.
    // An opening nobody has looked at yet counts as open (the search is optimistic).
    bool hasWall(int8_t x, int8_t y, Direction dir) const;

    // Seen open more often than walled. Speed runs only drive through these.
    bool isKnownOpen(int8_t x, int8_t y, Direction dir) const;

    // Record one sensor reading of the wall on side `dir` of cell (x, y)
    void observeWall(int8_t x, int8_t y, Direction dir, bool wall_present);

    // The robot has just driven through this opening, which settles it: counts as the
    // strongest possible "open" reading
    void confirmOpen(int8_t x, int8_t y, Direction dir);

    // Drop walls that rest on a single reading (used when the map says the robot is walled in)
    void forgetWeakWalls();

    // Visited flags
    bool isVisited(int8_t x, int8_t y) const;
    void setVisited(int8_t x, int8_t y);

    // Record the three walls the robot can see from the centre of a cell, and mark it visited
    void updateCellWalls(int8_t x, int8_t y, Direction heading, bool wall_left, bool wall_front, bool wall_right);

    // Utility: get absolute direction from relative sensor orientation
    static Direction getAbsoluteDirection(Direction heading, int8_t relative_turn);

    // ASCII visualizer
    void printMazeToSerial(int8_t current_x, int8_t current_y) const;

    // Flash NVS persistence
    void saveToNVS();
    bool loadFromNVS();
    void clearNVS();
    bool hasSavedMaze() const;

#ifdef MAZE_SIZE_SWITCHABLE
    // Which maze size is in use (3 or 16) is itself kept in flash
    static void loadSizeFromNVS();
    static void saveSizeToNVS();
#endif

private:
    // The tally for one wall, or nullptr for an outer wall (always present, never changes).
    // Each wall is stored once: as the north wall or the east wall of the cell below / left of it.
    const int8_t* votesFor(int8_t x, int8_t y, Direction dir) const;

    int8_t wall_votes_[MAZE_WIDTH][MAZE_HEIGHT][2]; // [0] = north wall of the cell, [1] = east wall
    bool   visited_[MAZE_WIDTH][MAZE_HEIGHT];
};
