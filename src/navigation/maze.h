#pragma once

#include <Arduino.h>
#include "maze_constants.h"
#include "types.h"

// =============================================================================
// 16x16 Micromouse Maze Representation
// Wall Bitflags: NORTH=0x01, EAST=0x02, SOUTH=0x04, WEST=0x08, VISITED=0x10
// =============================================================================

class Maze {
public:
    Maze();

    // Reset maze, install boundary walls, and set start cell walls
    void reset();

    // Wall inspection and mutation
    bool hasWall(int8_t x, int8_t y, Direction dir) const;
    void setWall(int8_t x, int8_t y, Direction dir, bool present = true);

    bool isVisited(int8_t x, int8_t y) const;
    void setVisited(int8_t x, int8_t y, bool visited = true);

    uint8_t getCellRaw(int8_t x, int8_t y) const;

    // Check if cell coordinate is inside the 16x16 maze bounds
    static bool isValidCoordinate(int8_t x, int8_t y);

    // Print ASCII representation of known walls to Serial console
    void printAscii() const;

private:
    uint8_t cells_[MAZE_WIDTH][MAZE_HEIGHT];
};
