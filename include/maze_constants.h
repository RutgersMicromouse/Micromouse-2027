#pragma once

#include <stdint.h>

// =============================================================================
// Micromouse Maze Constants & Direction Utilities
// =============================================================================

#define MAZE_WIDTH             16
#define MAZE_HEIGHT            16
#define TOTAL_CELLS            (MAZE_WIDTH * MAZE_HEIGHT)
#define DIST_INFINITY          0xFFFF

// Wall presence flags (bitfield stored per cell)
#define WALL_NORTH             0x01
#define WALL_EAST              0x02
#define WALL_SOUTH             0x04
#define WALL_WEST              0x08
#define CELL_VISITED           0x10
#define WALL_MASK              0x0F

enum Direction : int8_t {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3,
    DIR_INVALID = -1
};

// Direction helper utilities
inline Direction oppositeDirection(Direction dir) {
    if (dir == DIR_INVALID) return DIR_INVALID;
    return static_cast<Direction>((dir + 2) & 0x03);
}

inline Direction turnRight(Direction dir) {
    return static_cast<Direction>((dir + 1) & 0x03);
}

inline Direction turnLeft(Direction dir) {
    return static_cast<Direction>((dir + 3) & 0x03);
}

inline Direction relativeToAbsolute(Direction current_heading, int8_t relative_turn) {
    // relative_turn: 0 = straight, 1 = right, -1 = left, 2 = 180
    return static_cast<Direction>((current_heading + relative_turn + 4) & 0x03);
}

inline int8_t dxFromDir(Direction dir) {
    switch (dir) {
        case DIR_EAST:  return 1;
        case DIR_WEST:  return -1;
        default:        return 0;
    }
}

inline int8_t dyFromDir(Direction dir) {
    switch (dir) {
        case DIR_NORTH: return 1;
        case DIR_SOUTH: return -1;
        default:        return 0;
    }
}

inline uint8_t wallBitFromDir(Direction dir) {
    switch (dir) {
        case DIR_NORTH: return WALL_NORTH;
        case DIR_EAST:  return WALL_EAST;
        case DIR_SOUTH: return WALL_SOUTH;
        case DIR_WEST:  return WALL_WEST;
        default:        return 0;
    }
}
