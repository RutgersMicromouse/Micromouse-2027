#pragma once

#include <stdint.h>
#include "config.h"

// Cell Wall Bitmasks
#define WALL_NORTH_BIT    (1 << 0)
#define WALL_EAST_BIT     (1 << 1)
#define WALL_SOUTH_BIT    (1 << 2)
#define WALL_WEST_BIT     (1 << 3)
#define CELL_VISITED_BIT  (1 << 4)

// Distance Matrix Sentinel Value
#define DIST_INFINITY     0xFFFF

struct Coordinate {
    int8_t x;
    int8_t y;

    bool operator==(const Coordinate& other) const {
        return (x == other.x && y == other.y);
    }
};
