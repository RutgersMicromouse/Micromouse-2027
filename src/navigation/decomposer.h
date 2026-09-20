#pragma once

#include <stdint.h>
#include "maze_constants.h"
#include "types.h"

class Decomposer {
public:
    // Decomposes a continuous sequence of cells into optimal execution primitives:
    // - SEG_SLALOM: Multi-wave up-and-down / left-and-right continuous zigzags
    // - SEG_DIAGONAL: Diagonal staircases and 45° corner cuts
    // - SEG_STRAIGHT: Multi-cell straight corridor sprints
    static uint8_t decompose(const Coordinate* path, uint8_t path_len,
                             PathSegment* out_segments, uint8_t max_segments,
                             bool allow_diagonals = true);

    static Direction getDirection(Coordinate from, Coordinate to);

private:
    static uint8_t findSlalomLength(const Direction* dirs, uint8_t start_idx, uint8_t n);
};
