#pragma once

#include <stdint.h>
#include "maze_constants.h"
#include "maze.h"
#include "types.h"

class Dijkstra {
public:
    Dijkstra(const Maze& maze);

    // Compute fastest path to center goals: (7,7), (7,8), (8,7), (8,8)
    uint8_t findFastestPathToCenter(int8_t start_x, int8_t start_y, Direction start_h,
                                   Coordinate* out_path, uint8_t max_path_len);

    // Compute fastest path to start goal: (0,0)
    uint8_t findFastestPathToStart(int8_t start_x, int8_t start_y, Direction start_h,
                                  Coordinate* out_path, uint8_t max_path_len);

    // General search with custom goal callback/matrix
    uint8_t findFastestPath(int8_t start_x, int8_t start_y, Direction start_h,
                            bool (*is_goal_fn)(int8_t, int8_t),
                            Coordinate* out_path, uint8_t max_path_len);

    struct HeapNode {
        float cost;
        int8_t x;
        int8_t y;
        uint8_t h;
    };

    struct ParentEdge {
        int8_t prev_x;
        int8_t prev_y;
        uint8_t prev_h;
        uint8_t move_type; // 0=turn, 1=straight, 2=diagonal, 3=slalom
        uint8_t count;
        uint8_t d_cross;
        bool has_parent;
    };

private:
    const Maze& maze_;
};
