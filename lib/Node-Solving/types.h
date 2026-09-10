#pragma once
#include <vector>

//Global cardinal directions
enum class Direction { NORTH = 0, EAST = 1, SOUTH = 2, WEST = 3};

// Instruction types matching two rotate variants
enum class InstructionType {
    FORWARD,
    ROTATE_RELATIVE, // relative to bot's current heading
    ROTATE_GLOBAL // absolute cardinal (0 = North always)
};

struct Instruction {
    InstructionType type;
    int value; // cells for forward, degrees for rotate, or direction cast for rotate_global
};


struct RobotState {
    int current_node_id;
    Direction heading;
    int x, y;
};


