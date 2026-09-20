#pragma once

#include <Arduino.h>
#include "maze_constants.h"
#include "types.h"

class Maze {
public:
    Maze();
    void reset();

    // Wall inspection and updates
    bool hasWall(int8_t x, int8_t y, Direction dir) const;
    void setWall(int8_t x, int8_t y, Direction dir);
    void clearWall(int8_t x, int8_t y, Direction dir);

    // Visited flags
    bool isVisited(int8_t x, int8_t y) const;
    void setVisited(int8_t x, int8_t y);

    // Update cell walls based on robot heading and detected IR walls
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

private:
    uint8_t cells_[MAZE_WIDTH][MAZE_HEIGHT];
};
