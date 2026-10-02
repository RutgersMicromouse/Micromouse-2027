#include "maze.h"

Maze::Maze() {
    reset();
}

void Maze::reset() {
    memset(cells_, 0, sizeof(cells_));

    // Install outer boundary perimeter walls
    for (int8_t x = 0; x < MAZE_WIDTH; ++x) {
        cells_[x][0]               |= WALL_SOUTH;
        cells_[x][MAZE_HEIGHT - 1] |= WALL_NORTH;
    }
    for (int8_t y = 0; y < MAZE_HEIGHT; ++y) {
        cells_[0][y]              |= WALL_WEST;
        cells_[MAZE_WIDTH - 1][y] |= WALL_EAST;
    }

    // Standard micromouse start cell (0, 0) has East wall
    cells_[0][0] |= WALL_EAST;
    cells_[1][0] |= WALL_WEST;
}

bool Maze::isValidCoordinate(int8_t x, int8_t y) {
    return (x >= 0 && x < MAZE_WIDTH && y >= 0 && y < MAZE_HEIGHT);
}

bool Maze::hasWall(int8_t x, int8_t y, Direction dir) const {
    if (!isValidCoordinate(x, y) || dir == DIR_INVALID) return true;
    uint8_t mask = wallBitFromDir(dir);
    return (cells_[x][y] & mask) != 0;
}

void Maze::setWall(int8_t x, int8_t y, Direction dir, bool present) {
    if (!isValidCoordinate(x, y) || dir == DIR_INVALID) return;

    uint8_t mask = wallBitFromDir(dir);
    if (present) {
        cells_[x][y] |= mask;
    } else {
        cells_[x][y] &= ~mask;
    }

    // Set corresponding reciprocal wall in adjacent neighbor cell
    int8_t nx = x + dxFromDir(dir);
    int8_t ny = y + dyFromDir(dir);
    if (isValidCoordinate(nx, ny)) {
        uint8_t opp_mask = wallBitFromDir(oppositeDirection(dir));
        if (present) {
            cells_[nx][ny] |= opp_mask;
        } else {
            cells_[nx][ny] &= ~opp_mask;
        }
    }
}

bool Maze::isVisited(int8_t x, int8_t y) const {
    if (!isValidCoordinate(x, y)) return false;
    return (cells_[x][y] & CELL_VISITED) != 0;
}

void Maze::setVisited(int8_t x, int8_t y, bool visited) {
    if (!isValidCoordinate(x, y)) return;
    if (visited) {
        cells_[x][y] |= CELL_VISITED;
    } else {
        cells_[x][y] &= ~CELL_VISITED;
    }
}

uint8_t Maze::getCellRaw(int8_t x, int8_t y) const {
    if (!isValidCoordinate(x, y)) return 0xFF;
    return cells_[x][y];
}

void Maze::printAscii() const {
    Serial.println("\n--- MAZE MAP (16x16) ---");
    for (int8_t y = MAZE_HEIGHT - 1; y >= 0; --y) {
        // Top horizontal walls
        for (int8_t x = 0; x < MAZE_WIDTH; ++x) {
            Serial.print("+");
            Serial.print(hasWall(x, y, DIR_NORTH) ? "---" : "   ");
        }
        Serial.println("+");

        // Vertical walls and cell space
        for (int8_t x = 0; x < MAZE_WIDTH; ++x) {
            Serial.print(hasWall(x, y, DIR_WEST) ? "|" : " ");
            if (isVisited(x, y)) {
                Serial.print(" . ");
            } else {
                Serial.print("   ");
            }
        }
        Serial.println(hasWall(MAZE_WIDTH - 1, y, DIR_EAST) ? "|" : " ");
    }

    // Bottom row
    for (int8_t x = 0; x < MAZE_WIDTH; ++x) {
        Serial.print("+---");
    }
    Serial.println("+");
}
