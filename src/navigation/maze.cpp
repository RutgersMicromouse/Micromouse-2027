#include "maze.h"

Maze::Maze() {
    reset();
}

void Maze::reset() {
    memset(cells_, 0, sizeof(cells_));
    memset(known_west_, 0, sizeof(known_west_));

    // Install outer boundary perimeter walls
    for (int8_t x = 0; x < MAZE_WIDTH; ++x) {
        cells_[x][0]               |= WALL_SOUTH;
        cells_[x][MAZE_HEIGHT - 1] |= WALL_NORTH;
        setWallKnown(x, 0, DIR_SOUTH);
        setWallKnown(x, MAZE_HEIGHT - 1, DIR_NORTH);
    }
    for (int8_t y = 0; y < MAZE_HEIGHT; ++y) {
        cells_[0][y]              |= WALL_WEST;
        cells_[MAZE_WIDTH - 1][y] |= WALL_EAST;
        setWallKnown(0, y, DIR_WEST);
        setWallKnown(MAZE_WIDTH - 1, y, DIR_EAST);
    }

    // Official Micromouse starting cell (0,0) has a permanent East wall
    setWall(0, 0, DIR_EAST, true);
}

uint8_t Maze::knownBitFromDir(Direction dir) {
    switch (dir) {
        case DIR_NORTH: return KNOWN_NORTH;
        case DIR_EAST:  return KNOWN_EAST;
        case DIR_SOUTH: return KNOWN_SOUTH;
        default:        return 0;
    }
}

void Maze::setWallKnown(int8_t x, int8_t y, Direction dir) {
    if (!isValidCoordinate(x, y) || dir == DIR_INVALID) return;
    if (dir == DIR_WEST) {
        known_west_[x][y] = 1;
    } else {
        cells_[x][y] |= knownBitFromDir(dir);
    }
}

bool Maze::isValidCoordinate(int8_t x, int8_t y) {
    return (x >= 0 && x < MAZE_WIDTH && y >= 0 && y < MAZE_HEIGHT);
}

bool Maze::hasWall(int8_t x, int8_t y, Direction dir) const {
    if (!isValidCoordinate(x, y) || dir == DIR_INVALID) return true;
    uint8_t mask = wallBitFromDir(dir);
    return (cells_[x][y] & mask) != 0;
}

bool Maze::isWallKnown(int8_t x, int8_t y, Direction dir) const {
    if (!isValidCoordinate(x, y) || dir == DIR_INVALID) return true;
    if (dir == DIR_WEST) return known_west_[x][y] != 0;
    return (cells_[x][y] & knownBitFromDir(dir)) != 0;
}

void Maze::setWall(int8_t x, int8_t y, Direction dir, bool present) {
    if (!isValidCoordinate(x, y) || dir == DIR_INVALID) return;

    // The physical perimeter is immutable even if a sensor misses it.
    int8_t nx = x + dxFromDir(dir);
    int8_t ny = y + dyFromDir(dir);
    if (!isValidCoordinate(nx, ny)) present = true;

    // Official Micromouse competition rule: starting cell (0,0) East wall is immutable
    if ((x == 0 && y == 0 && dir == DIR_EAST) ||
        (x == 1 && y == 0 && dir == DIR_WEST)) {
        present = true;
    }

    // Monotonic wall permanence: Once a physical wall is confirmed present,
    // never erase it due to sensor noise or opening glare
    if (!present && hasWall(x, y, dir) && isWallKnown(x, y, dir)) {
        return;
    }

    uint8_t mask = wallBitFromDir(dir);
    if (present) {
        cells_[x][y] |= mask;
    } else {
        cells_[x][y] &= ~mask;
    }

    setWallKnown(x, y, dir);

    // Set corresponding reciprocal wall and observation in adjacent neighbor.
    if (isValidCoordinate(nx, ny)) {
        uint8_t opp_mask = wallBitFromDir(oppositeDirection(dir));
        if (present) {
            cells_[nx][ny] |= opp_mask;
        } else {
            cells_[nx][ny] &= ~opp_mask;
        }
        setWallKnown(nx, ny, oppositeDirection(dir));
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

void Maze::printAscii(int8_t robot_x, int8_t robot_y, Direction robot_heading) const {
    Serial.printf("\n--- MAZE MAP (%dx%d) ---\n", MAZE_WIDTH, MAZE_HEIGHT);
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
            if (x == robot_x && y == robot_y) {
                char arrow = '^';
                if (robot_heading == DIR_EAST) arrow = '>';
                else if (robot_heading == DIR_SOUTH) arrow = 'v';
                else if (robot_heading == DIR_WEST) arrow = '<';
                Serial.printf(" %c ", arrow);
            } else if (isVisited(x, y)) {
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

