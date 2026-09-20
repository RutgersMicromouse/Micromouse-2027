#include "maze.h"
#include <Preferences.h>

Maze::Maze() {
    reset();
}

void Maze::reset() {
    memset(cells_, 0, sizeof(cells_));

    // Initialize outer boundary walls
    for (int8_t x = 0; x < MAZE_WIDTH; ++x) {
        cells_[x][0] |= WALL_SOUTH_BIT;
        cells_[x][MAZE_HEIGHT - 1] |= WALL_NORTH_BIT;
    }

    for (int8_t y = 0; y < MAZE_HEIGHT; ++y) {
        cells_[0][y] |= WALL_WEST_BIT;
        cells_[MAZE_WIDTH - 1][y] |= WALL_EAST_BIT;
    }

    // Standard starting cell (0, 0) has an East wall in classical micromouse rules
    cells_[0][0] |= WALL_EAST_BIT;
    cells_[1][0] |= WALL_WEST_BIT;
}

bool Maze::hasWall(int8_t x, int8_t y, Direction dir) const {
    if (x < 0 || x >= MAZE_WIDTH || y < 0 || y >= MAZE_HEIGHT) {
        return true; // Boundaries treated as walls
    }

    switch (dir) {
        case DIR_NORTH: return (cells_[x][y] & WALL_NORTH_BIT) != 0;
        case DIR_EAST:  return (cells_[x][y] & WALL_EAST_BIT)  != 0;
        case DIR_SOUTH: return (cells_[x][y] & WALL_SOUTH_BIT) != 0;
        case DIR_WEST:  return (cells_[x][y] & WALL_WEST_BIT)  != 0;
        default:        return true;
    }
}

void Maze::setWall(int8_t x, int8_t y, Direction dir) {
    if (x < 0 || x >= MAZE_WIDTH || y < 0 || y >= MAZE_HEIGHT) return;

    switch (dir) {
        case DIR_NORTH:
            cells_[x][y] |= WALL_NORTH_BIT;
            if (y + 1 < MAZE_HEIGHT) cells_[x][y + 1] |= WALL_SOUTH_BIT;
            break;
        case DIR_EAST:
            cells_[x][y] |= WALL_EAST_BIT;
            if (x + 1 < MAZE_WIDTH) cells_[x + 1][y] |= WALL_WEST_BIT;
            break;
        case DIR_SOUTH:
            cells_[x][y] |= WALL_SOUTH_BIT;
            if (y - 1 >= 0) cells_[x][y - 1] |= WALL_NORTH_BIT;
            break;
        case DIR_WEST:
            cells_[x][y] |= WALL_WEST_BIT;
            if (x - 1 >= 0) cells_[x - 1][y] |= WALL_EAST_BIT;
            break;
        default:
            break;
    }
}

void Maze::clearWall(int8_t x, int8_t y, Direction dir) {
    if (x < 0 || x >= MAZE_WIDTH || y < 0 || y >= MAZE_HEIGHT) return;

    switch (dir) {
        case DIR_NORTH:
            cells_[x][y] &= ~WALL_NORTH_BIT;
            if (y + 1 < MAZE_HEIGHT) cells_[x][y + 1] &= ~WALL_SOUTH_BIT;
            break;
        case DIR_EAST:
            cells_[x][y] &= ~WALL_EAST_BIT;
            if (x + 1 < MAZE_WIDTH) cells_[x + 1][y] &= ~WALL_WEST_BIT;
            break;
        case DIR_SOUTH:
            cells_[x][y] &= ~WALL_SOUTH_BIT;
            if (y - 1 >= 0) cells_[x][y - 1] &= ~WALL_NORTH_BIT;
            break;
        case DIR_WEST:
            cells_[x][y] &= ~WALL_WEST_BIT;
            if (x - 1 >= 0) cells_[x - 1][y] &= ~WALL_EAST_BIT;
            break;
        default:
            break;
    }
}

bool Maze::isVisited(int8_t x, int8_t y) const {
    if (x < 0 || x >= MAZE_WIDTH || y < 0 || y >= MAZE_HEIGHT) return false;
    return (cells_[x][y] & CELL_VISITED_BIT) != 0;
}

void Maze::setVisited(int8_t x, int8_t y) {
    if (x >= 0 && x < MAZE_WIDTH && y >= 0 && y < MAZE_HEIGHT) {
        cells_[x][y] |= CELL_VISITED_BIT;
    }
}

Direction Maze::getAbsoluteDirection(Direction heading, int8_t relative_turn) {
    // relative_turn: 0 = front, 1 = right, 2 = behind, 3 (or -1) = left
    int8_t dir = ((int8_t)heading + relative_turn) % 4;
    if (dir < 0) dir += 4;
    return (Direction)dir;
}

void Maze::updateCellWalls(int8_t x, int8_t y, Direction heading, bool wall_left, bool wall_front, bool wall_right) {
    setVisited(x, y);

    Direction dir_front = getAbsoluteDirection(heading, 0);
    Direction dir_right = getAbsoluteDirection(heading, 1);
    Direction dir_left  = getAbsoluteDirection(heading, -1);

    if (wall_front) setWall(x, y, dir_front);
    if (wall_right) setWall(x, y, dir_right);
    if (wall_left)  setWall(x, y, dir_left);
}

void Maze::printMazeToSerial(int8_t current_x, int8_t current_y) const {
    Serial.println("\n--- Current Maze Grid ---");
    for (int8_t y = MAZE_HEIGHT - 1; y >= 0; --y) {
        // Print north walls
        for (int8_t x = 0; x < MAZE_WIDTH; ++x) {
            Serial.print("+");
            Serial.print(hasWall(x, y, DIR_NORTH) ? "---" : "   ");
        }
        Serial.println("+");

        // Print west/east walls and mouse location
        for (int8_t x = 0; x < MAZE_WIDTH; ++x) {
            Serial.print(hasWall(x, y, DIR_WEST) ? "|" : " ");
            if (x == current_x && y == current_y) {
                Serial.print(" M ");
            } else if (isVisited(x, y)) {
                Serial.print(" . ");
            } else {
                Serial.print("   ");
            }
        }
        Serial.println(hasWall(MAZE_WIDTH - 1, y, DIR_EAST) ? "|" : " ");
    }

    // Bottom south walls
    for (int8_t x = 0; x < MAZE_WIDTH; ++x) {
        Serial.print("+---");
    }
    Serial.println("+");
}

void Maze::saveToNVS() {
    Preferences prefs;
    prefs.begin("maze_grid", false);
    prefs.putBytes("cells", cells_, sizeof(cells_));
    prefs.putBool("valid", true);
    prefs.end();
    Serial.println("[MAZE] Mapped maze saved to Flash NVS!");
}

bool Maze::loadFromNVS() {
    Preferences prefs;
    prefs.begin("maze_grid", true);
    if (!prefs.getBool("valid", false)) {
        prefs.end();
        return false;
    }
    prefs.getBytes("cells", cells_, sizeof(cells_));
    prefs.end();
    Serial.println("[MAZE] Mapped maze loaded from Flash NVS!");
    return true;
}

void Maze::clearNVS() {
    Preferences prefs;
    prefs.begin("maze_grid", false);
    prefs.clear();
    prefs.end();
    Serial.println("[MAZE] Flash NVS maze cleared.");
}

bool Maze::hasSavedMaze() const {
    Preferences prefs;
    prefs.begin("maze_grid", true);
    bool valid = prefs.getBool("valid", false);
    prefs.end();
    return valid;
}
