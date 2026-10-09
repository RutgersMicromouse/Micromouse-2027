#include "navigation/maze/maze.h"

// ==============================================================================
// MAZE MAP
// ==============================================================================

#include <Preferences.h>

// Flash storage name. The practice maze keeps its own map so it never mixes with a real one.
#define MAZE_STORAGE ((MAZE_ACTIVE_SIZE == 16) ? "maze_votes" : "maze_small")

#ifdef MAZE_SIZE_SWITCHABLE
// The maze size in use (see MAZE_ACTIVE_SIZE in config.h), switchable while the robot is on
int g_maze_size = MAZE_DEFAULT_SIZE;

void Maze::loadSizeFromNVS() {
    Preferences prefs;
    prefs.begin("maze_cfg", true);
    const int size = prefs.getInt("size", MAZE_DEFAULT_SIZE);
    prefs.end();
    g_maze_size = (size == 16) ? 16 : 3;
}

void Maze::saveSizeToNVS() {
    Preferences prefs;
    prefs.begin("maze_cfg", false);
    prefs.putInt("size", g_maze_size);
    prefs.end();
}
#endif

Maze::Maze() {
    reset();
}

void Maze::reset() {
    memset(wall_votes_, 0, sizeof(wall_votes_));
    memset(visited_, 0, sizeof(visited_));

    // The start cell has a wall on its right in every standard maze. Held as one ordinary vote,
    // so a clear reading overrules it if a practice maze is built differently.
    observeWall(0, 0, DIR_EAST, true);
}

bool Maze::inMaze(int8_t x, int8_t y) {
    return x >= 0 && x < MAZE_ACTIVE_SIZE && y >= 0 && y < MAZE_ACTIVE_SIZE;
}

bool Maze::isGoalCell(int8_t x, int8_t y) {
    // Even-sized maze: the four cells around the middle. Odd-sized: the one middle cell.
    const int8_t hi = MAZE_ACTIVE_SIZE / 2;
    const int8_t lo = (MAZE_ACTIVE_SIZE % 2 == 0) ? (int8_t)(hi - 1) : hi;
    return x >= lo && x <= hi && y >= lo && y <= hi;
}

const int8_t* Maze::votesFor(int8_t x, int8_t y, Direction dir) const {
    if (!inMaze(x, y)) return nullptr;
    switch (dir) {
        case DIR_NORTH: return inMaze(x, y + 1) ? &wall_votes_[x][y][0]     : nullptr;
        case DIR_EAST:  return inMaze(x + 1, y) ? &wall_votes_[x][y][1]     : nullptr;
        case DIR_SOUTH: return inMaze(x, y - 1) ? &wall_votes_[x][y - 1][0] : nullptr;
        case DIR_WEST:  return inMaze(x - 1, y) ? &wall_votes_[x - 1][y][1] : nullptr;
        default:        return nullptr;
    }
}

bool Maze::hasWall(int8_t x, int8_t y, Direction dir) const {
    const int8_t* votes = votesFor(x, y, dir);
    return votes == nullptr || *votes > 0; // Outer walls are always there
}

bool Maze::isKnownOpen(int8_t x, int8_t y, Direction dir) const {
    const int8_t* votes = votesFor(x, y, dir);
    return votes != nullptr && *votes < 0;
}

void Maze::observeWall(int8_t x, int8_t y, Direction dir, bool wall_present) {
    int8_t* votes = const_cast<int8_t*>(votesFor(x, y, dir));
    if (votes == nullptr) return;
    if (wall_present  && *votes <  WALL_VOTE_LIMIT) (*votes)++;
    if (!wall_present && *votes > -WALL_VOTE_LIMIT) (*votes)--;
}

void Maze::confirmOpen(int8_t x, int8_t y, Direction dir) {
    int8_t* votes = const_cast<int8_t*>(votesFor(x, y, dir));
    if (votes != nullptr) *votes = -WALL_VOTE_LIMIT;
}

void Maze::forgetWeakWalls() {
    for (int8_t x = 0; x < MAZE_WIDTH; ++x) {
        for (int8_t y = 0; y < MAZE_HEIGHT; ++y) {
            for (int8_t side = 0; side < 2; ++side) {
                if (wall_votes_[x][y][side] == 1) wall_votes_[x][y][side] = 0;
            }
        }
    }
}

bool Maze::isVisited(int8_t x, int8_t y) const {
    return inMaze(x, y) && visited_[x][y];
}

void Maze::setVisited(int8_t x, int8_t y) {
    if (inMaze(x, y)) visited_[x][y] = true;
}

Direction Maze::getAbsoluteDirection(Direction heading, int8_t relative_turn) {
    // relative_turn: 0 = front, 1 = right, 2 = behind, 3 (or -1) = left
    int8_t dir = ((int8_t)heading + relative_turn) % 4;
    if (dir < 0) dir += 4;
    return (Direction)dir;
}

void Maze::updateCellWalls(int8_t x, int8_t y, Direction heading, bool wall_left, bool wall_front, bool wall_right) {
    setVisited(x, y);
    observeWall(x, y, getAbsoluteDirection(heading, 0),  wall_front);
    observeWall(x, y, getAbsoluteDirection(heading, 1),  wall_right);
    observeWall(x, y, getAbsoluteDirection(heading, -1), wall_left);
}

void Maze::printMazeToSerial(int8_t current_x, int8_t current_y) const {
    Serial.println("\n--- Current Maze Grid ---");
    for (int8_t y = MAZE_ACTIVE_SIZE - 1; y >= 0; --y) {
        // Print north walls
        for (int8_t x = 0; x < MAZE_ACTIVE_SIZE; ++x) {
            Serial.print("+");
            Serial.print(hasWall(x, y, DIR_NORTH) ? "---" : "   ");
        }
        Serial.println("+");

        // Print west/east walls and mouse location
        for (int8_t x = 0; x < MAZE_ACTIVE_SIZE; ++x) {
            Serial.print(hasWall(x, y, DIR_WEST) ? "|" : " ");
            if (x == current_x && y == current_y) {
                Serial.print(" M ");
            } else if (isVisited(x, y)) {
                Serial.print(" . ");
            } else {
                Serial.print("   ");
            }
        }
        Serial.println("|");
    }
    // Bottom south walls
    for (int8_t x = 0; x < MAZE_ACTIVE_SIZE; ++x) {
        Serial.print("+---");
    }
    Serial.println("+");
}

void Maze::saveToNVS() {
    Preferences prefs;
    prefs.begin(MAZE_STORAGE, false);
    prefs.putBytes("votes", wall_votes_, sizeof(wall_votes_));
    prefs.putBytes("visited", visited_, sizeof(visited_));
    prefs.putBool("valid", true);
    prefs.end();
    Serial.println("[MAZE] Mapped maze saved to Flash NVS!");
}

bool Maze::loadFromNVS() {
    Preferences prefs;
    prefs.begin(MAZE_STORAGE, true);
    if (!prefs.getBool("valid", false)) {
        prefs.end();
        return false;
    }
    prefs.getBytes("votes", wall_votes_, sizeof(wall_votes_));
    prefs.getBytes("visited", visited_, sizeof(visited_));
    prefs.end();
    Serial.println("[MAZE] Mapped maze loaded from Flash NVS!");
    return true;
}

void Maze::clearNVS() {
    Preferences prefs;
    prefs.begin(MAZE_STORAGE, false);
    prefs.clear();
    prefs.end();
    Serial.println("[MAZE] Flash NVS maze cleared.");
}

bool Maze::hasSavedMaze() const {
    Preferences prefs;
    prefs.begin(MAZE_STORAGE, true);
    bool valid = prefs.getBool("valid", false);
    prefs.end();
    return valid;
}
