#include "floodfill.h"

// Ring-buffer queue for high-performance BFS expansion
struct CellQueue {
    Coordinate buffer[TOTAL_CELLS];
    uint16_t head;
    uint16_t tail;

    void init() { head = 0; tail = 0; }
    bool isEmpty() const { return head == tail; }
    void push(Coordinate c) {
        buffer[tail] = c;
        tail = (tail + 1) % TOTAL_CELLS;
    }
    Coordinate pop() {
        Coordinate c = buffer[head];
        head = (head + 1) % TOTAL_CELLS;
        return c;
    }
};

Floodfill::Floodfill(const Maze& maze)
    : maze_(maze),
      known_edges_only_(false) {
    setGoalToCenter();
    recalculate();
}

void Floodfill::setGoalToCenter() {
    memset(is_goal_, 0, sizeof(is_goal_));
    // Standard 16x16 center square
    is_goal_[7][7] = true;
    is_goal_[7][8] = true;
    is_goal_[8][7] = true;
    is_goal_[8][8] = true;
}

void Floodfill::setGoalToStart() {
    memset(is_goal_, 0, sizeof(is_goal_));
    is_goal_[0][0] = true;
}

void Floodfill::setCustomGoal(int8_t x, int8_t y) {
    if (!Maze::isValidCoordinate(x, y)) return;
    memset(is_goal_, 0, sizeof(is_goal_));
    is_goal_[x][y] = true;
}

bool Floodfill::isAtGoal(int8_t x, int8_t y) const {
    if (!Maze::isValidCoordinate(x, y)) return false;
    return is_goal_[x][y];
}

uint16_t Floodfill::getDistance(int8_t x, int8_t y) const {
    if (!Maze::isValidCoordinate(x, y)) return DIST_INFINITY;
    return distance_[x][y];
}

void Floodfill::recalculate(bool known_edges_only) {
    known_edges_only_ = known_edges_only;
    // Reset all distances to infinity
    for (int8_t x = 0; x < MAZE_WIDTH; ++x) {
        for (int8_t y = 0; y < MAZE_HEIGHT; ++y) {
            distance_[x][y] = DIST_INFINITY;
        }
    }

    CellQueue queue;
    queue.init();

    // Push all goal cells with distance 0
    for (int8_t x = 0; x < MAZE_WIDTH; ++x) {
        for (int8_t y = 0; y < MAZE_HEIGHT; ++y) {
            if (is_goal_[x][y]) {
                distance_[x][y] = 0;
                queue.push({x, y});
            }
        }
    }

    // BFS wave expansion
    while (!queue.isEmpty()) {
        Coordinate curr = queue.pop();
        uint16_t next_dist = distance_[curr.x][curr.y] + 1;

        const Direction dirs[4] = {DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST};
        for (int i = 0; i < 4; ++i) {
            Direction d = dirs[i];
            if (!maze_.hasWall(curr.x, curr.y, d) &&
                (!known_edges_only_ || maze_.isWallKnown(curr.x, curr.y, d))) {
                int8_t nx = curr.x + dxFromDir(d);
                int8_t ny = curr.y + dyFromDir(d);
                if (Maze::isValidCoordinate(nx, ny) && distance_[nx][ny] > next_dist) {
                    distance_[nx][ny] = next_dist;
                    queue.push({nx, ny});
                }
            }
        }
    }
}

Direction Floodfill::getNextDirection(int8_t current_x, int8_t current_y, Direction current_heading) {
    uint16_t min_dist = DIST_INFINITY;
    Direction best_dir = DIR_INVALID;

    // Movement preference: Straight (0), Right (+1), Left (-1), Turn Around (+2)
    // Minimizes rotation maneuvers for faster run times
    const int8_t turn_preference[4] = {0, 1, -1, 2};

    for (int i = 0; i < 4; ++i) {
        Direction candidate_dir = relativeToAbsolute(current_heading, turn_preference[i]);

        if (!maze_.hasWall(current_x, current_y, candidate_dir) &&
            (!known_edges_only_ || maze_.isWallKnown(current_x, current_y, candidate_dir))) {
            int8_t nx = current_x + dxFromDir(candidate_dir);
            int8_t ny = current_y + dyFromDir(candidate_dir);

            if (Maze::isValidCoordinate(nx, ny)) {
                uint16_t d = distance_[nx][ny];
                if (d < min_dist) {
                    min_dist = d;
                    best_dir = candidate_dir;
                }
            }
        }
    }

    return best_dir;
}
