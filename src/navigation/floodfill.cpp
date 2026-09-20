#include "floodfill.h"

// Ring buffer queue for BFS floodfill
struct CellQueue {
    Coordinate buffer[MAZE_WIDTH * MAZE_HEIGHT];
    int head;
    int tail;

    void init() { head = 0; tail = 0; }
    bool isEmpty() const { return head == tail; }
    void push(Coordinate c) {
        buffer[tail] = c;
        tail = (tail + 1) % (MAZE_WIDTH * MAZE_HEIGHT);
    }
    Coordinate pop() {
        Coordinate c = buffer[head];
        head = (head + 1) % (MAZE_WIDTH * MAZE_HEIGHT);
        return c;
    }
};

Floodfill::Floodfill(const Maze& maze) : maze_(maze) {
    setGoalToCenter();
    recalculate();
}

void Floodfill::setGoalToCenter() {
    memset(is_goal_, 0, sizeof(is_goal_));
    // Standard 16x16 center cells
    is_goal_[7][7] = true;
    is_goal_[7][8] = true;
    is_goal_[8][7] = true;
    is_goal_[8][8] = true;
}

void Floodfill::setGoalToStart() {
    memset(is_goal_, 0, sizeof(is_goal_));
    is_goal_[0][0] = true;
}

bool Floodfill::isAtGoal(int8_t x, int8_t y) const {
    if (x < 0 || x >= MAZE_WIDTH || y < 0 || y >= MAZE_HEIGHT) return false;
    return is_goal_[x][y];
}

uint16_t Floodfill::getDistance(int8_t x, int8_t y) const {
    if (x < 0 || x >= MAZE_WIDTH || y < 0 || y >= MAZE_HEIGHT) return DIST_INFINITY;
    return distance_[x][y];
}

void Floodfill::recalculate() {
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

        // Check North
        if (!maze_.hasWall(curr.x, curr.y, DIR_NORTH)) {
            if (curr.y + 1 < MAZE_HEIGHT && distance_[curr.x][curr.y + 1] > next_dist) {
                distance_[curr.x][curr.y + 1] = next_dist;
                queue.push({curr.x, (int8_t)(curr.y + 1)});
            }
        }

        // Check East
        if (!maze_.hasWall(curr.x, curr.y, DIR_EAST)) {
            if (curr.x + 1 < MAZE_WIDTH && distance_[curr.x + 1][curr.y] > next_dist) {
                distance_[curr.x + 1][curr.y] = next_dist;
                queue.push({(int8_t)(curr.x + 1), curr.y});
            }
        }

        // Check South
        if (!maze_.hasWall(curr.x, curr.y, DIR_SOUTH)) {
            if (curr.y - 1 >= 0 && distance_[curr.x][curr.y - 1] > next_dist) {
                distance_[curr.x][curr.y - 1] = next_dist;
                queue.push({curr.x, (int8_t)(curr.y - 1)});
            }
        }

        // Check West
        if (!maze_.hasWall(curr.x, curr.y, DIR_WEST)) {
            if (curr.x - 1 >= 0 && distance_[curr.x - 1][curr.y] > next_dist) {
                distance_[curr.x - 1][curr.y] = next_dist;
                queue.push({(int8_t)(curr.x - 1), curr.y});
            }
        }
    }
}

Direction Floodfill::getNextDirection(int8_t current_x, int8_t current_y, Direction current_heading) {
    uint16_t min_dist = DIST_INFINITY;
    Direction best_dir = DIR_INVALID;

    // Order of checks: prioritize going straight to avoid turn penalties
    const int8_t turn_preference[4] = {0, 1, -1, 2}; // Straight, Right, Left, Turn Around

    for (int i = 0; i < 4; ++i) {
        Direction candidate_dir = Maze::getAbsoluteDirection(current_heading, turn_preference[i]);

        if (!maze_.hasWall(current_x, current_y, candidate_dir)) {
            int8_t nx = current_x;
            int8_t ny = current_y;

            if (candidate_dir == DIR_NORTH) ny++;
            else if (candidate_dir == DIR_EAST)  nx++;
            else if (candidate_dir == DIR_SOUTH) ny--;
            else if (candidate_dir == DIR_WEST)  nx--;

            if (nx >= 0 && nx < MAZE_WIDTH && ny >= 0 && ny < MAZE_HEIGHT) {
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
