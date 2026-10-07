#include "navigation.h"

// ==============================================================================
// MAZE MAP
// ==============================================================================

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

// ==============================================================================
// FLOODFILL SOLVER
// ==============================================================================

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

// ==============================================================================
// DIJKSTRA SOLVER
// ==============================================================================

#include <string.h>

Dijkstra::Dijkstra(const Maze& maze) : maze_(maze) {}

static const int8_t DX[4] = { 0, 1, 0, -1 };
static const int8_t DY[4] = { 1, 0, -1, 0 };

static bool isCenterGoal(int8_t x, int8_t y) {
    return (x == 7 || x == 8) && (y == 7 || y == 8);
}

static bool isStartGoal(int8_t x, int8_t y) {
    return (x == 0 && y == 0);
}

uint8_t Dijkstra::findFastestPathToCenter(int8_t start_x, int8_t start_y, Direction start_h,
                                          Coordinate* out_path, uint8_t max_path_len) {
    return findFastestPath(start_x, start_y, start_h, isCenterGoal, out_path, max_path_len);
}

uint8_t Dijkstra::findFastestPathToStart(int8_t start_x, int8_t start_y, Direction start_h,
                                         Coordinate* out_path, uint8_t max_path_len) {
    return findFastestPath(start_x, start_y, start_h, isStartGoal, out_path, max_path_len);
}

// Simple Binary Min-Heap
class MinHeap {
public:
    MinHeap() : size_(0) { clear(); }

    void clear() {
        size_ = 0;
        for (uint16_t i = 0; i < STATE_COUNT; ++i) {
            positions_[i] = -1;
        }
    }

    bool isEmpty() const { return size_ == 0; }

    void pushOrDecrease(float cost, int8_t x, int8_t y, uint8_t h) {
        const uint16_t state = stateIndex(x, y, h);
        int16_t i = positions_[state];
        if (i < 0) {
            i = (int16_t)size_++;
            heap_[i] = { cost, x, y, h };
            positions_[state] = i;
        } else {
            if (cost >= heap_[i].cost) return;
            heap_[i].cost = cost;
        }

        siftUp(i);
    }

    Dijkstra::HeapNode pop() {
        Dijkstra::HeapNode root = heap_[0];
        positions_[stateIndex(root.x, root.y, root.h)] = -1;
        --size_;
        if (size_ > 0) {
            heap_[0] = heap_[size_];
            positions_[stateIndex(heap_[0].x, heap_[0].y, heap_[0].h)] = 0;
            siftDown(0);
        }
        return root;
    }

private:
    static constexpr uint16_t STATE_COUNT = 16 * 16 * 4;

    static uint16_t stateIndex(int8_t x, int8_t y, uint8_t h) {
        return (uint16_t)(((uint16_t)x * 16 + (uint16_t)y) * 4 + h);
    }

    void swapNodes(int16_t a, int16_t b) {
        Dijkstra::HeapNode tmp = heap_[a];
        heap_[a] = heap_[b];
        heap_[b] = tmp;
        positions_[stateIndex(heap_[a].x, heap_[a].y, heap_[a].h)] = a;
        positions_[stateIndex(heap_[b].x, heap_[b].y, heap_[b].h)] = b;
    }

    void siftUp(int16_t i) {
        while (i > 0) {
            int16_t parent = (i - 1) / 2;
            if (heap_[parent].cost <= heap_[i].cost) break;
            swapNodes(parent, i);
            i = parent;
        }
    }

    void siftDown(int16_t i) {
        for (;;) {
            int16_t left = 2 * i + 1;
            int16_t right = left + 1;
            int16_t smallest = i;
            if (left < (int16_t)size_ && heap_[left].cost < heap_[smallest].cost) smallest = left;
            if (right < (int16_t)size_ && heap_[right].cost < heap_[smallest].cost) smallest = right;
            if (smallest == i) break;
            swapNodes(i, smallest);
            i = smallest;
        }
    }

    Dijkstra::HeapNode heap_[STATE_COUNT];
    int16_t positions_[STATE_COUNT];
    uint16_t size_;
};

static MinHeap g_min_heap;
static float g_best_cost[16][16][4];
static Dijkstra::ParentEdge g_parent[16][16][4];
static bool g_finalized[16][16][4];

static void relaxState(float cost, int8_t x, int8_t y, uint8_t h,
                       const Dijkstra::ParentEdge& parent) {
    if (g_finalized[x][y][h] || cost >= g_best_cost[x][y][h]) return;

    g_best_cost[x][y][h] = cost;
    g_parent[x][y][h] = parent;
    g_min_heap.pushOrDecrease(cost, x, y, h);
}

uint8_t Dijkstra::findFastestPath(int8_t start_x, int8_t start_y, Direction start_h,
                                  bool (*is_goal_fn)(int8_t, int8_t),
                                  Coordinate* out_path, uint8_t max_path_len) {
    if (!out_path || !is_goal_fn || max_path_len < 2 ||
        start_x < 0 || start_x >= 16 || start_y < 0 || start_y >= 16 ||
        start_h > DIR_WEST) return 0;

    for (int x = 0; x < 16; ++x) {
        for (int y = 0; y < 16; ++y) {
            for (int h = 0; h < 4; ++h) {
                g_best_cost[x][y][h] = 1e9f;
                g_parent[x][y][h].has_parent = false;
                g_finalized[x][y][h] = false;
            }
        }
    }

    g_min_heap.clear();
    g_best_cost[start_x][start_y][start_h] = 0.0f;
    g_min_heap.pushOrDecrease(0.0f, start_x, start_y, (uint8_t)start_h);

    int8_t goal_state_x = -1;
    int8_t goal_state_y = -1;
    uint8_t goal_state_h = 0;
    bool goal_found = false;

    while (!g_min_heap.isEmpty()) {
        HeapNode node = g_min_heap.pop();
        int8_t x = node.x;
        int8_t y = node.y;
        uint8_t h = node.h;
        float cost = node.cost;

        if (g_finalized[x][y][h] || cost > g_best_cost[x][y][h]) continue;
        g_finalized[x][y][h] = true;

        if (is_goal_fn(x, y)) {
            goal_state_x = x;
            goal_state_y = y;
            goal_state_h = h;
            goal_found = true;
            break;
        }

        // 1. In-place turn (90 deg left / right)
        for (int td : { 1, 3 }) {
            uint8_t nh = (h + td) % 4;
            float tc = cost + 1.5f;
            ParentEdge parent = { x, y, h, 0, 0, 0, true };
            relaxState(tc, x, y, nh, parent);
        }

        // 2. Straight sprint (1 to 15 cells)
        int8_t dx = DX[h];
        int8_t dy = DY[h];
        for (uint8_t L = 1; L <= 15; ++L) {
            int8_t px = x + dx * (L - 1);
            int8_t py = y + dy * (L - 1);
            if (maze_.hasWall(px, py, (Direction)h)) break;

            int8_t nx = x + dx * L;
            int8_t ny = y + dy * L;
            if (nx < 0 || nx >= 16 || ny < 0 || ny >= 16) break;
            if (!maze_.isVisited(nx, ny)) break;

            float sprint_cost = cost + 1.0f + 0.35f * (float)(L - 1);
            ParentEdge parent = { x, y, h, 1, L, 0, true };
            relaxState(sprint_cost, nx, ny, h, parent);
        }

        // 3. Big Diagonal Staircase (M >= 2)
        for (int td : { 1, 3 }) {
            uint8_t d1 = (h + td) % 4;
            int8_t cur_x = x;
            int8_t cur_y = y;

            for (uint8_t m_step = 1; m_step <= 15; ++m_step) {
                uint8_t s_dir = (m_step % 2 == 1) ? h : d1;
                if (maze_.hasWall(cur_x, cur_y, (Direction)s_dir)) break;

                cur_x += DX[s_dir];
                cur_y += DY[s_dir];
                if (cur_x < 0 || cur_x >= 16 || cur_y < 0 || cur_y >= 16) break;
                if (!maze_.isVisited(cur_x, cur_y)) break;

                if (m_step >= 2) {
                    float diag_cost = cost + 1.8f + (float)(m_step - 1) * 0.5f;
                    uint8_t end_h = s_dir;
                    ParentEdge parent = { x, y, h, 2, m_step, d1, true };
                    relaxState(diag_cost, cur_x, cur_y, end_h, parent);
                }
            }
        }

        // 4. Up-and-Down Slalom Zigzag (Length >= 5, odd)
        for (int td : { 1, 3 }) {
            uint8_t d_c1 = (h + td) % 4;
            uint8_t d_c2 = (d_c1 + 2) % 4;
            int8_t cur_x = x;
            int8_t cur_y = y;

            for (uint8_t m_idx = 0; m_idx < 15; ++m_idx) {
                uint8_t s_dir;
                if (m_idx % 2 == 0) {
                    s_dir = h;
                } else {
                    s_dir = ((m_idx / 2) % 2 == 0) ? d_c1 : d_c2;
                }

                if (maze_.hasWall(cur_x, cur_y, (Direction)s_dir)) break;

                cur_x += DX[s_dir];
                cur_y += DY[s_dir];
                if (cur_x < 0 || cur_x >= 16 || cur_y < 0 || cur_y >= 16) break;
                if (!maze_.isVisited(cur_x, cur_y)) break;

                uint8_t m_count = m_idx + 1;
                if (m_count >= 5 && m_count % 2 == 1) {
                    uint8_t num_w = (m_count - 1) / 2;
                    float slalom_cost = cost + 1.5f + (float)num_w * 1.2f + 0.5f;
                    uint8_t end_h = h;
                    ParentEdge parent = { x, y, h, 3, m_count, d_c1, true };
                    relaxState(slalom_cost, cur_x, cur_y, end_h, parent);
                }
            }
        }
    }

    if (!goal_found) return 0;

    // Backtrack to reconstruct full cell path
    static Coordinate temp_path[256];
    uint16_t temp_len = 0;

    int8_t curr_x = goal_state_x;
    int8_t curr_y = goal_state_y;
    uint8_t curr_h = goal_state_h;
    uint16_t backtrack_steps = 0;

    while (g_parent[curr_x][curr_y][curr_h].has_parent && backtrack_steps++ < 1024) {
        ParentEdge edge = g_parent[curr_x][curr_y][curr_h];
        if (edge.move_type == 0) {
            // In-place turn (no spatial change)
        } else if (edge.move_type == 1) {
            // Straight sprint
            int8_t dx = DX[edge.prev_h];
            int8_t dy = DY[edge.prev_h];
            for (int s = edge.count; s >= 1; --s) {
                if (temp_len >= 256) return 0;
                temp_path[temp_len++] = { (int8_t)(edge.prev_x + dx * s), (int8_t)(edge.prev_y + dy * s) };
            }
        } else if (edge.move_type == 2) {
            // Big Diagonal Staircase
            static Coordinate d_cells[16];
            int8_t cx = edge.prev_x;
            int8_t cy = edge.prev_y;
            for (uint8_t s = 1; s <= edge.count; ++s) {
                uint8_t s_dir = (s % 2 == 1) ? edge.prev_h : edge.d_cross;
                cx += DX[s_dir];
                cy += DY[s_dir];
                d_cells[s - 1] = { cx, cy };
            }
            for (int s = edge.count - 1; s >= 0; --s) {
                if (temp_len >= 256) return 0;
                temp_path[temp_len++] = d_cells[s];
            }
        } else if (edge.move_type == 3) {
            // Up-and-Down Slalom Zigzag
            static Coordinate s_cells[16];
            int8_t cx = edge.prev_x;
            int8_t cy = edge.prev_y;
            uint8_t d_c1 = edge.d_cross;
            uint8_t d_c2 = (d_c1 + 2) % 4;
            for (uint8_t s = 0; s < edge.count; ++s) {
                uint8_t s_dir = (s % 2 == 0) ? edge.prev_h : (((s / 2) % 2 == 0) ? d_c1 : d_c2);
                cx += DX[s_dir];
                cy += DY[s_dir];
                s_cells[s] = { cx, cy };
            }
            for (int s = edge.count - 1; s >= 0; --s) {
                if (temp_len >= 256) return 0;
                temp_path[temp_len++] = s_cells[s];
            }
        } else {
            return 0;
        }

        curr_x = edge.prev_x;
        curr_y = edge.prev_y;
        curr_h = edge.prev_h;
    }

    if (backtrack_steps > 1024 || curr_x != start_x || curr_y != start_y ||
        curr_h != (uint8_t)start_h || temp_len + 1 > max_path_len) return 0;

    // Add start cell
    temp_path[temp_len++] = { start_x, start_y };

    // Reverse into out_path
    uint8_t path_len = 0;
    for (int i = (int)temp_len - 1; i >= 0; --i) {
        out_path[path_len++] = temp_path[i];
    }

    return path_len;
}

// ==============================================================================
// PATH HELPERS
// ==============================================================================

Direction directionBetween(Coordinate from, Coordinate to) {
    if (to.y > from.y) return DIR_NORTH;
    if (to.x > from.x) return DIR_EAST;
    if (to.y < from.y) return DIR_SOUTH;
    if (to.x < from.x) return DIR_WEST;
    return DIR_INVALID;
}

// ==============================================================================
// NAVIGATOR
// ==============================================================================

Navigator::Navigator(QueueHandle_t motion_cmd_queue, QueueHandle_t telemetry_queue)
    : motion_cmd_queue_(motion_cmd_queue),
      telemetry_queue_(telemetry_queue),
      floodfill_(maze_),
      dijkstra_(maze_),
      state_(NAV_STATE_IDLE),
      current_strategy_(SPEEDRUN_HYBRID_AUTO),
      waiting_for_motion_(false),
      current_search_speed_(0.0f),
      speed_scale_(1.0f), best_route_explored_(false), maze_changed_(false), map_reset_this_run_(false),
      search_phase_(PHASE_AT_CENTRE),
      curve_cell_x_(0), curve_cell_y_(0), curve_entry_dir_(DIR_NORTH), curve_cell_was_known_(false),
      sub_cmd_count_(0),
      sub_cmd_idx_(0),
      sub_cmd_overflow_(false) {
    memset(&pose_, 0, sizeof(pose_));
    pose_.cell_x = 0;
    pose_.cell_y = 0;
    pose_.current_dir = DIR_NORTH;
}

void Navigator::begin() {
    maze_.reset();
    if (maze_.loadFromNVS()) {
        Serial.println("[NAV] Found previously saved maze in Flash! Loaded successfully for instant Speedrun.");
    }
    floodfill_.setGoalToCenter();
    floodfill_.recalculate();
    state_ = NAV_STATE_IDLE;
    waiting_for_motion_ = false;
    current_search_speed_ = 0.0f;
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;
}

void Navigator::startSearchRun() {
    // The map is NOT wiped: everything learned in earlier attempts (including ones that ended in
    // a crash) is kept, so each search starts from what is already known and explores further.
    map_reset_this_run_ = false;
    maze_.setVisited(0, 0); // Start cell (0,0) is visited
    floodfill_.setGoalToCenter();
    floodfill_.recalculate();
    pose_.cell_x = 0;
    pose_.cell_y = 0;
    pose_.current_dir = DIR_NORTH;
    state_ = NAV_STATE_EXPLORING_TO_CENTER;
    search_phase_ = PHASE_AT_CENTRE;
    waiting_for_motion_ = false;
    current_search_speed_ = 0.0f;
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;
    Serial.println("[NAV] Starting search run to the maze centre (keeping the known map).");
}

void Navigator::saveMazeIfChanged() {
    // Writing flash briefly stalls the processor, so this is only called while standing still
    if (maze_changed_) {
        maze_.saveToNVS();
        maze_changed_ = false;
    }
}

// Follows the floodfill from the start cell to the centre with unexplored cells treated as open.
// If that route never leaves explored cells, no shorter route can be hiding in the unknown.
bool Navigator::checkBestRouteExplored() {
    int8_t x = 0, y = 0;
    Direction heading = DIR_NORTH;
    for (int i = 0; i < 255; ++i) {
        if (!maze_.isVisited(x, y)) return false;
        if (floodfill_.isAtGoal(x, y)) return true;
        Direction next = floodfill_.getNextDirection(x, y, heading);
        if (next == DIR_INVALID) return false;
        heading = next;
        if (next == DIR_NORTH) y++;
        else if (next == DIR_EAST)  x++;
        else if (next == DIR_SOUTH) y--;
        else if (next == DIR_WEST)  x--;
    }
    return false;
}

// Time a straight of `dist` mm takes when entered at v0 and left at v1, never faster than v_max
static float moveTime(float dist, float v_max, float accel, float v0, float v1) {
    if (dist <= 0.0f) return 0.0f;
    float d_up   = (v_max * v_max - v0 * v0) / (2.0f * accel);
    float d_down = (v_max * v_max - v1 * v1) / (2.0f * accel);
    if (d_up + d_down > dist) {
        // Too short to reach v_max: accelerate to a lower peak, then brake
        float v_peak = sqrtf((2.0f * accel * dist + v0 * v0 + v1 * v1) * 0.5f);
        v_peak = fmaxf(v_peak, fmaxf(v0, v1));
        return (v_peak - v0) / accel + (v_peak - v1) / accel;
    }
    return (v_max - v0) / accel + (v_max - v1) / accel + (dist - d_up - d_down) / v_max;
}

// Plans a whole speed run: turns the cell path into one unbroken chain of moves.
//
// The path is read as a list of turns, one per cell the path bends in:
//   a turn on its own                 -> smooth 90° curve, cell edge to cell edge
//   turns that alternate (L R L R..)  -> one diagonal, entered and left with smooth 45° curves
//   two turns the same way in a row   -> the diagonal doubles back: a smooth 90° "V" turn from
//       inside a diagonal                one diagonal onto the next
// The robot never stops between the start cell and the centre (unless the path reverses on
// itself, which a shortest path does not do).
//
// Distances: `pending` is straight-line distance still to be driven before the next turn. Every
// cell step adds 180 mm; each turn takes over part of the steps either side of it.
float Navigator::planSpeedRun(const Coordinate* path, uint8_t path_len, bool use_diagonals) {
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;
    sub_cmd_overflow_ = false;

    const float cruise     = SPEEDRUN_CRUISE_SPEED_MM_S * speed_scale_;
    const float accel      = SPEEDRUN_ACCEL_MM_S2       * speed_scale_;
    const float diag_speed = SPEEDRUN_DIAG_SPEED_MM_S   * speed_scale_;
    const float turn_speed = SPEEDRUN_CURVE_SPEED_MM_S  * speed_scale_;
    const float v_speed    = turn_speed * V90_SPEED_RATIO;
    const float spin_speed = SPEEDRUN_TURN_SPEED_DEG_S  * speed_scale_;
    const float spin_accel = SPEEDRUN_TURN_ACCEL_DEG_S2 * speed_scale_;

    // Every run begins in the start cell facing into the maze
    pose_.cell_x = path[0].x;
    pose_.cell_y = path[0].y;
    pose_.current_dir = DIR_NORTH;

    static Direction dirs[256];
    const uint8_t steps = path_len - 1;
    for (uint8_t i = 0; i < steps; ++i) {
        dirs[i] = directionBetween(path[i], path[i + 1]);
    }
    // The turn made in cell i: 0 = straight on, 1 = right, 3 = left, 2 = back the way it came
    auto turnAt = [&](uint8_t i) -> int8_t { return (int8_t)(((int8_t)dirs[i] - (int8_t)dirs[i - 1] + 4) % 4); };

    float time_s  = 0.0f;
    float pending = 0.0f; // Straight distance not yet turned into a move
    float speed   = 0.0f; // Speed of the robot at the point reached so far

    // Emits the pending straight, ending at `exit_speed`
    auto driveStraight = [&](float exit_speed) {
        if (pending > 1.0f) {
            pushSubCommand(ACTION_MOVE_DISTANCE, pending, cruise, accel, true, speed, exit_speed);
            time_s += moveTime(pending, cruise, accel, speed, exit_speed);
        }
        pending = 0.0f;
        speed = exit_speed;
    };
    auto driveCurve = [&](MotionAction action, float length, float v) {
        pushSubCommand(action, length, v, accel, false, v, v);
        time_s += length / v;
        speed = v;
    };
    auto spinOnSpot = [&](MotionAction action, float degrees) {
        pushSubCommand(action, degrees, spin_speed, spin_accel, false);
        time_s += degrees / spin_speed + spin_speed / spin_accel;
    };

    // Face the first step (normally it already does)
    int8_t first_turn = ((int8_t)dirs[0] - (int8_t)pose_.current_dir + 4) % 4;
    if (first_turn == 1) spinOnSpot(ACTION_TURN_RIGHT_90, 90.0f);
    if (first_turn == 3) spinOnSpot(ACTION_TURN_LEFT_90, 90.0f);
    if (first_turn == 2) spinOnSpot(ACTION_TURN_AROUND_180, 180.0f);

    pending = MAZE_CELL_SIZE_MM; // The first step, start cell centre to the next cell centre
    uint8_t i = 1;
    while (i < steps) {
        const int8_t turn = turnAt(i);

        if (turn == 0) { // Straight on through cell i
            pending += MAZE_CELL_SIZE_MM;
            i++;
            continue;
        }
        if (turn == 2) { // The path reverses: stop at the cell centre and turn around
            driveStraight(0.0f);
            spinOnSpot(ACTION_TURN_AROUND_180, 180.0f);
            pending = MAZE_CELL_SIZE_MM;
            i++;
            continue;
        }

        // Cells i..j: a run of left / right turns with no straight cell in between
        uint8_t j = i;
        bool alternates = false;
        while (j + 1 < steps && (turnAt(j + 1) == 1 || turnAt(j + 1) == 3)) {
            if (turnAt(j + 1) != turnAt(j)) alternates = true;
            j++;
        }

        if (!use_diagonals || !alternates) {
            // Smooth 90° curves, one per cell, each from the cell's entry edge to its exit edge
            for (uint8_t k = i; k <= j; ++k) {
                pending -= HALF_CELL_SIZE_MM;
                driveStraight(turn_speed);
                driveCurve(turnAt(k) == 1 ? ACTION_CURVE_RIGHT_90 : ACTION_CURVE_LEFT_90, CURVE_90_LENGTH_MM, turn_speed);
                pending = HALF_CELL_SIZE_MM; // From the exit edge to the next cell centre
            }
            i = j + 1;
            continue;
        }

        // Diagonal: leave the cell centreline with a 45° curve...
        pending += DIAG_LEAD_MM - MAZE_CELL_SIZE_MM;
        driveStraight(turn_speed);
        driveCurve(turnAt(i) == 1 ? ACTION_CURVE_RIGHT_45 : ACTION_CURVE_LEFT_45, CURVE_45_LENGTH_MM, turn_speed);

        // ...then one diagonal straight per stretch of alternating turns, with a V turn wherever
        // two turns in a row go the same way...
        float trim_in = DIAG_TRIM_MM;
        uint8_t k = i;
        while (k <= j) {
            uint8_t m = k;
            while (m < j && turnAt(m + 1) != turnAt(m)) m++;

            const bool last_stretch = (m == j);
            const float trim_out   = last_stretch ? DIAG_TRIM_MM : V90_TRIM_MM;
            const float exit_speed = last_stretch ? turn_speed : v_speed;
            const float length     = (float)(m - k + 1) * DIAG_HALF_STEP_MM - trim_in - trim_out;

            pushSubCommand(ACTION_MOVE_DIAGONAL_HALF, length / DIAG_HALF_STEP_MM, diag_speed, accel, false, speed, exit_speed);
            time_s += moveTime(length, diag_speed, accel, speed, exit_speed);
            speed = exit_speed;

            if (!last_stretch) {
                driveCurve(turnAt(m) == 1 ? ACTION_CURVE_RIGHT_90 : ACTION_CURVE_LEFT_90, CURVE_V90_LENGTH_MM, v_speed);
                trim_in = V90_TRIM_MM;
            }
            k = m + 1;
        }

        // ...and back onto a cell centreline with another 45° curve
        driveCurve(turnAt(j) == 1 ? ACTION_CURVE_RIGHT_45 : ACTION_CURVE_LEFT_45, CURVE_45_LENGTH_MM, turn_speed);
        pending = DIAG_LEAD_MM; // From the end of that curve to the next cell centre
        i = j + 1;
    }

    driveStraight(0.0f); // Into the last cell, and stop

    pose_.cell_x = path[steps].x;
    pose_.cell_y = path[steps].y;
    pose_.current_dir = dirs[steps - 1];

    return sub_cmd_overflow_ ? -1.0f : time_s;
}

void Navigator::startSpeedRun(SpeedrunStrategy strategy) {
    current_strategy_ = strategy;
    state_ = NAV_STATE_SPEED_RUNNING;
    waiting_for_motion_ = false;

    static Coordinate path[256];
    uint8_t path_len = dijkstra_.findFastestPathToCenter(0, 0, DIR_NORTH, path, 255);
    if (path_len < 2) {
        Serial.println("[NAV] Error: No speedrun path found!");
        state_ = NAV_STATE_ERROR;
        return;
    }

    bool use_diagonals = (strategy != SPEEDRUN_CURVES_ONLY);
    if (strategy == SPEEDRUN_HYBRID_AUTO) {
        // Plan it both ways and keep whichever is quicker
        float t_curves = planSpeedRun(path, path_len, false);
        float t_diags  = planSpeedRun(path, path_len, true);
        use_diagonals = (t_diags >= 0.0f) && (t_curves < 0.0f || t_diags < t_curves);
        Serial.printf("[NAV] Hybrid: curves %.2f s, diagonals %.2f s -> %s\n",
                      t_curves, t_diags, use_diagonals ? "diagonals" : "curves");
    }

    float run_time = planSpeedRun(path, path_len, use_diagonals);
    if (run_time < 0.0f) {
        Serial.println("[NAV] Error: speed run has too many moves to plan!");
        sub_cmd_count_ = 0;
        state_ = NAV_STATE_ERROR;
        return;
    }
    Serial.printf("[NAV] Speed run planned: %d cells, %d moves, about %.2f s.\n",
                  (int)path_len, (int)sub_cmd_count_, run_time);

    processSubcommandQueue();
}

void Navigator::stop() {
    // Keep what this run learned, even if it ended badly: the next search carries on from it
    if (state_ == NAV_STATE_EXPLORING_TO_CENTER || state_ == NAV_STATE_RETURNING_TO_START) {
        saveMazeIfChanged();
    }
    state_ = NAV_STATE_IDLE;
    search_phase_ = PHASE_AT_CENTRE;
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;
    sendMotionCommand(ACTION_EMERGENCY_STOP, 0.0f, 0.0f, 0.0f);
}

void Navigator::notifyMotionComplete() {
    waiting_for_motion_ = false;
    processSubcommandQueue();
}

void Navigator::sendMotionCommand(MotionAction action, float param, float max_speed, float accel,
                                  bool wall_centering, float entry_speed, float exit_speed,
                                  bool stop_at_front_wall) {
    MotionCommand cmd;
    cmd.action = action;
    cmd.param_value = param;
    cmd.max_speed_mm_s = max_speed;
    cmd.acceleration = accel;
    cmd.enable_wall_centering = wall_centering;
    cmd.entry_speed_mm_s = entry_speed;
    cmd.exit_speed_mm_s = exit_speed;
    cmd.start_offset_mm = 0.0f;
    cmd.stop_at_front_wall = stop_at_front_wall;

    xQueueSend(motion_cmd_queue_, &cmd, portMAX_DELAY);
    waiting_for_motion_ = true;
}

void Navigator::processSubcommandQueue() {
    if (sub_cmd_idx_ < sub_cmd_count_) {
        // Send next sub-command in current segment
        MotionCommand next_cmd = sub_cmd_queue_[sub_cmd_idx_++];
        xQueueSend(motion_cmd_queue_, &next_cmd, portMAX_DELAY);
        waiting_for_motion_ = true;
        return;
    }

    // Nothing left to send: a speed run ends here (the search carries on in step())
    if (state_ == NAV_STATE_SPEED_RUNNING) {
        Serial.println("[NAV] Speed run complete: centre reached.");
        state_ = NAV_STATE_FINISHED;
    }
}

void Navigator::pushSubCommand(MotionAction action, float param, float max_speed, float accel,
                               bool wall_centering, float entry_speed, float exit_speed) {
    if (sub_cmd_count_ >= MAX_SUB_CMDS) {
        sub_cmd_overflow_ = true;
        return;
    }
    sub_cmd_queue_[sub_cmd_count_++] = { action, param, max_speed, accel, wall_centering, entry_speed, exit_speed, 0.0f, false };
}

// Search run: one decision per cell, taken at the cell centre where the wall sensors are reliable.
// The same stepping drives both legs: out to the centre, then back to the start cell. Unexplored
// cells count as open on both legs, so the way back naturally explores new ground.
//
// The robot rolls straight through cells without stopping whenever the way ahead is clear:
// at full search speed through cells it already knows, at SEARCH_PROBE_SPEED into cells it has
// never seen (so it can stop within a few millimetres if a wall turns up). It stops and turns on
// the spot only where the path actually turns.
void Navigator::step(const IRReadings& ir, const WallPreview& preview) {
    if (waiting_for_motion_ ||
        (state_ != NAV_STATE_EXPLORING_TO_CENTER && state_ != NAV_STATE_RETURNING_TO_START)) {
        return;
    }

    // Part-way through a look-ahead move: the robot is on a cell edge, not a centre
    if (search_phase_ == PHASE_AT_EDGE)     { stepAtCellEdge(ir, preview); return; }
    if (search_phase_ == PHASE_AFTER_CURVE) { stepAfterCurve(preview);     return; }

    const float search_speed = SEARCH_SPEED_DEFAULT_MM_S;
    const float search_accel = SEARCH_ACCEL_DEFAULT_MM_S2;
    const float turn_speed   = SEARCH_TURN_SPEED_DEG_S;
    const float turn_accel   = SEARCH_TURN_ACCEL_DEG_S2;

    // 1. Update walls in current cell based on reliable 90° and front sensor readings
    if (!maze_.isVisited(pose_.cell_x, pose_.cell_y)) maze_changed_ = true;
    maze_.updateCellWalls(pose_.cell_x, pose_.cell_y, pose_.current_dir,
                          ir.wall_left, ir.wall_front, ir.wall_right);

    // A move that rolled in at speed comes to rest by itself if it met a wall ahead
    // (MotionCommand::stop_at_front_wall), so with a wall in front the robot is standing still.
    if (ir.wall_front) current_search_speed_ = 0.0f;

    // 2. Work out which way to go next
    const bool at_goal = floodfill_.isAtGoal(pose_.cell_x, pose_.cell_y);
    static Coordinate path[256];
    uint8_t path_len = 1;
    int8_t diff = 0;
    Direction d0 = pose_.current_dir;

    if (!at_goal) {
        floodfill_.recalculate();

        path[0] = { pose_.cell_x, pose_.cell_y };
        int8_t cur_x = pose_.cell_x;
        int8_t cur_y = pose_.cell_y;
        Direction cur_h = pose_.current_dir;

        for (int step_i = 0; step_i < 254; ++step_i) {
            Direction nd = floodfill_.getNextDirection(cur_x, cur_y, cur_h);
            if (nd == DIR_INVALID) break;
            cur_h = nd;
            if (nd == DIR_NORTH) cur_y++;
            else if (nd == DIR_EAST)  cur_x++;
            else if (nd == DIR_SOUTH) cur_y--;
            else if (nd == DIR_WEST)  cur_x--;
            path[path_len++] = { cur_x, cur_y };
            if (floodfill_.isAtGoal(cur_x, cur_y)) break;
        }

        if (path_len < 2) {
            // Walled in. If that happens in the start cell before moving, the remembered map must
            // be wrong (it came from a run that went badly): forget it once and start afresh.
            const bool at_start = (pose_.cell_x == 0 && pose_.cell_y == 0);
            if (at_start && state_ == NAV_STATE_EXPLORING_TO_CENTER && !map_reset_this_run_) {
                Serial.println("[NAV] Remembered map has no way out of the start cell. Forgetting it and starting afresh.");
                map_reset_this_run_ = true;
                maze_.reset();
                maze_.setVisited(0, 0);
                floodfill_.recalculate();
                step(ir, preview);
                return;
            }
            Serial.println("[NAV] Error: Trapped! No valid paths.");
            state_ = NAV_STATE_ERROR;
            return;
        }

        d0 = directionBetween({ pose_.cell_x, pose_.cell_y }, path[1]);
        diff = ((int8_t)d0 - (int8_t)pose_.current_dir + 4) % 4;
    }

    // 3. Anything other than "straight on" happens from a standstill at the cell centre. If the
    // robot rolled in at speed, brake, then back up the few millimetres it overshot, so that
    // turning on the spot never leaves it off-centre. Then decide again.
    if ((at_goal || diff != 0) && current_search_speed_ > 0.0f) {
        float v = current_search_speed_;
        float brake_mm = (v * v) / (2.0f * search_accel) + 2.0f;
        sub_cmd_count_ = 0;
        sub_cmd_idx_ = 0;
        pushSubCommand(ACTION_MOVE_DISTANCE,  brake_mm, v, search_accel, false, v, 0.0f);
        pushSubCommand(ACTION_MOVE_DISTANCE, -brake_mm, SEARCH_PROBE_SPEED_MM_S, search_accel, false);
        processSubcommandQueue();
        current_search_speed_ = 0.0f;
        return;
    }

    if (at_goal) {
        saveMazeIfChanged();

        if (state_ == NAV_STATE_EXPLORING_TO_CENTER) {
            // Centre reached: now explore back to the start cell
            Serial.printf("[NAV] Centre reached at (%d, %d). Exploring back to the start.\n", pose_.cell_x, pose_.cell_y);
            floodfill_.setGoalToStart();
            floodfill_.recalculate();
            state_ = NAV_STATE_RETURNING_TO_START;
            step(ir, preview);
            return;
        }

        // Back in the start cell: the search is over
        floodfill_.setGoalToCenter();
        floodfill_.recalculate();
        best_route_explored_ = checkBestRouteExplored();
        state_ = NAV_STATE_PREPARING_SPEED_RUN;
        Serial.println(best_route_explored_
            ? "[NAV] Search complete. The shortest possible route is fully explored: ready for speed runs."
            : "[NAV] Search complete. A shorter route may still be hiding in unexplored cells: search again to look for it.");
        return;
    }

    if (diff == 0) {
        // --- STRAIGHT ON: one cell forward, without stopping if the plan keeps going straight ---
        Coordinate next_cell = path[1];
        float exit_v = 0.0f;
        bool plan_continues_straight = path_len >= 3 &&
                                       directionBetween(path[1], path[2]) == d0 &&
                                       !floodfill_.isAtGoal(next_cell.x, next_cell.y);
        if (plan_continues_straight) {
            exit_v = maze_.isVisited(next_cell.x, next_cell.y) ? search_speed : SEARCH_PROBE_SPEED_MM_S;
        }

#if ENABLE_SEARCH_LOOKAHEAD
        // Look-ahead: if the next cell is new, or the plan turns there, drive only as far as its
        // edge. By then the 45° sensors have seen its side walls, and stepAtCellEdge() can curve
        // straight through it instead of stopping at its centre to turn on the spot.
        const bool next_is_goal  = floodfill_.isAtGoal(next_cell.x, next_cell.y);
        const bool next_is_known = maze_.isVisited(next_cell.x, next_cell.y);
        if (!next_is_goal && (!next_is_known || !plan_continues_straight)) {
            float edge_v = next_is_known ? SEARCH_CURVE_SPEED_MM_S : SEARCH_PROBE_SPEED_MM_S;
            sendMotionCommand(ACTION_MOVE_DISTANCE, HALF_CELL_SIZE_MM, search_speed, search_accel, true,
                              current_search_speed_, edge_v);
            current_search_speed_ = edge_v;
            pose_.cell_x = next_cell.x;
            pose_.cell_y = next_cell.y;
            search_phase_ = PHASE_AT_EDGE;
            return;
        }
#endif
        sendMotionCommand(ACTION_MOVE_FORWARD_CELLS, 1.0f, search_speed, search_accel, true,
                          current_search_speed_, exit_v, true);
        current_search_speed_ = exit_v;
        pose_.cell_x = next_cell.x;
        pose_.cell_y = next_cell.y;

    } else if (diff == 1 || diff == 3) {
        saveMazeIfChanged();

        // --- TURN: on the spot at the cell centre. The robot stays in this cell; the next step
        // re-reads the walls facing the new way and then drives straight on.
        sendMotionCommand((diff == 1) ? ACTION_TURN_RIGHT_90 : ACTION_TURN_LEFT_90,
                          90.0f, turn_speed, turn_accel, false);
        pose_.current_dir = d0;

    } else {
        saveMazeIfChanged();

        // --- DEAD END: OPTICAL FRONT SQUARING + 180° TURNAROUND ---
        Serial.printf("[NAV] Dead end reached at (%d, %d). Squaring optically against front wall...\n",
                      pose_.cell_x, pose_.cell_y);

        sub_cmd_count_ = 0;
        sub_cmd_idx_ = 0;

        // 1. Optically square against front wall using FL and FR sensor symmetry
        pushSubCommand(ACTION_SQUARE_FRONT_OPTICAL, 0.0f, 0.0f, 0.0f, false);

        // 2. High-precision 180° turnaround from a freshly zeroed heading baseline
        pushSubCommand(ACTION_TURN_AROUND_180, 180.0f, turn_speed, turn_accel, false);

        processSubcommandQueue();
        pose_.current_dir = d0;
    }
}

// The robot is on the entry edge of pose_.cell, still rolling. Curve through the cell if the path
// turns there and the turn is certain to be clear; otherwise carry on to its centre and decide
// there in the normal way.
void Navigator::stepAtCellEdge(const IRReadings& ir, const WallPreview& preview) {
    const int8_t x = pose_.cell_x;
    const int8_t y = pose_.cell_y;
    const Direction heading = pose_.current_dir;
    const Direction left  = Maze::getAbsoluteDirection(heading, -1);
    const Direction right = Maze::getAbsoluteDirection(heading, 1);

    const bool known = maze_.isVisited(x, y);
    bool sides_certain = known;

    if (!known) {
        // Two different sensors must agree about each side: the 45° sensor on the way here, and
        // the 90° sensor now that the robot's nose is inside the cell.
        const bool left_wall  = preview.left_wall  && ir.wall_left;
        const bool left_open  = preview.left_open  && !ir.wall_left;
        const bool right_wall = preview.right_wall && ir.wall_right;
        const bool right_open = preview.right_open && !ir.wall_right;
        sides_certain = (left_wall || left_open) && (right_wall || right_open);

        if (sides_certain) {
            if (left_wall)  maze_.setWall(x, y, left);
            if (right_wall) maze_.setWall(x, y, right);
            floodfill_.recalculate();
        }
    }

    // The front wall is not known yet, so the floodfill treats it as open. If turning is still the
    // best way out of this cell, it is the best way whatever the front wall turns out to be.
    const Direction best = sides_certain ? floodfill_.getNextDirection(x, y, heading) : DIR_INVALID;

    if (best == left || best == right) {
        const float v = current_search_speed_;
        curve_cell_x_ = x;
        curve_cell_y_ = y;
        curve_entry_dir_ = heading;
        curve_cell_was_known_ = known;

        sendMotionCommand((best == right) ? ACTION_CURVE_RIGHT_90 : ACTION_CURVE_LEFT_90,
                          CURVE_90_LENGTH_MM, v, SEARCH_ACCEL_DEFAULT_MM_S2, false, v, SEARCH_PROBE_SPEED_MM_S);
        current_search_speed_ = SEARCH_PROBE_SPEED_MM_S;

        // The curve ends on the entry edge of the neighbouring cell
        pose_.current_dir = best;
        if (best == DIR_NORTH) pose_.cell_y++;
        else if (best == DIR_EAST)  pose_.cell_x++;
        else if (best == DIR_SOUTH) pose_.cell_y--;
        else if (best == DIR_WEST)  pose_.cell_x--;
        search_phase_ = PHASE_AFTER_CURVE;
        return;
    }

    driveToCellCentre();
}

// The curve through (curve_cell_x_, curve_cell_y_) has finished. Half-way round it the outer 45°
// sensor was facing that cell's front wall, which completes what is known about the cell.
void Navigator::stepAfterCurve(const WallPreview& preview) {
    if (!curve_cell_was_known_) {
        if (preview.front_wall) {
            maze_.setWall(curve_cell_x_, curve_cell_y_, curve_entry_dir_);
        }
        // Only count the cell as explored if the front wall reading was clear either way;
        // otherwise it stays unexplored and the speed run will not be routed through it on trust.
        if (preview.front_wall || preview.front_open) {
            maze_.setVisited(curve_cell_x_, curve_cell_y_);
            maze_changed_ = true;
        }
    }
    driveToCellCentre();
}

// Second half of a look-ahead move: from the entry edge of pose_.cell to its centre, where the
// next decision is taken with all the sensors in the normal way.
void Navigator::driveToCellCentre() {
    const Direction best = floodfill_.getNextDirection(pose_.cell_x, pose_.cell_y, pose_.current_dir);
    const bool keep_rolling = (best == pose_.current_dir) && !floodfill_.isAtGoal(pose_.cell_x, pose_.cell_y);
    const float exit_v = keep_rolling ? SEARCH_PROBE_SPEED_MM_S : 0.0f;

    sendMotionCommand(ACTION_MOVE_DISTANCE, HALF_CELL_SIZE_MM, SEARCH_SPEED_DEFAULT_MM_S, SEARCH_ACCEL_DEFAULT_MM_S2,
                      true, current_search_speed_, exit_v, true);
    current_search_speed_ = exit_v;
    search_phase_ = PHASE_AT_CENTRE;
}

NavState Navigator::getState() const {
    return state_;
}

RobotPose Navigator::getPose() const {
    return pose_;
}

const Maze& Navigator::getMaze() const {
    return maze_;
}

Maze& Navigator::getMaze() {
    return maze_;
}

void Navigator::clearSavedMaze() {
    maze_.clearNVS();
    maze_.reset();
    floodfill_.setGoalToCenter();
    floodfill_.recalculate();
    Serial.println("[NAV] Flash NVS maze cleared and grid reset.");
}
