#include "navigation/dijkstra/dijkstra.h"

// ==============================================================================
// DIJKSTRA SOLVER
// ==============================================================================

#include <string.h>
#include <initializer_list>

Dijkstra::Dijkstra(const Maze& maze) : maze_(maze) {}

static const int8_t DX[4] = { 0, 1, 0, -1 };
static const int8_t DY[4] = { 1, 0, -1, 0 };

static bool isCenterGoal(int8_t x, int8_t y) {
    return Maze::isGoalCell(x, y);
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
            if (!maze_.isKnownOpen(px, py, (Direction)h)) break;

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
                if (!maze_.isKnownOpen(cur_x, cur_y, (Direction)s_dir)) break;

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

                if (!maze_.isKnownOpen(cur_x, cur_y, (Direction)s_dir)) break;

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
