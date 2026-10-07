#!/usr/bin/env python3
r"""
Runs the robot's REAL navigation code (src/navigation.cpp) on this PC, through every maze in
sim/mazes/.

Unlike the Python simulators next to it, this compiles the firmware's own Maze, Floodfill,
Dijkstra, Decomposer and Navigator and drives them with an ideal robot: each MotionCommand the
navigator sends is carried out exactly, the wall sensors report the true maze, and every
millimetre of the path is checked against the walls and posts.

For each maze it checks that
  * the search reaches the centre and explores its way back to the start cell,
  * searching again (keeping the map) eventually proves the best route is fully explored,
  * a search that is cut short can be resumed from the saved map after a "reboot",
  * all three speed-run strategies reach the centre,
  * nothing ever comes closer to a wall or post than the robot is wide,
with the look-ahead sensing both working and returning "not sure" (so the fallback is exercised).

Needs a C++ compiler. The easiest way on any machine:   pip install ziglang
Run from the repository root:                           python sim/tests/test_firmware_nav.py
"""
import glob
import os
import shutil
import subprocess
import sys
import tempfile

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))

# ----------------------------------------------------------------------------------------------
# Stand-ins for the ESP32 / Arduino headers that navigation.cpp includes
# ----------------------------------------------------------------------------------------------
STUBS = {
    'Arduino.h': r'''
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#define PI 3.14159265358979f
extern bool g_verbose; // --verbose: show what the firmware prints
struct SerialStub {
    template <class... A> void printf(const char* fmt, A... args) { if (g_verbose) ::printf(fmt, args...); }
    void print(const char* s) { if (g_verbose) ::printf("%s", s); }
    template <class T> void print(T) {}
    void println(const char* s) { if (g_verbose) ::printf("%s\n", s); }
    template <class T> void println(T) {}
    void println() {}
};
extern SerialStub Serial;
''',
    'Preferences.h': r'''
#pragma once
#include <map>
#include <string>
#include <vector>
#include <string.h>
// Flash storage kept in memory, so a "reboot" (a new Navigator) finds what the last one saved
extern std::map<std::string, std::vector<unsigned char>> g_fake_flash;
class Preferences {
public:
    void begin(const char* ns, bool) { ns_ = ns; }
    void end() {}
    void clear() {
        for (auto it = g_fake_flash.begin(); it != g_fake_flash.end();) {
            if (it->first.rfind(ns_ + "/", 0) == 0) it = g_fake_flash.erase(it); else ++it;
        }
    }
    void putBytes(const char* key, const void* data, size_t len) {
        const unsigned char* p = (const unsigned char*)data;
        g_fake_flash[ns_ + "/" + key] = std::vector<unsigned char>(p, p + len);
    }
    size_t getBytes(const char* key, void* out, size_t len) {
        auto it = g_fake_flash.find(ns_ + "/" + key);
        if (it == g_fake_flash.end()) return 0;
        size_t n = it->second.size() < len ? it->second.size() : len;
        memcpy(out, it->second.data(), n);
        return n;
    }
    void putBool(const char* key, bool v) { unsigned char b = v; putBytes(key, &b, 1); }
    bool getBool(const char* key, bool fallback) {
        unsigned char b = 0;
        return getBytes(key, &b, 1) == 1 ? (b != 0) : fallback;
    }
private:
    std::string ns_;
};
''',
    'freertos/FreeRTOS.h': r'''
#pragma once
#define portMAX_DELAY 0
''',
    'freertos/queue.h': r'''
#pragma once
typedef void* QueueHandle_t;
int xQueueSend(QueueHandle_t queue, const void* item, int wait); // provided by the harness
''',
}

# ----------------------------------------------------------------------------------------------
# The ideal robot and the checks
# ----------------------------------------------------------------------------------------------
HARNESS = r'''
#include <deque>
#include <map>
#include <string>
#include <vector>
#include "navigation.h"

SerialStub Serial;
bool g_verbose = false;
std::map<std::string, std::vector<unsigned char>> g_fake_flash;

static std::deque<MotionCommand> g_commands;
int xQueueSend(QueueHandle_t, const void* item, int) {
    g_commands.push_back(*(const MotionCommand*)item);
    return 1;
}

// ---------------------------------------------------------------- the true maze
static bool g_wall[16][16][4]; // [x][y][N,E,S,W]
static const int DX[4] = { 0, 1, 0, -1 };
static const int DY[4] = { 1, 0, -1, 0 };

static bool loadMaze(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return false;
    int x, y, n, e, s, w, rows = 0;
    while (fscanf(f, "%d %d %d %d %d %d", &x, &y, &n, &e, &s, &w) == 6) {
        if (x < 0 || x > 15 || y < 0 || y > 15) continue;
        g_wall[x][y][0] = n; g_wall[x][y][1] = e; g_wall[x][y][2] = s; g_wall[x][y][3] = w;
        rows++;
    }
    fclose(f);
    return rows == 256;
}

static bool trueWall(int x, int y, int dir) {
    if (x < 0 || x > 15 || y < 0 || y > 15) return true;
    return g_wall[x][y][dir];
}

// ---------------------------------------------------------------- the ideal robot
// Position in mm with the start cell centre at (0, 0); heading in degrees clockwise from north.
static const float SENSOR_AHEAD_MM = 40.0f;  // How far in front of the wheel axle the sensors sit
static const float ROBOT_RADIUS_MM = 40.0f;  // Half the robot's width
static const float WALL_HALF_MM    = 6.0f;   // Half the thickness of a wall / post

struct Robot {
    float x = 0, y = 0, heading = 0;
    float min_clearance = 1e9f;   // Closest the centre of the robot ever came to a wall or post
    float distance = 0;
    int   stops = 0;              // Times it came to rest part-way through a run
    bool  preview_enabled = true;
    WallPreview preview = {};
};
static Robot g_robot;

static int cellOf(float mm) { return (int)floorf((mm + 90.0f) / 180.0f); }
static int headingDir(float heading) { return (((int)lroundf(heading / 90.0f)) % 4 + 4) % 4; }

static float distToSegment(float px, float py, float ax, float ay, float bx, float by) {
    float vx = bx - ax, vy = by - ay;
    float t = ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return hypotf(px - (ax + t * vx), py - (ay + t * vy));
}

static void checkClearance() {
    int cx = cellOf(g_robot.x), cy = cellOf(g_robot.y);
    float best = g_robot.min_clearance;
    for (int x = cx - 1; x <= cx + 1; x++) {
        for (int y = cy - 1; y <= cy + 1; y++) {
            float ox = x * 180.0f, oy = y * 180.0f; // cell centre
            // Posts stand on every cell corner whether or not walls are attached
            for (int sx = -1; sx <= 1; sx += 2)
                for (int sy = -1; sy <= 1; sy += 2)
                    best = fminf(best, hypotf(g_robot.x - (ox + sx * 90.0f), g_robot.y - (oy + sy * 90.0f)));
            if (trueWall(x, y, 0)) best = fminf(best, distToSegment(g_robot.x, g_robot.y, ox - 90, oy + 90, ox + 90, oy + 90));
            if (trueWall(x, y, 1)) best = fminf(best, distToSegment(g_robot.x, g_robot.y, ox + 90, oy - 90, ox + 90, oy + 90));
            if (trueWall(x, y, 2)) best = fminf(best, distToSegment(g_robot.x, g_robot.y, ox - 90, oy - 90, ox + 90, oy - 90));
            if (trueWall(x, y, 3)) best = fminf(best, distToSegment(g_robot.x, g_robot.y, ox - 90, oy - 90, ox - 90, oy + 90));
        }
    }
    g_robot.min_clearance = best;
}

static void advance(float mm, float heading_deg) {
    float rad = heading_deg * (float)M_PI / 180.0f;
    g_robot.x += sinf(rad) * mm;
    g_robot.y += cosf(rad) * mm;
    g_robot.distance += fabsf(mm);
    checkClearance();
}

static void driveStraight(float mm) {
    int steps = (int)ceilf(fabsf(mm) / 2.0f);
    for (int i = 0; i < steps; i++) advance(mm / steps, g_robot.heading);
}

// Smooth turn exactly as the motion controller drives it: heading = angle * (3u^2 - 2u^3)
static void driveCurve(float angle_cw_deg, float length_mm) {
    const int steps = 200;
    float h0 = g_robot.heading;
    for (int i = 0; i < steps; i++) {
        float u = (i + 0.5f) / steps;
        advance(length_mm / steps, h0 + angle_cw_deg * u * u * (3.0f - 2.0f * u));
    }
    g_robot.heading = h0 + angle_cw_deg;
}

// What the side walls of the cell in front of the sensors are
static void sideWallsAhead(bool& left, bool& right, int& cx, int& cy, int& dir) {
    float rad = g_robot.heading * (float)M_PI / 180.0f;
    cx = cellOf(g_robot.x + sinf(rad) * SENSOR_AHEAD_MM);
    cy = cellOf(g_robot.y + cosf(rad) * SENSOR_AHEAD_MM);
    dir = headingDir(g_robot.heading);
    left  = trueWall(cx, cy, (dir + 3) % 4);
    right = trueWall(cx, cy, (dir + 1) % 4);
}

static IRReadings senseWalls() {
    IRReadings ir = {};
    bool left, right;
    int cx, cy, dir;
    sideWallsAhead(left, right, cx, cy, dir);
    ir.wall_left = left;
    ir.wall_right = right;

    // The front sensors only register a wall once the robot is well inside the cell
    float wall_x = cx * 180.0f + DX[dir] * 90.0f, wall_y = cy * 180.0f + DY[dir] * 90.0f;
    float to_wall = fabsf((wall_x - g_robot.x) * DX[dir] + (wall_y - g_robot.y) * DY[dir]);
    ir.wall_front = trueWall(cx, cy, dir) && to_wall < 130.0f;
    return ir;
}

static void execute(const MotionCommand& cmd) {
    g_robot.preview = {};
    switch (cmd.action) {
        case ACTION_MOVE_FORWARD_CELLS: driveStraight(cmd.param_value * 180.0f); break;
        case ACTION_MOVE_DISTANCE:      driveStraight(cmd.param_value); break;
        case ACTION_MOVE_HALF_CELL:     driveStraight(90.0f); break;
        case ACTION_MOVE_DIAGONAL_HALF: driveStraight(cmd.param_value * DIAG_HALF_STEP_MM); break;
        case ACTION_TURN_LEFT_90:       g_robot.heading -= 90.0f; break;
        case ACTION_TURN_RIGHT_90:      g_robot.heading += 90.0f; break;
        case ACTION_TURN_LEFT_45:       g_robot.heading -= 45.0f; break;
        case ACTION_TURN_RIGHT_45:      g_robot.heading += 45.0f; break;
        case ACTION_TURN_AROUND_180:    g_robot.heading += 180.0f; break;
        case ACTION_CURVE_LEFT_90:
        case ACTION_CURVE_RIGHT_90: {
            // Half-way round, the outer 45 degree sensor reads the front wall of this cell
            bool l, r; int cx, cy, dir;
            sideWallsAhead(l, r, cx, cy, dir);
            bool front = trueWall(cx, cy, dir);
            driveCurve(cmd.action == ACTION_CURVE_RIGHT_90 ? 90.0f : -90.0f, cmd.param_value);
            if (g_robot.preview_enabled) { g_robot.preview.front_wall = front; g_robot.preview.front_open = !front; }
            return;
        }
        case ACTION_CURVE_LEFT_45:      driveCurve(-45.0f, cmd.param_value); break;
        case ACTION_CURVE_RIGHT_45:     driveCurve(45.0f, cmd.param_value); break;
        default: break; // squaring, alignment and emergency stop do not move the ideal robot
    }

    // After a straight, the 45 degree sensors have seen the side walls of the cell ahead
    bool is_straight = (cmd.action == ACTION_MOVE_FORWARD_CELLS || cmd.action == ACTION_MOVE_DISTANCE ||
                        cmd.action == ACTION_MOVE_HALF_CELL);
    if (is_straight && g_robot.preview_enabled) {
        bool l, r; int cx, cy, dir;
        sideWallsAhead(l, r, cx, cy, dir);
        g_robot.preview.left_wall = l;   g_robot.preview.left_open = !l;
        g_robot.preview.right_wall = r;  g_robot.preview.right_open = !r;
    }
    if (cmd.exit_speed_mm_s <= 10.0f && is_straight) g_robot.stops++;
}

static bool isActive(NavState s) {
    return s == NAV_STATE_EXPLORING_TO_CENTER || s == NAV_STATE_RETURNING_TO_START || s == NAV_STATE_SPEED_RUNNING;
}

// Carries out everything the navigator asks for until the run ends (or `max_commands` is reached)
static int runUntilDone(Navigator& nav, int max_commands) {
    int executed = 0;
    while (executed < max_commands) {
        if (g_commands.empty()) break;
        MotionCommand cmd = g_commands.front();
        g_commands.pop_front();
        execute(cmd);
        executed++;
        nav.notifyMotionComplete();
        nav.step(senseWalls(), g_robot.preview);
        if (!isActive(nav.getState()) && g_commands.empty()) break;
    }
    return executed;
}

static void placeAtStart() {
    g_robot.x = 0; g_robot.y = 0; g_robot.heading = 0;
    g_robot.min_clearance = 1e9f; g_robot.distance = 0; g_robot.stops = 0;
    g_robot.preview = {};
    g_commands.clear();
}

static int g_failures = 0;
static void check(bool ok, const char* maze, const char* what) {
    if (!ok) {
        printf("    FAIL [%s] %s\n", maze, what);
        g_failures++;
    }
}

static bool inGoal(int cx, int cy) { return (cx == 7 || cx == 8) && (cy == 7 || cy == 8); }
static const float MIN_OK_CLEARANCE = ROBOT_RADIUS_MM + WALL_HALF_MM;

static void testMaze(const char* path, const char* name, bool preview_enabled) {
    g_robot.preview_enabled = preview_enabled;
    g_fake_flash.clear();
    printf("  %s (look-ahead %s)\n", name, preview_enabled ? "seeing" : "unsure");

    // ---- a search that is cut short, then a "reboot"
    {
        Navigator nav(nullptr, nullptr);
        nav.begin();
        placeAtStart();
        nav.startSearchRun();
        nav.step(senseWalls(), g_robot.preview);
        runUntilDone(nav, 25);
        nav.stop(); // what the firmware does when a run is aborted
        g_commands.clear();
    }

    // ---- searches from the saved map, repeated until the best route is proven
    int passes = 0;
    bool proven = false;
    float search_mm = 0;
    while (passes < 6 && !proven) {
        Navigator nav(nullptr, nullptr);
        nav.begin();
        placeAtStart();
        nav.startSearchRun();
        nav.step(senseWalls(), g_robot.preview);
        runUntilDone(nav, 20000);
        passes++;
        search_mm += g_robot.distance;

        check(nav.getState() == NAV_STATE_PREPARING_SPEED_RUN, name, "search did not finish back at the start");
        check(hypotf(g_robot.x, g_robot.y) < 25.0f, name, "search did not end in the start cell centre");
        check(g_robot.min_clearance >= MIN_OK_CLEARANCE, name, "search path came too close to a wall or post");
        if (nav.getState() != NAV_STATE_PREPARING_SPEED_RUN) break;
        proven = nav.isBestRouteExplored();
        printf("    search %d: %6.1f m, %3d stops, closest approach %5.1f mm, best route %s\n",
               passes, g_robot.distance / 1000.0f, g_robot.stops, g_robot.min_clearance,
               proven ? "proven" : "not yet proven");
    }
    check(proven, name, "best route still not proven after 6 searches");

    // ---- speed runs, each from a fresh "boot" using only the saved map
    const SpeedrunStrategy strategies[3] = { SPEEDRUN_CURVES_ONLY, SPEEDRUN_DIAGONALS_ONLY, SPEEDRUN_HYBRID_AUTO };
    const char* names[3] = { "curves", "diagonals", "hybrid" };
    for (int i = 0; i < 3; i++) {
        Navigator nav(nullptr, nullptr);
        nav.begin();
        placeAtStart();
        nav.startSpeedRun(strategies[i]);
        runUntilDone(nav, 20000);

        float cxf = roundf(g_robot.x / 180.0f) * 180.0f, cyf = roundf(g_robot.y / 180.0f) * 180.0f;
        check(nav.getState() == NAV_STATE_FINISHED, name, "speed run did not finish");
        check(inGoal(cellOf(g_robot.x), cellOf(g_robot.y)), name, "speed run did not end in the centre");
        check(hypotf(g_robot.x - cxf, g_robot.y - cyf) < 1.0f, name, "speed run did not end on a cell centre");
        check(g_robot.min_clearance >= MIN_OK_CLEARANCE, name, "speed run came too close to a wall or post");
        printf("    speed run %-9s: %5.2f m, %2d stops, closest approach %5.1f mm\n",
               names[i], g_robot.distance / 1000.0f, g_robot.stops - 1, g_robot.min_clearance);
    }
    (void)search_mm;
}

int main(int argc, char** argv) {
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--verbose") == 0) { g_verbose = true; continue; }
        const char* path = argv[i];
        const char* name = strrchr(path, '/') ? strrchr(path, '/') + 1 : path;
        if (strrchr(name, '\\')) name = strrchr(name, '\\') + 1;
        if (!loadMaze(path)) { printf("  could not read %s\n", path); g_failures++; continue; }
        testMaze(path, name, true);
        testMaze(path, name, false);
    }
    printf(g_failures ? "\n%d CHECK(S) FAILED\n" : "\nALL FIRMWARE NAVIGATION CHECKS PASSED\n", g_failures);
    return g_failures ? 1 : 0;
}
'''


def find_compiler():
    """A command list that compiles C++: zig (pip install ziglang), or any g++ / clang++ on PATH."""
    try:
        import ziglang  # noqa: F401
        return [sys.executable, '-m', 'ziglang', 'c++']
    except ImportError:
        pass
    for name in ('g++', 'clang++'):
        if shutil.which(name):
            return [name]
    return None


def main():
    compiler = find_compiler()
    if compiler is None:
        print('No C++ compiler found. Install one with:  pip install ziglang')
        return 2

    work = tempfile.mkdtemp(prefix='mouse_nav_')
    try:
        for rel, text in STUBS.items():
            path = os.path.join(work, 'stubs', rel)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w', encoding='utf-8') as f:
                f.write(text)
        harness = os.path.join(work, 'harness.cpp')
        with open(harness, 'w', encoding='utf-8') as f:
            f.write(HARNESS)

        exe = os.path.join(work, 'harness.exe')
        build = compiler + ['-std=c++17', '-O1', '-D_USE_MATH_DEFINES',
                            '-I', os.path.join(work, 'stubs'), '-I', os.path.join(ROOT, 'src'),
                            harness, os.path.join(ROOT, 'src', 'navigation.cpp'), '-o', exe]
        print('Compiling the firmware navigation code for this PC...')
        result = subprocess.run(build, capture_output=True, text=True)
        if result.returncode != 0:
            print(result.stdout + result.stderr)
            return 1

        # Optional arguments: maze names to run (default: all of them), and --verbose
        wanted = [a for a in sys.argv[1:] if not a.startswith('--')]
        flags = [a for a in sys.argv[1:] if a.startswith('--')]
        mazes = sorted(glob.glob(os.path.join(ROOT, 'sim', 'mazes', '*.num')))
        if wanted:
            mazes = [m for m in mazes if any(w in os.path.basename(m) for w in wanted)]
        print(f'Running {len(mazes)} mazes...')
        sys.stdout.flush()
        return subprocess.call([exe] + flags + mazes)
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
