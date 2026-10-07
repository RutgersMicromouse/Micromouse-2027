#!/usr/bin/env python3
r"""
Drives a simulated robot with the REAL firmware: the drivers (src/hardware.cpp), the motion
controller (src/control.cpp) and the navigator (src/navigation.cpp), all compiled for this PC.

Where test_firmware_nav.py moves an ideal robot exactly as commanded, this one closes the loop
the way the real robot does. A small physics model plays the hardware:

  motors      wheel speed follows the Motoron command with a lag and some friction
  encoders    the pulse counters tick as the simulated wheels turn
  IMU         a BNO055 that answers the firmware's I2C reads with the simulated heading
  IR sensors  each receiver reads what its emitter would reflect off the nearest wall or post
  battery     a steady supply

The firmware then runs its 500 Hz loop exactly as on the robot: calibrate the IR sensors in the
start cell, search to the goal and back, then speed runs. The test fails if the robot touches a
wall, trips one of its own safety stops, or does not finish.

What this can and cannot tell you. It catches wrong signs, wrong units, state-machine mistakes
and moves that do not join up, in code that has never run on the robot. It does NOT prove the
tuning: the motor lag, friction, sensor positions and reflection strength below are guesses, and
the real robot will differ. Treat a pass as "the logic holds together", not "the gains are right".

Needs a C++ compiler:   pip install ziglang
Run:                    python sim/tests/test_firmware_drive.py
Watch one run:          python sim/tests/test_firmware_drive.py --verbose --size 3 --seed 1
"""
import os
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_firmware_nav import ROOT, STUBS, find_compiler, run_test_program  # noqa: E402

# ----------------------------------------------------------------------------------------------
# Stand-ins for the ESP32 / Arduino / Pololu headers the drivers include. They forward to the
# simulated hardware in the harness below.
# ----------------------------------------------------------------------------------------------
DRIVE_STUBS = dict(STUBS)
DRIVE_STUBS['Arduino.h'] = STUBS['Arduino.h'] + r'''
#include <algorithm>
using std::min;
using std::max;
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
#define LOW 0
#define HIGH 1
#define ADC_11db 3
inline void pinMode(int, int) {}
inline void analogReadResolution(int) {}
inline void analogSetAttenuation(int) {}
inline void delayMicroseconds(unsigned) {}
void digitalWrite(int pin, int level);
int analogRead(int pin);
unsigned long millis();
unsigned long micros();
void delay(unsigned long ms);
'''
DRIVE_STUBS['Preferences.h'] = STUBS['Preferences.h'].replace('''    void putBool(''', '''    void putFloat(const char* key, float v) { putBytes(key, &v, sizeof(v)); }
    float getFloat(const char* key, float fallback) { float v; return getBytes(key, &v, sizeof(v)) == sizeof(v) ? v : fallback; }
    void putUShort(const char* key, uint16_t v) { putBytes(key, &v, sizeof(v)); }
    uint16_t getUShort(const char* key, uint16_t fallback) { uint16_t v; return getBytes(key, &v, sizeof(v)) == sizeof(v) ? v : fallback; }
    void putBool(''')
DRIVE_STUBS['Wire.h'] = r'''
#pragma once
#include <stdint.h>
// The I2C bus. The only device the firmware talks to through it directly is the BNO055 IMU.
class TwoWire {
public:
    void beginTransmission(uint8_t address);
    void write(uint8_t value);
    uint8_t endTransmission();
    uint8_t requestFrom(uint8_t address, uint8_t count);
    int available();
    int read();
};
extern TwoWire Wire;
'''
DRIVE_STUBS['Motoron.h'] = r'''
#pragma once
#include <stdint.h>
#define MOTORON_STATUS_FLAG_RESET 9
// The Pololu Motoron motor driver, as far as the firmware uses it
class MotoronI2C {
public:
    void setAddress(uint8_t) {}
    void reinitialize() {}
    void clearResetFlag() {}
    void setMaxAcceleration(uint8_t, uint16_t) {}
    void setMaxDeceleration(uint8_t, uint16_t) {}
    uint16_t getStatusFlags() { return 0; }
    uint8_t getLastError() { return 0; }
    uint32_t getVinVoltageMv(uint16_t) { return 12000; }
    void setAllSpeedsNow(int16_t motor1_right, int16_t motor2_left);
    void setBrakingNow(uint8_t motor, uint16_t amount);
};
'''
DRIVE_STUBS['driver/pcnt.h'] = r'''
#pragma once
#include <stdint.h>
// The ESP32 pulse counters that read the wheel encoders
typedef enum { PCNT_UNIT_0 = 0, PCNT_UNIT_1 = 1 } pcnt_unit_t;
typedef enum { PCNT_CHANNEL_0 = 0, PCNT_CHANNEL_1 = 1 } pcnt_channel_t;
typedef enum { PCNT_MODE_KEEP = 0, PCNT_MODE_REVERSE = 1 } pcnt_ctrl_mode_t;
typedef enum { PCNT_COUNT_INC = 1, PCNT_COUNT_DEC = 2 } pcnt_count_mode_t;
typedef struct {
    int pulse_gpio_num;
    int ctrl_gpio_num;
    pcnt_ctrl_mode_t lctrl_mode;
    pcnt_ctrl_mode_t hctrl_mode;
    pcnt_count_mode_t pos_mode;
    pcnt_count_mode_t neg_mode;
    int16_t counter_h_lim;
    int16_t counter_l_lim;
    pcnt_unit_t unit;
    pcnt_channel_t channel;
} pcnt_config_t;
inline void pcnt_unit_config(const pcnt_config_t*) {}
inline void pcnt_set_filter_value(pcnt_unit_t, uint16_t) {}
inline void pcnt_filter_enable(pcnt_unit_t) {}
inline void pcnt_counter_pause(pcnt_unit_t) {}
inline void pcnt_counter_resume(pcnt_unit_t) {}
void pcnt_counter_clear(pcnt_unit_t unit);
void pcnt_get_counter_value(pcnt_unit_t unit, int16_t* count);
'''

# ----------------------------------------------------------------------------------------------
# The simulated robot, and the firmware's main loop around it
# ----------------------------------------------------------------------------------------------
HARNESS = r'''
#include <stdlib.h>
#include <deque>
#include <map>
#include <string>
#include <vector>
#include "hardware.h"
#include "control.h"
#include "navigation.h"

SerialStub Serial;
bool g_verbose = false;
int g_maze_size = 16;
#define N g_maze_size
std::map<std::string, std::vector<unsigned char>> g_fake_flash;

static std::deque<MotionCommand> g_commands;
int xQueueSend(QueueHandle_t, const void* item, int) {
    g_commands.push_back(*(const MotionCommand*)item);
    return 1;
}

// ================================================================ the maze
static bool g_wall[16][16][4]; // [x][y][N,E,S,W]
static const int DX[4] = { 0, 1, 0, -1 };
static const int DY[4] = { 1, 0, -1, 0 };

static bool trueWall(int x, int y, int dir) {
    if (x < 0 || x >= N || y < 0 || y >= N) return true;
    return g_wall[x][y][dir];
}

static uint32_t g_rng = 1;
static uint32_t rnd() {
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
    return g_rng;
}

static void setTrueWall(int x, int y, int dir, bool present) {
    g_wall[x][y][dir] = present;
    int nx = x + DX[dir], ny = y + DY[dir];
    if (nx >= 0 && nx < N && ny >= 0 && ny < N) g_wall[nx][ny][(dir + 2) % 4] = present;
}

// Same generator as test_firmware_nav.py: every cell reachable, start open only to the north,
// some loops, an open goal area
static void randomMaze(uint32_t seed) {
    g_rng = seed * 2654435761u + 12345u;
    if (g_rng == 0) g_rng = 1;
    for (int x = 0; x < N; x++)
        for (int y = 0; y < N; y++)
            for (int d = 0; d < 4; d++) g_wall[x][y][d] = true;

    static bool seen[16][16];
    static int stack_x[256], stack_y[256];
    memset(seen, 0, sizeof(seen));
    seen[0][0] = true;
    int depth = 0;
    stack_x[0] = 0; stack_y[0] = 1; seen[0][1] = true;
    while (depth >= 0) {
        int x = stack_x[depth], y = stack_y[depth];
        int order[4] = { 0, 1, 2, 3 };
        for (int i = 3; i > 0; i--) { int j = rnd() % (i + 1); int t = order[i]; order[i] = order[j]; order[j] = t; }
        bool moved = false;
        for (int i = 0; i < 4 && !moved; i++) {
            int nx = x + DX[order[i]], ny = y + DY[order[i]];
            if (nx < 0 || nx >= N || ny < 0 || ny >= N || seen[nx][ny]) continue;
            setTrueWall(x, y, order[i], false);
            seen[nx][ny] = true;
            depth++;
            stack_x[depth] = nx; stack_y[depth] = ny;
            moved = true;
        }
        if (!moved) depth--;
    }
    setTrueWall(0, 0, 0, false);
    for (int i = 0; i < N * N / 5; i++) {
        int x = rnd() % N, y = rnd() % N, d = rnd() % 2;
        if (x == 0 && y == 0) continue;
        if (x + DX[d] >= N || y + DY[d] >= N) continue;
        if (x + DX[d] == 0 && y + DY[d] == 0) continue;
        setTrueWall(x, y, d, false);
    }
    for (int x = 0; x < N; x++)
        for (int y = 0; y < N; y++)
            for (int d = 0; d < 2; d++)
                if (Maze::isGoalCell(x, y) && Maze::isGoalCell(x + DX[d], y + DY[d])) setTrueWall(x, y, d, false);
}

static bool loadMaze(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return false;
    int x, y, n, e, s, w, rows = 0;
    while (fscanf(f, "%d %d %d %d %d %d", &x, &y, &n, &e, &s, &w) == 6) {
        if (x < 0 || x >= N || y < 0 || y >= N) continue;
        g_wall[x][y][0] = n; g_wall[x][y][1] = e; g_wall[x][y][2] = s; g_wall[x][y][3] = w;
        rows++;
    }
    fclose(f);
    return rows == N * N;
}

// ================================================================ the simulated robot
// These numbers are guesses at the real robot. Change them to see how sensitive the firmware is.
static const float MOTOR_LAG_S        = 0.080f;  // How quickly wheel speed follows the command
static const float BRAKE_LAG_S        = 0.025f;  // ...and how quickly an active brake stops it
static const float FRICTION_EFFORT    = 0.030f;  // Effort lost to friction before a wheel turns
static const float IR_STRENGTH        = 6.9e6f;  // Reflected reading = this / distance², ~850 at 90 mm
static const float IR_AMBIENT         = 60.0f;   // Reading with the emitter off
static const float ROBOT_RADIUS_MM    = 40.0f;   // Half the robot's width
static const float WALL_HALF_MM       = 6.0f;

// Where each IR sensor sits on the robot (mm ahead of the axle, mm to the left) and where it points
// (degrees, left positive). Order: L90, L45, FL, FR, R45, R90.
static const float SENSOR_AHEAD[6] = { 40.0f, 46.0f, 50.0f, 50.0f, 46.0f, 40.0f };
static const float SENSOR_LEFT[6]  = { 28.0f, 20.0f, 10.0f, -10.0f, -20.0f, -28.0f };
static const float SENSOR_AIM[6]   = { 90.0f, 45.0f, 0.0f, 0.0f, -45.0f, -90.0f };
static const int EMITTER_PIN[6]  = { PIN_IR_E1, PIN_IR_E2, PIN_IR_E3, PIN_IR_E4, PIN_IR_E5, PIN_IR_E6 };
static const int RECEIVER_PIN[6] = { PIN_IR_R1, PIN_IR_R2, PIN_IR_R3, PIN_IR_R4, PIN_IR_R5, PIN_IR_R6 };

struct SimRobot {
    double x = 0, y = 0;        // mm, start cell centre = (0, 0); x east, y north
    double heading = 0;         // radians, counter-clockwise from north (left turns are positive)
    double yaw_rate = 0;        // rad/s
    double wheel_left = 0, wheel_right = 0;      // mm/s
    double ticks_left = 0, ticks_right = 0;      // encoder ticks not yet read by the firmware
    int16_t motor_left = 0, motor_right = 0;     // Motoron commands, -800..800
    bool braking = false;
    bool emitter_on[6] = {};
    double time_s = 0;
    float min_clearance = 1e9f;
    double distance = 0;
    // Where and when the robot was closest to a wall, for explaining a failure
    double worst_x = 0, worst_y = 0, worst_heading = 0, worst_time = 0, worst_speed = 0;
    int worst_action = 0;
};
static int g_current_action = 0; // MotionAction the firmware is carrying out
static SimRobot g_sim;

unsigned long millis() { return (unsigned long)(g_sim.time_s * 1000.0); }
unsigned long micros() { return (unsigned long)(g_sim.time_s * 1e6); }
void delay(unsigned long ms) { g_sim.time_s += ms / 1000.0; } // Only used while standing still

// ---------------------------------------------------------------- Motoron
void MotoronI2C::setAllSpeedsNow(int16_t motor1_right, int16_t motor2_left) {
    g_sim.motor_right = motor1_right;
    g_sim.motor_left = motor2_left;
    g_sim.braking = false;
}
void MotoronI2C::setBrakingNow(uint8_t, uint16_t) {
    g_sim.motor_left = g_sim.motor_right = 0;
    g_sim.braking = true;
}

// ---------------------------------------------------------------- encoders (unit 0 = left, 1 = right)
void pcnt_get_counter_value(pcnt_unit_t unit, int16_t* count) {
    *count = (int16_t)(unit == PCNT_UNIT_0 ? g_sim.ticks_left : g_sim.ticks_right);
}
void pcnt_counter_clear(pcnt_unit_t unit) {
    double& ticks = (unit == PCNT_UNIT_0) ? g_sim.ticks_left : g_sim.ticks_right;
    ticks -= (double)(int16_t)ticks; // Keep the fraction of a tick not yet counted
}

// ---------------------------------------------------------------- BNO055 on the I2C bus
TwoWire Wire;
static uint8_t g_i2c_address = 0, g_i2c_bytes[4], g_i2c_count = 0;
static uint8_t g_bno_register = 0, g_bno_mode = 0;
static uint8_t g_i2c_reply[8], g_i2c_reply_len = 0, g_i2c_reply_pos = 0;

void TwoWire::beginTransmission(uint8_t address) { g_i2c_address = address; g_i2c_count = 0; }
void TwoWire::write(uint8_t value) { if (g_i2c_count < 4) g_i2c_bytes[g_i2c_count++] = value; }
uint8_t TwoWire::endTransmission() {
    if (g_i2c_address != 0x28) return 2; // Nothing else answers
    if (g_i2c_count >= 1) g_bno_register = g_i2c_bytes[0];
    if (g_i2c_count >= 2 && g_bno_register == 0x3D) g_bno_mode = g_i2c_bytes[1];
    return 0;
}
uint8_t TwoWire::requestFrom(uint8_t address, uint8_t count) {
    g_i2c_reply_len = g_i2c_reply_pos = 0;
    if (address != 0x28) return 0;
    for (uint8_t i = 0; i < count && i < 8; i++) {
        uint8_t reg = g_bno_register + i;
        uint8_t value = 0;
        // The BNO055 reports heading clockwise-positive in 1/16 degree, gyro in 1/16 deg/s
        double cw_deg = fmod(-g_sim.heading * 180.0 / M_PI, 360.0);
        if (cw_deg < 0) cw_deg += 360.0;
        int16_t heading_raw = (int16_t)(cw_deg * 16.0);
        int16_t gyro_raw = (int16_t)(g_sim.yaw_rate * 180.0 / M_PI * 16.0);
        if (reg == 0x00) value = 0xA0;                       // Chip ID
        else if (reg == 0x18) value = gyro_raw & 0xFF;
        else if (reg == 0x19) value = (gyro_raw >> 8) & 0xFF;
        else if (reg == 0x1A) value = heading_raw & 0xFF;
        else if (reg == 0x1B) value = (heading_raw >> 8) & 0xFF;
        else if (reg == 0x3D) value = g_bno_mode;
        g_i2c_reply[g_i2c_reply_len++] = value;
    }
    return g_i2c_reply_len;
}
int TwoWire::available() { return g_i2c_reply_len - g_i2c_reply_pos; }
int TwoWire::read() { return g_i2c_reply_pos < g_i2c_reply_len ? g_i2c_reply[g_i2c_reply_pos++] : -1; }

// ---------------------------------------------------------------- IR sensors
void digitalWrite(int pin, int level) {
    for (int i = 0; i < 6; i++) if (pin == EMITTER_PIN[i]) g_sim.emitter_on[i] = (level != 0);
}

// Distance from (ox, oy) along (dx, dy) to the segment a-b, or a huge number if it misses
static float rayToSegment(float ox, float oy, float dx, float dy, float ax, float ay, float bx, float by) {
    float ex = bx - ax, ey = by - ay;
    float denom = dx * ey - dy * ex;
    if (fabsf(denom) < 1e-6f) return 1e9f;
    float t = ((ax - ox) * ey - (ay - oy) * ex) / denom;
    float u = ((ax - ox) * dy - (ay - oy) * dx) / denom;
    return (t > 0.0f && u >= 0.0f && u <= 1.0f) ? t : 1e9f;
}

static int cellOf(double mm) { return (int)floor((mm + 90.0) / 180.0); }

// How far sensor `i` can see before its beam hits a wall face or a post
static float sensorRange(int i) {
    float c = cosf((float)g_sim.heading), s = sinf((float)g_sim.heading);
    // Robot frame (ahead, left) -> world: ahead = (-s, c), left = (-c, -s)
    float ox = (float)g_sim.x + SENSOR_AHEAD[i] * -s + SENSOR_LEFT[i] * -c;
    float oy = (float)g_sim.y + SENSOR_AHEAD[i] * c + SENSOR_LEFT[i] * -s;
    float aim = (float)g_sim.heading + SENSOR_AIM[i] * (float)M_PI / 180.0f;
    float dx = -sinf(aim), dy = cosf(aim);

    float best = 1e9f;
    int cx = cellOf(ox), cy = cellOf(oy);
    for (int x = cx - 3; x <= cx + 3; x++) {
        for (int y = cy - 3; y <= cy + 3; y++) {
            float mx = x * 180.0f, my = y * 180.0f;
            const float h = 90.0f, w = WALL_HALF_MM;
            // Walls are 12 mm thick: the beam hits the near face
            if (trueWall(x, y, 0)) { best = fminf(best, rayToSegment(ox, oy, dx, dy, mx - h, my + h - w, mx + h, my + h - w)); }
            if (trueWall(x, y, 2)) { best = fminf(best, rayToSegment(ox, oy, dx, dy, mx - h, my - h + w, mx + h, my - h + w)); }
            if (trueWall(x, y, 1)) { best = fminf(best, rayToSegment(ox, oy, dx, dy, mx + h - w, my - h, mx + h - w, my + h)); }
            if (trueWall(x, y, 3)) { best = fminf(best, rayToSegment(ox, oy, dx, dy, mx - h + w, my - h, mx - h + w, my + h)); }
            // Posts on the corners: a 12 mm square
            for (int sx = -1; sx <= 1; sx += 2) for (int sy = -1; sy <= 1; sy += 2) {
                float px = mx + sx * h, py = my + sy * h;
                best = fminf(best, rayToSegment(ox, oy, dx, dy, px - w, py - w, px + w, py - w));
                best = fminf(best, rayToSegment(ox, oy, dx, dy, px - w, py + w, px + w, py + w));
                best = fminf(best, rayToSegment(ox, oy, dx, dy, px - w, py - w, px - w, py + w));
                best = fminf(best, rayToSegment(ox, oy, dx, dy, px + w, py - w, px + w, py + w));
            }
        }
    }
    return best;
}

int analogRead(int pin) {
    for (int i = 0; i < 6; i++) {
        if (pin != RECEIVER_PIN[i]) continue;
        float reading = IR_AMBIENT + (float)(rnd() % 9) - 4.0f; // A little noise
        if (g_sim.emitter_on[i]) {
            float d = fmaxf(sensorRange(i), 12.0f);
            reading += IR_STRENGTH / (d * d);
        }
        return (int)constrain(reading, 0.0f, 4095.0f);
    }
    return 1900; // Battery sense pin
}

// ---------------------------------------------------------------- physics, one 2 ms step
static float distToSegment(float px, float py, float ax, float ay, float bx, float by) {
    float vx = bx - ax, vy = by - ay;
    float t = ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy);
    t = constrain(t, 0.0f, 1.0f);
    return hypotf(px - (ax + t * vx), py - (ay + t * vy));
}

static void checkClearance() {
    float px = (float)g_sim.x, py = (float)g_sim.y;
    int cx = cellOf(g_sim.x), cy = cellOf(g_sim.y);
    float best = g_sim.min_clearance;
    for (int x = cx - 1; x <= cx + 1; x++) {
        for (int y = cy - 1; y <= cy + 1; y++) {
            float ox = x * 180.0f, oy = y * 180.0f;
            for (int sx = -1; sx <= 1; sx += 2)
                for (int sy = -1; sy <= 1; sy += 2)
                    best = fminf(best, hypotf(px - (ox + sx * 90.0f), py - (oy + sy * 90.0f)));
            if (trueWall(x, y, 0)) best = fminf(best, distToSegment(px, py, ox - 90, oy + 90, ox + 90, oy + 90));
            if (trueWall(x, y, 1)) best = fminf(best, distToSegment(px, py, ox + 90, oy - 90, ox + 90, oy + 90));
            if (trueWall(x, y, 2)) best = fminf(best, distToSegment(px, py, ox - 90, oy - 90, ox + 90, oy - 90));
            if (trueWall(x, y, 3)) best = fminf(best, distToSegment(px, py, ox - 90, oy - 90, ox - 90, oy + 90));
        }
    }
    if (best < g_sim.min_clearance) {
        g_sim.worst_x = g_sim.x; g_sim.worst_y = g_sim.y; g_sim.worst_heading = g_sim.heading;
        g_sim.worst_time = g_sim.time_s; g_sim.worst_action = g_current_action;
        g_sim.worst_speed = (g_sim.wheel_left + g_sim.wheel_right) * 0.5;
    }
    g_sim.min_clearance = best;
}

static double wheelTarget(int16_t command) {
    double effort = command / (double)MOTORON_MAX_SPEED;
    double magnitude = fabs(effort) - FRICTION_EFFORT;
    if (magnitude <= 0.0) return 0.0;
    return (effort > 0 ? 1.0 : -1.0) * magnitude / (1.0 - FRICTION_EFFORT) * MOTOR_NO_LOAD_SPEED_MM_S;
}

static void physicsStep(double dt) {
    double lag = g_sim.braking ? BRAKE_LAG_S : MOTOR_LAG_S;
    g_sim.wheel_left  += (wheelTarget(g_sim.motor_left)  - g_sim.wheel_left)  * dt / lag;
    g_sim.wheel_right += (wheelTarget(g_sim.motor_right) - g_sim.wheel_right) * dt / lag;

    double speed = (g_sim.wheel_left + g_sim.wheel_right) * 0.5;
    g_sim.yaw_rate = (g_sim.wheel_right - g_sim.wheel_left) / WHEEL_BASE_MM; // Left turn positive
    g_sim.heading += g_sim.yaw_rate * dt;
    g_sim.x += -sin(g_sim.heading) * speed * dt;
    g_sim.y +=  cos(g_sim.heading) * speed * dt;
    g_sim.distance += fabs(speed) * dt;

    g_sim.ticks_left  += g_sim.wheel_left  * dt * TICKS_PER_MM;
    g_sim.ticks_right += g_sim.wheel_right * dt * TICKS_PER_MM;
    g_sim.time_s += dt;
    checkClearance();
}

// ================================================================ the firmware, as main.cpp runs it
static bool g_wall_centring = true; // --no-centring: switch the IR wall centring off, to see what it contributes
static Encoders*         g_encoders;
static Motors*           g_motors;
static IRSensors*        g_ir;
static IMU*              g_imu;
static MotionController* g_motion;
static Navigator*        g_nav;

static bool g_was_busy = false;
static uint32_t g_tick = 0;
static IRReadings g_ir_snapshot = {};
static const char* g_abort_reason = nullptr;

// One pass of motionControlTask + navigationTask from src/main.cpp
static void firmwareTick() {
    g_encoders->update(CONTROL_DT_S);
    g_ir->update();
    EncoderState enc = g_encoders->getState();
    float yaw_rate = (enc.right_speed_mm_s - enc.left_speed_mm_s) / WHEEL_BASE_MM * (180.0f / PI);
    g_imu->update(CONTROL_DT_S, yaw_rate, enc.linear_speed_mm_s);

    if (!g_commands.empty()) {
        MotionCommand cmd = g_commands.front();
        g_commands.pop_front();
        g_motion->executeCommand(cmd);
        g_current_action = (int)cmd.action;
        g_was_busy = true;
    }
    g_motion->update(CONTROL_DT_S);

    bool motion_done = false;
    if (g_was_busy && g_motion->isCommandFinished()) {
        g_was_busy = false;
        motion_done = true;
    }
    if (g_tick++ % 10 == 0) g_ir_snapshot = g_ir->getReadings(); // The 50 Hz telemetry snapshot

    if (g_motion->consumeSafetyStop()) {
        g_abort_reason = "the firmware tripped a safety stop (stall, encoder fault or lost heading)";
        g_nav->stop();
        return;
    }
    if (motion_done) {
        g_nav->notifyMotionComplete();
        g_nav->step(g_ir_snapshot, g_motion->getWallPreview());
    }
}

static bool isActive(NavState s) {
    return s == NAV_STATE_EXPLORING_TO_CENTER || s == NAV_STATE_RETURNING_TO_START || s == NAV_STATE_SPEED_RUNNING;
}

// Runs physics + firmware until the run ends. Returns false on timeout.
static bool runUntilIdle(double time_limit_s) {
    double deadline = g_sim.time_s + time_limit_s;
    int settle = 0;
    while (g_sim.time_s < deadline) {
        physicsStep(CONTROL_DT_S);
        firmwareTick();
        if (g_abort_reason) return true;
        if (!isActive(g_nav->getState()) && g_commands.empty() && !g_was_busy) {
            if (++settle > 150) return true; // Let it come to rest
        } else {
            settle = 0;
        }
    }
    return false;
}

static void placeAtStart() {
    g_sim.x = 0; g_sim.y = 0; g_sim.heading = 0; g_sim.yaw_rate = 0;
    g_sim.wheel_left = g_sim.wheel_right = 0;
    g_sim.ticks_left = g_sim.ticks_right = 0;
    g_sim.motor_left = g_sim.motor_right = 0;
    g_sim.min_clearance = 1e9f;
    g_sim.distance = 0;
    g_commands.clear();
    g_was_busy = false;
    g_abort_reason = nullptr;
}

// Power-on: construct and start every driver, as setup() does
static void bootFirmware() {
    delete g_nav; delete g_motion; delete g_imu; delete g_ir; delete g_motors; delete g_encoders;
    g_bno_mode = 0;
    g_encoders = new Encoders();
    g_motors   = new Motors();
    g_ir       = new IRSensors();
    g_imu      = new IMU();
    g_motion   = new MotionController(*g_encoders, *g_motors, *g_ir, *g_imu);
    g_encoders->begin();
    g_motors->begin();
    g_ir->begin();
    g_imu->begin();
    g_motion->begin();
    g_motion->setWallCenteringEnabled(g_wall_centring);
    g_nav = new Navigator(nullptr, nullptr);
    g_nav->begin();
    for (int i = 0; i < 100; i++) { physicsStep(CONTROL_DT_S); firmwareTick(); } // Idle for 0.2 s
}

static int g_failures = 0;
static int g_tier3_warnings = 0;
static bool g_quiet = false;
static bool check(bool ok, const char* maze, const char* what) {
    if (!ok) {
        printf("    FAIL [%s] %s\n", maze, what);
        printf("         now: t=%.1f s, cell (%d,%d), %.0f mm / %.0f mm from its centre, heading %.0f deg, nav state %d\n",
               g_sim.time_s, cellOf(g_sim.x), cellOf(g_sim.y),
               g_sim.x - cellOf(g_sim.x) * 180.0, g_sim.y - cellOf(g_sim.y) * 180.0,
               -g_sim.heading * 180.0 / M_PI, g_nav ? (int)g_nav->getState() : -1);
        printf("         closest approach %.1f mm at t=%.1f s in cell (%d,%d), %.0f / %.0f mm from its centre, heading %.0f deg, %.0f mm/s, action %d\n",
               g_sim.min_clearance, g_sim.worst_time, cellOf(g_sim.worst_x), cellOf(g_sim.worst_y),
               g_sim.worst_x - cellOf(g_sim.worst_x) * 180.0, g_sim.worst_y - cellOf(g_sim.worst_y) * 180.0,
               -g_sim.worst_heading * 180.0 / M_PI, g_sim.worst_speed, g_sim.worst_action);
        g_failures++;
    }
    return ok;
}

static const float MIN_OK_CLEARANCE = ROBOT_RADIUS_MM + WALL_HALF_MM;

static void describe(const char* label) {
    if (g_quiet) return;
    printf("    %-22s %6.1f s, %6.2f m, closest approach %5.1f mm, ended %5.1f mm from a cell centre\n",
           label, g_sim.time_s, g_sim.distance / 1000.0, g_sim.min_clearance,
           hypot(g_sim.x - round(g_sim.x / 180.0) * 180.0, g_sim.y - round(g_sim.y / 180.0) * 180.0));
}

static void testMaze(const char* name) {
    if (!g_quiet) printf("  %s\n", name);
    g_fake_flash.clear();

    // ---- power on in the start cell and calibrate the IR sensors (what 5 hand waves do)
    placeAtStart();
    bootFirmware();
    if (!check(g_imu->isHardwareConnected(), name, "the IMU driver did not find the simulated BNO055")) return;
    if (!check(g_ir->calibrateInCell(200), name, "IR calibration in the start cell failed")) return;

    // ---- search runs, until the best route is proven (the map is kept between them)
    bool proven = false;
    for (int pass = 1; pass <= 4 && !proven; pass++) {
        placeAtStart();
        g_sim.time_s = 0;
        bootFirmware();
        g_motion->resetTracking(); // What prepareForNewRun() does on the robot
        g_nav->startSearchRun();
        g_nav->step(g_ir->getReadings(), g_motion->getWallPreview());
        bool finished = runUntilIdle(900.0);
        char label[32];
        snprintf(label, sizeof(label), "search %d", pass);
        describe(label);

        if (!check(!g_abort_reason, name, g_abort_reason ? g_abort_reason : "")) return;
        if (!check(finished, name, "search ran out of time")) return;
        if (!check(g_sim.min_clearance >= MIN_OK_CLEARANCE, name, "search hit a wall or post")) return;
        if (!check(g_nav->getState() == NAV_STATE_PREPARING_SPEED_RUN, name, "search did not finish back at the start")) return;
        if (!check(hypot(g_sim.x, g_sim.y) < 45.0, name, "search did not end near the start cell centre")) return;
        proven = g_nav->isBestRouteExplored();
    }
    check(proven, name, "best route still not proven after 4 searches");

    // ---- speed runs at each tier, slowest first, each from a fresh power-on
    const float tiers[3] = { SPEED_TIER_1_SCALE, SPEED_TIER_2_SCALE, SPEED_TIER_3_SCALE };
    const SpeedrunStrategy strategies[2] = { SPEEDRUN_CURVES_ONLY, SPEEDRUN_DIAGONALS_ONLY };
    const char* strategy_names[2] = { "curves", "diagonals" };
    for (int t = 0; t < 3; t++) {
        for (int s = 0; s < 2; s++) {
            placeAtStart();
            g_sim.time_s = 0;
            bootFirmware();
            g_motion->resetTracking();
            g_nav->setSpeedScale(tiers[t]);
            g_nav->startSpeedRun(strategies[s]);
            bool finished = runUntilIdle(300.0);
            char label[48];
            snprintf(label, sizeof(label), "tier %d %s", t + 1, strategy_names[s]);
            describe(label);

            // Tier 3 is full speed, which the robot only reaches after tiers 1 and 2 have both
            // succeeded and which depends on tuning this simulation cannot know. Report it, but
            // do not fail the test on it.
            if (t == 2) {
                bool ok = !g_abort_reason && finished && g_sim.min_clearance >= MIN_OK_CLEARANCE &&
                          g_nav->getState() == NAV_STATE_FINISHED;
                if (!ok) {
                    g_tier3_warnings++;
                    if (!g_quiet) printf("      note: full speed did not get through cleanly here (closest approach %.1f mm, needs %.0f)\n",
                                         g_sim.min_clearance, MIN_OK_CLEARANCE);
                }
                continue;
            }

            char what[96];
            snprintf(what, sizeof(what), "speed run (%s): %s", label, g_abort_reason ? g_abort_reason : "");
            if (!check(!g_abort_reason, name, what)) continue;
            snprintf(what, sizeof(what), "speed run (%s) ran out of time", label);
            if (!check(finished, name, what)) continue;
            snprintf(what, sizeof(what), "speed run (%s) hit a wall or post", label);
            check(g_sim.min_clearance >= MIN_OK_CLEARANCE, name, what);
            snprintf(what, sizeof(what), "speed run (%s) did not finish in the goal", label);
            check(g_nav->getState() == NAV_STATE_FINISHED &&
                  Maze::isGoalCell((int8_t)cellOf(g_sim.x), (int8_t)cellOf(g_sim.y)), name, what);
        }
    }
}

int main(int argc, char** argv) {
    int random_count = 0;
    uint32_t first_seed = 1;
    std::vector<const char*> files;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--verbose") == 0) { g_verbose = true; continue; }
        if (strcmp(argv[i], "--quiet") == 0)   { g_quiet = true; continue; }
        if (strcmp(argv[i], "--no-centring") == 0) { g_wall_centring = false; continue; }
        if (strcmp(argv[i], "--size") == 0 && i + 1 < argc)   { g_maze_size = atoi(argv[++i]); continue; }
        if (strcmp(argv[i], "--random") == 0 && i + 1 < argc) { random_count = atoi(argv[++i]); continue; }
        if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc)   { first_seed = (uint32_t)atoi(argv[++i]); continue; }
        files.push_back(argv[i]);
    }

    int mazes = 0;
    for (const char* path : files) {
        const char* name = strrchr(path, '/') ? strrchr(path, '/') + 1 : path;
        if (strrchr(name, '\\')) name = strrchr(name, '\\') + 1;
        if (!loadMaze(path)) { printf("  could not read %s\n", path); g_failures++; continue; }
        testMaze(name);
        mazes++;
    }
    for (int i = 0; i < random_count; i++) {
        char name[48];
        snprintf(name, sizeof(name), "random %dx%d maze, seed %u", N, N, (unsigned)(first_seed + i));
        randomMaze(first_seed + i);
        testMaze(name);
        mazes++;
    }
    printf("  %d mazes, %d check(s) failed, %d full-speed (tier 3) run(s) not clean\n", mazes, g_failures, g_tier3_warnings);
    return g_failures ? 1 : 0;
}
'''


def main():
    compiler = find_compiler()
    if compiler is None:
        print('No C++ compiler found. Install one with:  pip install ziglang')
        return 2

    passthrough = sys.argv[1:]
    work = tempfile.mkdtemp(prefix='mouse_drive_')
    try:
        for rel, text in DRIVE_STUBS.items():
            path = os.path.join(work, 'stubs', rel)
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'w', encoding='utf-8') as f:
                f.write(text)
        harness = os.path.join(work, 'harness.cpp')
        with open(harness, 'w', encoding='utf-8') as f:
            f.write(HARNESS)

        exe = os.path.join(work, 'drive.exe')
        src = os.path.join(ROOT, 'src')
        command = compiler + ['-std=c++17', '-O2', '-D_USE_MATH_DEFINES', '-DMAZE_ACTIVE_SIZE=g_maze_size',
                              '-I', os.path.join(work, 'stubs'), '-I', src, harness,
                              os.path.join(src, 'hardware.cpp'), os.path.join(src, 'control.cpp'),
                              os.path.join(src, 'navigation.cpp'), '-o', exe]
        print('Compiling the firmware drivers, motion controller and navigator for this PC...')
        result = subprocess.run(command, capture_output=True, text=True)
        if result.returncode != 0:
            print(result.stdout + result.stderr)
            return 1

        if passthrough:
            batches = [('Chosen run', [exe] + passthrough)]
        else:
            maze = lambda name: os.path.join(ROOT, 'sim', 'mazes', name)  # noqa: E731
            batches = [
                ('20 random 3x3 practice mazes', [exe, '--size', '3', '--random', '20']),
                ('5 random full-size mazes',     [exe, '--random', '5']),
                ('Three of the mazes in sim/mazes', [exe, maze('apic_world_cup.num'), maze('diagonal_paradise.num'),
                                                     maze('japan_finals.num')]),
            ]
        failed = 0
        for title, run in batches:
            print(f'\n{title}')
            sys.stdout.flush()
            code = run_test_program(run)
            if code is None:
                return 3
            failed += code != 0
        print('\nALL FIRMWARE DRIVE CHECKS PASSED' if not failed else f'\n{failed} BATCH(ES) FAILED')
        return 1 if failed else 0
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
