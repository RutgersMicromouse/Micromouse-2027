#!/usr/bin/env python3
r"""
Drives a simulated robot with the REAL firmware: the drivers (src/hardware/), the motion
controller (src/control/) and the navigator (src/navigation/), all compiled for this PC.

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
#define ADC_0db 0
#define ADC_2_5db 1
#define ADC_6db 2
#define ADC_11db 3
inline void pinMode(int, int) {}
inline void analogReadResolution(int) {}
inline void analogSetAttenuation(int) {}
inline void analogSetPinAttenuation(int, int) {}
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
enum class MotoronVinSenseType { Motoron256 = 0, MotoronHp = 2, Motoron550 = 3, Motoron453 = 6 };
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
    uint32_t getVinVoltageMv(uint16_t, MotoronVinSenseType = MotoronVinSenseType::Motoron256);
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
#include "hardware/encoders/encoders.h"
#include "hardware/motors/motors.h"
#include "hardware/ir_sensors/ir_sensors.h"
#include "hardware/imu/imu.h"
#include "control/motion_controller/motion_controller.h"
#include "control/safety/safety.h"
#include "navigation/navigator/navigator.h"
#include "lab/params.h"

SerialStub Serial;
static LabParams P;                 // sim/lab/params.h: everything about the simulated robot that can be varied
static uint32_t g_lab_rng = 1;      // Its own generator, so the lab's noise never shifts the maze generator
static double labRand() {           // 0 .. 1
    g_lab_rng ^= g_lab_rng << 13; g_lab_rng ^= g_lab_rng >> 17; g_lab_rng ^= g_lab_rng << 5;
    return (g_lab_rng & 0xFFFFFF) / (double)0x1000000;
}
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
#define ROBOT_RADIUS_MM ((float)P.robot_radius_mm)  // Half the robot's width (lab parameter, 40 by default)
#define MIN_OK_CLEARANCE (ROBOT_RADIUS_MM + WALL_HALF_MM)  // Centre closer to a wall or post than this = a collision
static const float WALL_HALF_MM       = 6.0f;

// Where each IR sensor sits on the robot (mm ahead of the axle, mm to the left) and where it points
// (degrees, left positive). Order: L90, L45, FL, FR, R45, R90.
static const float SENSOR_AHEAD[6] = { SIDE_SENSOR_AHEAD_MM, 46.0f, 50.0f, 50.0f, 46.0f, SIDE_SENSOR_AHEAD_MM }; // 90° ones as config.h says
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
    float clearance = 1e9f;     // Robot centre to the nearest wall or post right now, mm
    double distance = 0;
    // Where and when the robot was closest to a wall, for explaining a failure
    double worst_x = 0, worst_y = 0, worst_heading = 0, worst_time = 0, worst_speed = 0;
    int worst_action = 0;
    // Only with the lab's force model (P.dynamics): the body's own speed, apart from the wheels'
    double v = 0, eff_left = 0, eff_right = 0;   // m/s; efforts after the driver's lag
    double slip_s = 0, energy_j = 0, imu_bias_deg = 0, nose_dive_s = 0;
    float hit_cos[6] = { 1, 1, 1, 1, 1, 1 };     // How squarely each IR beam meets what it hits (1 = square on)
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
uint32_t MotoronI2C::getVinVoltageMv(uint16_t, MotoronVinSenseType) { return (uint32_t)(P.supply_volts * 1000.0); }
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
    // Lab faults act on the heading read (which starts at 0x18): an unusable answer, noise, a
    // growing bias, the wrong sign. Each is drawn only when its parameter is not zero.
    const bool heading_read = (g_bno_register == 0x18);
    const bool dropped = heading_read && P.imu_drop > 0 && labRand() < P.imu_drop;
    const double noise = (heading_read && P.imu_noise_deg > 0) ? (labRand() * 2.0 - 1.0) * P.imu_noise_deg : 0.0;
    const double sense = P.imu_flip ? -1.0 : 1.0;
    for (uint8_t i = 0; i < count && i < 8; i++) {
        uint8_t reg = g_bno_register + i;
        uint8_t value = 0;
        // The BNO055 reports heading clockwise-positive in 1/16 degree, gyro in 1/16 deg/s
        double cw_deg = fmod(sense * -g_sim.heading * 180.0 / M_PI + g_sim.imu_bias_deg + noise, 360.0);
        if (cw_deg < 0) cw_deg += 360.0;
        int16_t heading_raw = (int16_t)(cw_deg * 16.0);
        int16_t gyro_raw = (int16_t)(sense * g_sim.yaw_rate * 180.0 / M_PI * 16.0);
        if (dropped) value = 0xFF;                           // (what the robot's failed reads look like: all ones)
        else if (reg == 0x00) value = 0xA0;                  // Chip ID
        else if (reg == 0x32) value = 981 & 0xFF;            // Gravity, Z: 9.81 m/s2 straight down, so the tilt reads level
        else if (reg == 0x33) value = 981 >> 8;
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

    float best = 1e9f, best_cos = 1.0f;
    // A face along x is met at cos = |dy| off square, a face along y at |dx|
    auto face = [&](float ax, float ay, float bx, float by) {
        const float t = rayToSegment(ox, oy, dx, dy, ax, ay, bx, by);
        if (t < best) { best = t; best_cos = (ay == by) ? fabsf(dy) : fabsf(dx); }
    };
    int cx = cellOf(ox), cy = cellOf(oy);
    for (int x = cx - 3; x <= cx + 3; x++) {
        for (int y = cy - 3; y <= cy + 3; y++) {
            float mx = x * 180.0f, my = y * 180.0f;
            const float h = 90.0f, w = WALL_HALF_MM;
            // Walls are 12 mm thick: the beam hits the near face
            if (trueWall(x, y, 0)) face(mx - h, my + h - w, mx + h, my + h - w);
            if (trueWall(x, y, 2)) face(mx - h, my - h + w, mx + h, my - h + w);
            if (trueWall(x, y, 1)) face(mx + h - w, my - h, mx + h - w, my + h);
            if (trueWall(x, y, 3)) face(mx - h + w, my - h, mx - h + w, my + h);
            // Posts on the corners: a 12 mm square
            for (int sx = -1; sx <= 1; sx += 2) for (int sy = -1; sy <= 1; sy += 2) {
                float px = mx + sx * h, py = my + sy * h;
                face(px - w, py - w, px + w, py - w);
                face(px - w, py + w, px + w, py + w);
                face(px - w, py - w, px - w, py + w);
                face(px + w, py - w, px + w, py + w);
            }
        }
    }
    g_sim.hit_cos[i] = best_cos;
    return best;
}

int analogRead(int pin) {
    for (int i = 0; i < 6; i++) {
        if (pin != RECEIVER_PIN[i]) continue;
        // Ambient light is in the reading whether the emitter is on or off (the firmware subtracts
        // one from the other), so it costs range at the top of the scale and, with
        // ir_ambient_noise, steadiness; it does not scale the reflection.
        float reading = (float)P.ir_ambient + ((float)(rnd() % 9) - 4.0f) * (float)(P.ir_noise / 4.0);
        if (P.ir_ambient_noise > 0) reading += (float)((labRand() * 2.0 - 1.0) * P.ir_ambient_noise * P.ir_ambient);
        const bool lost = P.ir_drop[i] > 0 && labRand() < P.ir_drop[i];
        if (g_sim.emitter_on[i] && !lost) {
            float d = fmaxf(sensorRange(i), 12.0f);
            // The same reading as before at 90 mm square on; ir_falloff and ir_angle_power bend it from there
            float reflected = IR_STRENGTH / (90.0f * 90.0f) * powf(90.0f / d, (float)P.ir_falloff) * (float)P.ir_gain[i];
            if (P.ir_angle_power > 0) reflected *= powf(g_sim.hit_cos[i], (float)P.ir_angle_power);
            reading += reflected;
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
    float best = 1e9f;
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
    g_sim.clearance = best;
    if (best < g_sim.min_clearance) {
        g_sim.worst_x = g_sim.x; g_sim.worst_y = g_sim.y; g_sim.worst_heading = g_sim.heading;
        g_sim.worst_time = g_sim.time_s; g_sim.worst_action = g_current_action;
        g_sim.worst_speed = (g_sim.wheel_left + g_sim.wheel_right) * 0.5;
        g_sim.min_clearance = best;
    }
}

static double wheelTarget(int16_t command) {
    double effort = command / (double)MOTORON_MAX_SPEED;
    double magnitude = fabs(effort) - FRICTION_EFFORT;
    if (magnitude <= 0.0) return 0.0;
    return (effort > 0 ? 1.0 : -1.0) * magnitude / (1.0 - FRICTION_EFFORT) * MOTOR_NO_LOAD_SPEED_MM_S;
}

// The lab's force model (P.dynamics 1), in SI units. Each wheel is pushed by its motor
// (stall force x (effort - wheel speed / no-load speed)) and held back by the tyre; the tyre
// passes on to the body what grip allows (friction x weight on that wheel, more with the fan);
// the body has mass and turning inertia. A wheel whose motor asks for more than grip gives
// spins or locks, and its encoder counts the wheel, not the floor.
// NOT modelled: sideways sliding (the robot never drifts sideways in a turn), weight shifting
// under acceleration, airflow, anything about the caster but its share of the weight.
static void dynamicsStep(double dt) {
    const double v_noload = MOTOR_NO_LOAD_SPEED_MM_S / 1000.0;
    const double mass = P.mass_kg + (P.fan_on ? P.fan_mass_kg : 0.0);
    const double weight = mass * 9.81 + (P.fan_on ? P.fan_down_n * P.fan_seal : 0.0);
    const double grip = P.friction * weight * P.drive_weight_share * 0.5;  // What one drive wheel can pass on, N
    const double tyre_stiffness = P.tyre_stiffness;  // (a property of the tyre: more grip raises the limit, not the stiffness)
    const double half_track = WHEEL_BASE_MM / 2000.0;
    const double command_left  = (INVERT_LEFT_MOTOR  ? -1.0 : 1.0) * g_sim.motor_left  / (double)MOTORON_MAX_SPEED;
    const double command_right = (INVERT_RIGHT_MOTOR ? -1.0 : 1.0) * g_sim.motor_right / (double)MOTORON_MAX_SPEED;
    double wl = g_sim.wheel_left / 1000.0, wr = g_sim.wheel_right / 1000.0;

    const int SUBSTEPS = 10; // The tyre is stiff: 0.2 ms steps keep it stable
    const double h = dt / SUBSTEPS;
    for (int i = 0; i < SUBSTEPS; i++) {
        g_sim.eff_left  += (command_left  - g_sim.eff_left)  * h / P.driver_lag_s;
        g_sim.eff_right += (command_right - g_sim.eff_right) * h / P.driver_lag_s;
        // The firmware scales its commands by 12 V / the supply it is told; sag is what it is not told
        const double volts = (P.supply_volts - P.sag_volts * (fabs(g_sim.eff_left) + fabs(g_sim.eff_right)) * 0.5) / 12.0;
        auto motor = [&](double effort, double wheel, double strength) {
            const double over = fabs(effort) - P.deadband;
            const double drive = over > 0.0 ? copysign(over / (1.0 - P.deadband), effort) : 0.0;
            return P.stall_force_n * strength * (drive * volts - wheel / v_noload);
        };
        const double motor_l = motor(g_sim.eff_left, wl, P.motor_left), motor_r = motor(g_sim.eff_right, wr, P.motor_right);
        const double ask_l = tyre_stiffness * (wl - (g_sim.v - g_sim.yaw_rate * half_track));
        const double ask_r = tyre_stiffness * (wr - (g_sim.v + g_sim.yaw_rate * half_track));
        const double tyre_l = constrain(ask_l, -grip, grip), tyre_r = constrain(ask_r, -grip, grip);
        if (fabs(ask_l) > grip || fabs(ask_r) > grip) g_sim.slip_s += h;

        wl += (motor_l - tyre_l) / P.wheel_mass_kg * h;
        wr += (motor_r - tyre_r) / P.wheel_mass_kg * h;
        double push = tyre_l + tyre_r;
        const double rolling = 2.0 * P.rolling_n;
        if (fabs(g_sim.v) < 0.002 && fabs(push) < rolling) { push = 0.0; g_sim.v = 0.0; }
        else push -= (g_sim.v != 0.0 ? copysign(rolling, g_sim.v) : copysign(rolling, push));
        g_sim.v += push / mass * h;
        // Braking harder than this lifts the back and dives the nose: the weight, acting behind the
        // axle, can no longer hold the back down against the braking force acting at the height of
        // the centre of mass. (Only counted: the simulated robot does not actually pitch.)
        const double forward = (g_sim.v >= 0.0) ? 1.0 : -1.0;
        if (-forward * push / mass * P.com_height_mm > 9.81 * P.com_behind_axle_mm) g_sim.nose_dive_s += h;
        g_sim.yaw_rate += ((tyre_r - tyre_l) * half_track - P.yaw_drag_nms * g_sim.yaw_rate) / P.inertia_kg_m2 * h;
        g_sim.heading += g_sim.yaw_rate * h;
        g_sim.x += -sin(g_sim.heading) * g_sim.v * 1000.0 * h;
        g_sim.y +=  cos(g_sim.heading) * g_sim.v * 1000.0 * h;
        g_sim.distance += fabs(g_sim.v) * 1000.0 * h;
        g_sim.energy_j += (fabs(motor_l * wl) + fabs(motor_r * wr) + (P.fan_on ? P.fan_power_w : 0.0)) * h;
    }
    g_sim.wheel_left = wl * 1000.0;
    g_sim.wheel_right = wr * 1000.0;
}

static void physicsStep(double dt) {
  g_sim.imu_bias_deg += P.imu_drift_deg_s * dt;
  if (P.dynamics) {
    dynamicsStep(dt);
  } else {
    double lag = g_sim.braking ? BRAKE_LAG_S : MOTOR_LAG_S;
    // The simulated motors are wired the way config.h says the real ones are
    const double left_motor_sign  = INVERT_LEFT_MOTOR  ? -1.0 : 1.0;
    const double right_motor_sign = INVERT_RIGHT_MOTOR ? -1.0 : 1.0;
    g_sim.wheel_left  += (left_motor_sign  * wheelTarget(g_sim.motor_left)  * P.motor_left  - g_sim.wheel_left)  * dt / lag;
    g_sim.wheel_right += (right_motor_sign * wheelTarget(g_sim.motor_right) * P.motor_right - g_sim.wheel_right) * dt / lag;

    double speed = (g_sim.wheel_left + g_sim.wheel_right) * 0.5;
    g_sim.yaw_rate = (g_sim.wheel_right - g_sim.wheel_left) / WHEEL_BASE_MM; // Left turn positive
    g_sim.heading += g_sim.yaw_rate * dt;
    g_sim.x += -sin(g_sim.heading) * speed * dt;
    g_sim.y +=  cos(g_sim.heading) * speed * dt;
    g_sim.distance += fabs(speed) * dt;
  }

    // The simulated encoders are wired the way config.h says the real ones are
    const double left_sign  = INVERT_LEFT_ENCODER  ? -1.0 : 1.0;
    const double right_sign = INVERT_RIGHT_ENCODER ? -1.0 : 1.0;
    g_sim.ticks_left  += left_sign  * g_sim.wheel_left  * dt / MM_PER_TICK_LEFT  * (1.0 - P.enc_miss_left);
    g_sim.ticks_right += right_sign * g_sim.wheel_right * dt / MM_PER_TICK_RIGHT * (1.0 - P.enc_miss_right);
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
static SafetySupervisor g_safety;
static int g_safety_brakings = 0;
static Navigator*        g_nav;

static bool g_was_busy = false;
static uint32_t g_tick = 0;
static IRReadings g_ir_snapshot = {};
static const char* g_abort_reason = nullptr;
static std::vector<std::pair<std::string, float>> g_tunes; // --tune name=value
static bool g_tune_warned = false;
static int g_moves = -1; // --moves 0|1|2: the speed run's move set (-1 = whatever the firmware starts with)

// One pass of motionControlTask + navigationTask from src/main.cpp
static void firmwareTick() {
    g_encoders->update(CONTROL_DT_S);
    g_ir->update();
    EncoderState enc = g_encoders->getState();
    float yaw_rate = (enc.right_speed_mm_s - enc.left_speed_mm_s) / WHEEL_BASE_MM * (180.0f / PI);
    g_imu->update(CONTROL_DT_S, yaw_rate, enc.linear_speed_mm_s);

    if (g_motors->isInhibited()) {
        g_commands.clear(); // (as main.cpp does: the safety supervisor is holding the robot)
    } else if (!g_commands.empty()) {
        MotionCommand cmd = g_commands.front();
        g_commands.pop_front();
        g_safety.limitCommand(cmd);
        g_motion->executeCommand(cmd);
        g_current_action = (int)cmd.action;
        g_was_busy = true;
    }
    g_motion->update(CONTROL_DT_S);

    // The safety supervisor, wired exactly as in main.cpp's motion task
    {
        static bool controller_stop_seen = false;
        const IRReadings ir_now = g_ir->getReadings();
        SafetySupervisor::Inputs in = {};
        in.speed_mm_s = enc.linear_speed_mm_s;
        in.front_gap_known = g_ir->getFrontGapMm(in.front_gap_mm);
        in.front_left = ir_now.front_left;
        in.front_right = ir_now.front_right;
        in.front_wall_level = g_ir->isFrontLevelMeasured() ? g_ir->getThresholdFront() : 0;
        in.heading_sensor_ok = g_imu->isHardwareConnected() && !g_imu->isUsingFallback();
        in.motor_driver_ok = g_motors->isConnected();
        in.supply_volts = g_motors->getSupplyVolts();
        in.move_active = g_motion->isMoveActive();
        const bool controller_stop = g_motion->safetyStopPending();
        in.controller_stopped = controller_stop && !controller_stop_seen;
        controller_stop_seen = controller_stop;
        const SafetySupervisor::Outputs allowed = g_safety.update(in, CONTROL_DT_S);
        if (allowed.brake_now) { g_motion->safetyStop(); g_safety_brakings++; }
        g_motors->setInhibited(allowed.inhibit);
        if (allowed.inhibit) g_motors->brake();
    }

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

// ================================================================ what is measured about every run (--metrics file)
// The truth is the simulated robot's; what it is compared with is where the maze says it should be.
struct RunMetrics {
    int collisions = 0, near_misses = 0;      // Times it came within the collision limit / within 10 mm of it
    bool hitting = false, near = false;
    double heading_err_max = 0, heading_err_sum = 0;   // True heading off the maze axis while driving straight, degrees
    double lateral_max = 0, lateral_sum = 0;  // Off the middle of the corridor on straights, mm
    long ticks = 0, straight_ticks = 0, saturated_ticks = 0;
    int turns[2] = {};                        // [left, right] turns on the spot measured
    double turn_sum[2] = {}, turn_sq[2] = {}, turn_worst[2] = {};  // True rotation less 90, degrees
    double off_centre_max = 0;                // Furthest from a cell centre at the start of a turn, mm (its stopping error)
    int turning = 0;                          // The turn being measured (action), 0 = none
    double turn_from = 0;
    long ir_checks = 0, ir_right = 0;         // Wall / no wall calls at cell centres against the true maze
    int brakings_at_start = 0;
};
static RunMetrics g_metrics;
static FILE* g_metrics_file = nullptr;
static const char* g_maze_name = "";
static bool g_run_finished = false;

static void metricsTick() {
    RunMetrics& m = g_metrics;
    m.ticks++;
    const bool hit = g_sim.clearance < MIN_OK_CLEARANCE, close = g_sim.clearance < MIN_OK_CLEARANCE + 10.0f;
    if (hit && !m.hitting) m.collisions++;
    else if (close && !m.near && !hit) m.near_misses++;
    m.hitting = hit; m.near = close;

    if (abs(g_sim.motor_left) >= MOTORON_MAX_SPEED || abs(g_sim.motor_right) >= MOTORON_MAX_SPEED) m.saturated_ticks++;

    const double x_off = g_sim.x - round(g_sim.x / 180.0) * 180.0, y_off = g_sim.y - round(g_sim.y / 180.0) * 180.0;
    const int quarter = (int)lround(g_sim.heading / (M_PI / 2.0));
    const bool on_grid = fabs(g_sim.heading - quarter * (M_PI / 2.0)) < 0.09; // within 5 degrees of a maze axis
    const bool straight = g_was_busy && (g_current_action == ACTION_MOVE_FORWARD_CELLS || g_current_action == ACTION_MOVE_DISTANCE ||
                                         g_current_action == ACTION_MOVE_HALF_CELL);
    if (straight) {
        // How far the robot really points off the maze axis it is driving along
        const double off_axis = fabs(g_sim.heading - quarter * (M_PI / 2.0)) * 180.0 / M_PI;
        m.heading_err_max = fmax(m.heading_err_max, off_axis);
        m.heading_err_sum += off_axis;
    }
    if (straight && on_grid) {
        const double lateral = fabs((quarter % 2 == 0) ? x_off : y_off);
        m.lateral_max = fmax(m.lateral_max, lateral);
        m.lateral_sum += lateral;
        m.straight_ticks++;
    }

    // Turns on the spot: true rotation from the tick the turn starts to the tick the robot is released
    const bool turn_action = (g_current_action == ACTION_TURN_LEFT_90 || g_current_action == ACTION_TURN_RIGHT_90);
    if (g_was_busy && turn_action && m.turning == 0) {
        m.turning = g_current_action;
        m.turn_from = g_sim.heading;
        m.off_centre_max = fmax(m.off_centre_max, hypot(x_off, y_off));
    } else if (m.turning != 0 && (!g_was_busy || g_current_action != m.turning)) {
        const int side = (m.turning == ACTION_TURN_LEFT_90) ? 0 : 1;
        const double error = fabs(g_sim.heading - m.turn_from) * 180.0 / M_PI - 90.0;
        m.turns[side]++;
        m.turn_sum[side] += error;
        m.turn_sq[side] += error * error;
        if (fabs(error) > fabs(m.turn_worst[side])) m.turn_worst[side] = error;
        m.turning = 0;
    }

    // Standing near a cell centre and square to the maze: does it call each wall rightly?
    if (m.ticks % 25 == 0 && on_grid && hypot(x_off, y_off) < 20.0) {
        const IRReadings ir = g_ir->getReadings();
        const int cx = cellOf(g_sim.x), cy = cellOf(g_sim.y);
        const int facing = ((-quarter % 4) + 4) % 4; // Heading is anticlockwise; directions go N, E, S, W
        m.ir_checks += 3;
        m.ir_right += (ir.wall_front == trueWall(cx, cy, facing)) + (ir.wall_left == trueWall(cx, cy, (facing + 3) % 4)) +
                      (ir.wall_right == trueWall(cx, cy, (facing + 1) % 4));
    }
}

static void metricsEnd(const char* label) {
    if (!g_metrics_file) return;
    const RunMetrics& m = g_metrics;
    auto mean = [](double sum, long n) { return n ? sum / n : 0.0; };
    auto sd = [](double sum, double sq, int n) { return n > 1 ? sqrt(fmax(0.0, sq / n - (sum / n) * (sum / n))) : 0.0; };
    fprintf(g_metrics_file,
            "{\"maze\":\"%s\",\"run\":\"%s\",\"finished\":%s,\"abort\":\"%s\",\"nav_state\":%d,"
            "\"time_s\":%.2f,\"distance_mm\":%.0f,\"min_clearance_mm\":%.1f,\"collision_limit_mm\":%.1f,"
            "\"collisions\":%d,\"near_misses\":%d,\"heading_err_max_deg\":%.2f,\"heading_err_mean_deg\":%.2f,"
            "\"lateral_max_mm\":%.1f,\"lateral_mean_mm\":%.1f,"
            "\"left_turns\":%d,\"left_turn_mean_deg\":%.2f,\"left_turn_sd_deg\":%.2f,\"left_turn_worst_deg\":%.2f,"
            "\"right_turns\":%d,\"right_turn_mean_deg\":%.2f,\"right_turn_sd_deg\":%.2f,\"right_turn_worst_deg\":%.2f,"
            "\"off_centre_at_turn_max_mm\":%.1f,\"end_off_centre_mm\":%.1f,"
            "\"slip_s\":%.3f,\"nose_dive_s\":%.3f,\"saturated_s\":%.3f,\"safety_brakings\":%d,\"energy_j\":%.2f,"
            "\"ir_checks\":%ld,\"ir_accuracy\":%.4f}\n",
            g_maze_name, label, g_run_finished ? "true" : "false", g_abort_reason ? g_abort_reason : "", (int)g_nav->getState(),
            g_sim.time_s, g_sim.distance, g_sim.min_clearance, MIN_OK_CLEARANCE,
            m.collisions, m.near_misses, m.heading_err_max, mean(m.heading_err_sum, m.straight_ticks + 1),
            m.lateral_max, mean(m.lateral_sum, m.straight_ticks),
            m.turns[0], mean(m.turn_sum[0], m.turns[0]), sd(m.turn_sum[0], m.turn_sq[0], m.turns[0]), m.turn_worst[0],
            m.turns[1], mean(m.turn_sum[1], m.turns[1]), sd(m.turn_sum[1], m.turn_sq[1], m.turns[1]), m.turn_worst[1],
            m.off_centre_max, hypot(g_sim.x - round(g_sim.x / 180.0) * 180.0, g_sim.y - round(g_sim.y / 180.0) * 180.0),
            g_sim.slip_s, g_sim.nose_dive_s, m.saturated_ticks * CONTROL_DT_S, g_safety_brakings - m.brakings_at_start, g_sim.energy_j,
            m.ir_checks, m.ir_checks ? (double)m.ir_right / m.ir_checks : 1.0);
    fflush(g_metrics_file);
}

// ================================================================ trace for the viewer (--trace file)
// One line of JSON per 20 ms of simulated time: where the simulated robot REALLY is, and what the
// firmware believes and commands at that moment. sim/viewer/index.html plays it back. Keep the
// field names in step with that page.
static FILE* g_trace = nullptr;

static void traceMaze(const char* name) {
    if (!g_trace) return;
    fprintf(g_trace, "{\"maze\":\"%s\",\"n\":%d,\"radius\":%.0f,\"wall_half\":%.0f,\"walls\":\"", name, N, ROBOT_RADIUS_MM, WALL_HALF_MM);
    for (int y = 0; y < N; y++)
        for (int x = 0; x < N; x++)
            fputc("0123456789abcdef"[(trueWall(x, y, 0) ? 1 : 0) | (trueWall(x, y, 1) ? 2 : 0) |
                                     (trueWall(x, y, 2) ? 4 : 0) | (trueWall(x, y, 3) ? 8 : 0)], g_trace);
    fprintf(g_trace, "\",\"sensor_ahead\":[%g,%g,%g,%g,%g,%g],\"sensor_left\":[%g,%g,%g,%g,%g,%g],\"sensor_aim\":[%g,%g,%g,%g,%g,%g]}\n",
            SENSOR_AHEAD[0], SENSOR_AHEAD[1], SENSOR_AHEAD[2], SENSOR_AHEAD[3], SENSOR_AHEAD[4], SENSOR_AHEAD[5],
            SENSOR_LEFT[0], SENSOR_LEFT[1], SENSOR_LEFT[2], SENSOR_LEFT[3], SENSOR_LEFT[4], SENSOR_LEFT[5],
            SENSOR_AIM[0], SENSOR_AIM[1], SENSOR_AIM[2], SENSOR_AIM[3], SENSOR_AIM[4], SENSOR_AIM[5]);
}

static void traceFrame() {
    if (!g_trace) return;
    const IRReadings ir = g_ir->getReadings();
    const EncoderState enc = g_encoders->getState();
    const RobotPose pose = g_nav->getPose();
    fprintf(g_trace, "{\"t\":%.2f,\"x\":%.1f,\"y\":%.1f,\"h\":%.1f,\"wl\":%.0f,\"wr\":%.0f,\"ml\":%d,\"mr\":%d,"
                     "\"ir\":[%d,%d,%d,%d,%d,%d],\"rng\":[%.0f,%.0f,%.0f,%.0f,%.0f,%.0f],\"seen\":\"%c%c%c\","
                     "\"act\":%d,\"nav\":%d,\"cx\":%d,\"cy\":%d,\"dir\":%d,\"clr\":%.1f,\"he\":%.1f,\"spd\":%.0f,\"gz\":%.0f,\"safe\":%d}\n",
            g_sim.time_s, g_sim.x, g_sim.y, g_sim.heading * 180.0 / M_PI, g_sim.wheel_left, g_sim.wheel_right,
            (int)g_sim.motor_left, (int)g_sim.motor_right,
            ir.left_90, ir.left_45, ir.front_left, ir.front_right, ir.right_45, ir.right_90,
            fminf(sensorRange(0), 999.0f), fminf(sensorRange(1), 999.0f), fminf(sensorRange(2), 999.0f),
            fminf(sensorRange(3), 999.0f), fminf(sensorRange(4), 999.0f), fminf(sensorRange(5), 999.0f),
            ir.wall_left ? 'L' : '.', ir.wall_front ? 'F' : '.', ir.wall_right ? 'R' : '.',
            g_current_action, (int)g_nav->getState(), (int)pose.cell_x, (int)pose.cell_y, (int)pose.current_dir,
            g_sim.clearance, g_motion->getHeadingErrorDeg(), enc.linear_speed_mm_s,
            g_sim.yaw_rate * 180.0 / M_PI, (int)g_safety.getState());
    // Twice a second, the firmware's own map: per cell, the walls it believes (hex: 1 N, 2 E, 4 S, 8 W)
    // and the openings it has confirmed; whatever is in neither is still unknown to it
    static int frame = 0;
    if (frame++ % 25 == 0) {
        const Maze& map = g_nav->getMaze();
        fputs("{\"map\":\"", g_trace);
        for (int y = 0; y < N; y++) for (int x = 0; x < N; x++) {
            int bits = 0;
            for (int d = 0; d < 4; d++) if (map.hasWall(x, y, (Direction)d)) bits |= 1 << d;
            fputc("0123456789abcdef"[bits], g_trace);
        }
        fputs("\",\"open\":\"", g_trace);
        for (int y = 0; y < N; y++) for (int x = 0; x < N; x++) {
            int bits = 0;
            for (int d = 0; d < 4; d++) if (map.isKnownOpen(x, y, (Direction)d)) bits |= 1 << d;
            fputc("0123456789abcdef"[bits], g_trace);
        }
        fputs("\"}\n", g_trace);
    }
}

// Closes one run in the trace: its name, and how it ended
static void traceEnd(const char* label) {
    if (!g_trace) return;
    fprintf(g_trace, "{\"end\":\"%s\",\"min_clr\":%.1f,\"ok_clr\":%.0f,\"abort\":\"%s\"}\n",
            label, g_sim.min_clearance, ROBOT_RADIUS_MM + WALL_HALF_MM, g_abort_reason ? g_abort_reason : "");
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
        metricsTick();
        if (g_tick % 10 == 0) traceFrame();
        if (g_abort_reason) { g_run_finished = false; return true; }
        if (!isActive(g_nav->getState()) && g_commands.empty() && !g_was_busy) {
            if (++settle > 150) { g_run_finished = true; return true; } // Let it come to rest
        } else {
            settle = 0;
        }
    }
    g_run_finished = false;
    return false;
}

static void placeAtStart() {
    g_sim.x = 0; g_sim.y = 0; g_sim.heading = 0; g_sim.yaw_rate = 0;
    g_sim.wheel_left = g_sim.wheel_right = 0;
    g_sim.ticks_left = g_sim.ticks_right = 0;
    g_sim.motor_left = g_sim.motor_right = 0;
    g_sim.min_clearance = 1e9f;
    g_sim.distance = 0;
    g_sim.v = g_sim.eff_left = g_sim.eff_right = 0;
    g_sim.slip_s = g_sim.energy_j = g_sim.imu_bias_deg = g_sim.nose_dive_s = 0;
    g_metrics = RunMetrics{};
    g_metrics.brakings_at_start = g_safety_brakings;
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
    g_safety.reset();
    // The code's distance scale (dist_k) makes up for the real tyres rolling a little larger than
    // measured. The simulated wheels are exactly the configured size, so here it has to be 1.
    g_motion->setTune(MotionController::TUNE_DIST_K, 1.0f);
    // --tune name=value: tuning values other than the code's, e.g. the set saved on the robot
    for (const auto& tune : g_tunes) {
        bool known = false;
        for (int i = 0; i < MotionController::TUNE_COUNT; i++) {
            if (tune.first == MotionController::tuneName(i)) known = g_motion->setTune(i, tune.second);
        }
        if (!known && !g_tune_warned) printf("  --tune %s=%g: no such setting, or the value is outside its range\n", tune.first.c_str(), tune.second);
    }
    g_tune_warned = true;
    g_motion->setWallCenteringEnabled(g_wall_centring);
    g_nav = new Navigator(nullptr, nullptr);
    g_nav->begin();
    if (g_moves >= 0) { g_nav->setMoveSet((uint8_t)g_moves); g_nav->setSearchCurves(g_moves >= 1); } // (as the checklist does on the robot)
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


static void describe(const char* label) {
    traceEnd(label);
    metricsEnd(label);
    if (g_quiet) return;
    printf("    %-22s %6.1f s, %6.2f m, closest approach %5.1f mm, ended %5.1f mm from a cell centre\n",
           label, g_sim.time_s, g_sim.distance / 1000.0, g_sim.min_clearance,
           hypot(g_sim.x - round(g_sim.x / 180.0) * 180.0, g_sim.y - round(g_sim.y / 180.0) * 180.0));
}

static void testMaze(const char* name) {
    if (!g_quiet) printf("  %s\n", name);
    traceMaze(name);
    g_maze_name = name;
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

// The safety supervisor on made-up measurements (the simulated robot never has a measured front
// level, so the mazes below do not exercise its front-wall check)
static void safetySelfCheck() {
    const char* name = "safety supervisor";
    SafetySupervisor s;
    SafetySupervisor::Inputs in = {};
    in.heading_sensor_ok = true; in.motor_driver_ok = true; in.supply_volts = 12.0f; in.move_active = true;
    SafetySupervisor::Outputs out = {};
    bool braked = false;
    auto run = [&](float seconds) {
        braked = false;
        for (float t = 0.0f; t < seconds; t += CONTROL_DT_S) { out = s.update(in, CONTROL_DT_S); braked = braked || out.brake_now; }
    };

    in.speed_mm_s = 300.0f; in.front_gap_known = true; in.front_gap_mm = 300.0f;
    run(0.5f);
    check(s.getState() == SafetySupervisor::NORMAL && !out.inhibit, name, "a far wall at 300 mm/s should be NORMAL");

    in.front_gap_mm = 40.0f; // 25 mm of room; 300 mm/s needs about 28
    run(0.1f);
    check(s.getState() == SafetySupervisor::BRAKING && braked && out.inhibit, name, "a wall inside the stopping distance should brake and hold the motors");

    in.speed_mm_s = 0.0f;
    run(0.4f);
    check(s.getState() == SafetySupervisor::RECOVERY && out.inhibit, name, "once still, BRAKING should give way to RECOVERY, motors still held");
    in.front_gap_mm = 300.0f;
    run(1.2f);
    check(s.getState() == SafetySupervisor::NORMAL && !out.inhibit, name, "RECOVERY should end by itself once the robot has stood still");

    in.speed_mm_s = 30.0f; in.front_gap_mm = 20.0f; // creeping up to a wall is allowed
    run(0.2f);
    check(s.getState() == SafetySupervisor::NORMAL, name, "creeping toward a wall below the minimum speed should not brake");

    in.speed_mm_s = 300.0f; in.front_gap_known = false; // no distance: no front check (a known limit)
    run(0.2f);
    check(s.getState() == SafetySupervisor::NORMAL, name, "with no front distance there is no front check");

    in.heading_sensor_ok = false;
    run(0.3f);
    MotionCommand straight = {}; straight.action = ACTION_MOVE_DISTANCE; straight.max_speed_mm_s = 400.0f; straight.exit_speed_mm_s = 400.0f;
    MotionCommand turn = {};     turn.action = ACTION_TURN_LEFT_90;      turn.max_speed_mm_s = 120.0f;
    s.limitCommand(straight); s.limitCommand(turn);
    check(s.getState() == SafetySupervisor::CAUTION && fabsf(straight.max_speed_mm_s - 400.0f * SAFETY_CAUTION_SPEED_SCALE) < 0.1f &&
          straight.exit_speed_mm_s <= straight.max_speed_mm_s && turn.max_speed_mm_s == 120.0f,
          name, "a heading sensor in fault should mean CAUTION: straights slowed, turns left alone");

    in.heading_sensor_ok = true; in.motor_driver_ok = false;
    run(0.4f);
    check(s.getState() == SafetySupervisor::FAULT && out.inhibit, name, "a silent motor driver during a move should be a FAULT");
}

// ================================================================ lab experiments (--experiment name)
// Small set pieces on an empty floor, each ending in one line of JSON in the metrics file.
static void emptyFloor() {
    for (int x = 0; x < N; x++) for (int y = 0; y < N; y++) for (int d = 0; d < 4; d++)
        g_wall[x][y][d] = (d == 0 && y == N - 1) || (d == 1 && x == N - 1) || (d == 2 && y == 0) || (d == 3 && x == 0);
}

static void spin(int ticks) {
    for (int i = 0; i < ticks; i++) { physicsStep(CONTROL_DT_S); firmwareTick(); if (g_tick % 10 == 0) traceFrame(); }
}
// Opens an experiment: an empty floor, the robot started, and the trace (if one is wanted) begun
static void beginExperiment(const char* name) {
    emptyFloor(); placeAtStart(); bootFirmware(); g_motion->resetTracking();
    traceMaze(name);
}

// Up to speed along an empty floor, then the firmware's own emergency stop: how far does it go on?
static void experimentBrake() {
    beginExperiment("braking on an empty floor");
    MotionCommand go = {};
    go.action = ACTION_MOVE_DISTANCE; go.param_value = 2400.0f;
    go.max_speed_mm_s = (float)P.exp_speed; go.acceleration = SPEEDRUN_ACCEL_MM_S2;
    g_commands.push_back(go);
    auto speed = [] { return P.dynamics ? g_sim.v * 1000.0 : (g_sim.wheel_left + g_sim.wheel_right) * 0.5; };
    for (int i = 0; i < 2500 && speed() < 0.98 * P.exp_speed; i++) spin(1);
    const double from_speed = speed(), from_y = g_sim.y, from_t = g_sim.time_s, slip_before = g_sim.slip_s;
    g_commands.clear();
    g_motion->emergencyStop();
    for (int i = 0; i < 2500 && fabs(speed()) > 5.0; i++) spin(1);
    spin(100);
    traceEnd("braking");
    printf("  braking from %.0f mm/s: %.1f mm in %.3f s (the safety supervisor's sum allows %.1f mm), wheels slipping for %.3f s\n",
           from_speed, g_sim.y - from_y, g_sim.time_s - from_t, SafetySupervisor::stoppingDistanceMm((float)from_speed), g_sim.slip_s - slip_before);
    if (g_metrics_file) fprintf(g_metrics_file, "{\"experiment\":\"brake\",\"speed_mm_s\":%.0f,\"stop_mm\":%.1f,\"stop_s\":%.3f,\"safety_sum_mm\":%.1f,\"slip_s\":%.3f,\"nose_dive_s\":%.3f}\n",
                                from_speed, g_sim.y - from_y, g_sim.time_s - from_t, SafetySupervisor::stoppingDistanceMm((float)from_speed), g_sim.slip_s - slip_before, g_sim.nose_dive_s);
}

// 900 mm by the wheels' own count on an empty floor, no walls to steer by: where does it really end up?
static void experimentStraight() {
    beginExperiment("a straight on an empty floor");
    MotionCommand go = {};
    go.action = ACTION_MOVE_DISTANCE; go.param_value = 900.0f;
    go.max_speed_mm_s = 300.0f; go.acceleration = SEARCH_ACCEL_DEFAULT_MM_S2;
    const double from_t = g_sim.time_s;
    g_commands.push_back(go);
    spin(2);
    for (int t = 0; t < 5000 && g_was_busy; t++) spin(1);
    const double took = g_sim.time_s - from_t;
    spin(150);
    traceEnd("a 900 mm straight");
    printf("  900 mm asked for: went %.1f mm, %.1f mm to the left, turned %.2f deg, in %.2f s\n",
           g_sim.y, -g_sim.x, g_sim.heading * 180.0 / M_PI, took);
    if (g_metrics_file) fprintf(g_metrics_file, "{\"experiment\":\"straight\",\"asked_mm\":900,\"went_mm\":%.1f,\"left_mm\":%.1f,\"turned_deg\":%.2f,\"took_s\":%.2f}\n",
                                g_sim.y, -g_sim.x, g_sim.heading * 180.0 / M_PI, took);
}

// Five left and five right 90 degree turns on the spot, as the turn test on the robot does them
static void experimentTurns() {
    beginExperiment("turns on the spot on an empty floor");
    g_sim.x = 360; g_sim.y = 360; // Clear of the walls
    double sum[2] = {}, sq[2] = {}, worst[2] = {};
    for (int i = 0; i < 10; i++) {
        const int side = i % 2;
        MotionCommand turn = {};
        turn.action = side ? ACTION_TURN_RIGHT_90 : ACTION_TURN_LEFT_90; turn.param_value = 90.0f;
        turn.max_speed_mm_s = SEARCH_TURN_SPEED_DEG_S; turn.acceleration = SEARCH_TURN_ACCEL_DEG_S2;
        const double from = g_sim.heading;
        g_commands.push_back(turn);
        spin(2);
        for (int t = 0; t < 3000 && g_was_busy; t++) spin(1);
        spin(150);
        const double error = fabs(g_sim.heading - from) * 180.0 / M_PI - 90.0;
        sum[side] += error; sq[side] += error * error;
        if (fabs(error) > fabs(worst[side])) worst[side] = error;
    }
    traceEnd("ten turns on the spot");
    auto sd = [&](int k) { return sqrt(fmax(0.0, sq[k] / 5 - (sum[k] / 5) * (sum[k] / 5))); };
    const double shifted = hypot(g_sim.x - 360.0, g_sim.y - 360.0); // How far the ten turns walked it off the spot
    printf("  turns on the spot, degrees off 90: left mean %+.2f sd %.2f worst %+.2f | right mean %+.2f sd %.2f worst %+.2f | ended %.1f mm from where it began\n",
           sum[0] / 5, sd(0), worst[0], sum[1] / 5, sd(1), worst[1], shifted);
    if (g_metrics_file) fprintf(g_metrics_file, "{\"experiment\":\"turns\",\"left_mean_deg\":%.2f,\"left_sd_deg\":%.2f,\"left_worst_deg\":%.2f,\"right_mean_deg\":%.2f,\"right_sd_deg\":%.2f,\"right_worst_deg\":%.2f,\"shifted_mm\":%.1f}\n",
                                sum[0] / 5, sd(0), worst[0], sum[1] / 5, sd(1), worst[1], shifted);
}

// The robot stood facing a wall at known distances and angles: what each front and 45 degree
// sensor reads there (after the firmware's own filtering), for seeing what lighting, a weak
// sensor or the fall-off rule do to a calibration
static void experimentIrCalibration() {
    emptyFloor(); placeAtStart(); bootFirmware();
    const float distances[] = { 30, 46, 60, 90, 130, 180, 226 }, angles[] = { 0, 15, 30 };
    printf("  front sensor tips to the wall (mm), turned (deg): readings FL FR L45 R45\n");
    for (float angle : angles) for (float d : distances) {
        placeAtStart();
        g_sim.heading = M_PI + angle * M_PI / 180.0;            // Facing the south wall of the start cell
        g_sim.y = -90.0 + WALL_HALF_MM + d + SENSOR_AHEAD[2];   // (the tips swing a little nearer as it turns)
        spin(150);
        const IRReadings ir = g_ir->getReadings();
        printf("    %4.0f mm %3.0f deg: %5d %5d %5d %5d\n", d, angle, ir.front_left, ir.front_right, ir.left_45, ir.right_45);
        if (g_metrics_file) fprintf(g_metrics_file, "{\"experiment\":\"ircal\",\"distance_mm\":%.0f,\"angle_deg\":%.0f,\"fl\":%d,\"fr\":%d,\"l45\":%d,\"r45\":%d}\n",
                                    d, angle, ir.front_left, ir.front_right, ir.left_45, ir.right_45);
    }
}

// The checklist's straights step (src/ui/limits_test, which is robot-only code) driven the same way
// here: a corridor of 5 cells closed at both ends, calibrated in its end cell, then end to end and
// back at each level of LIMITS_STEPS, judged by the SAME tolerances but against the truth (where
// the robot really stopped), plus "touched nothing". `hard` = the side walls change on the way:
// both, left only, none, right only, both (what the checklist tells the owner to build).
static bool runMove(const MotionCommand& move) {
    g_commands.push_back(move);
    spin(2);
    for (int t = 0; t < 6000 && g_was_busy; t++) spin(1);
    spin(200);
    return !g_was_busy;
}

static void experimentLimits(bool hard) {
    const int cx = 2, y0 = 1, cells = 5;
    emptyFloor();
    for (int i = 0; i < cells; i++) {
        setTrueWall(cx, y0 + i, 3, !hard || i == 0 || i == 1 || i == 4); // West = the robot's left going up
        setTrueWall(cx, y0 + i, 1, !hard || i == 0 || i == 3 || i == 4);
    }
    setTrueWall(cx, y0, 2, true);
    setTrueWall(cx, y0 + cells - 1, 0, true);
    placeAtStart(); bootFirmware(); g_motion->resetTracking();
    g_sim.x = cx * 180.0; g_sim.y = y0 * 180.0;
    traceMaze(hard ? "straights: a closed corridor whose side walls change" : "straights: a closed corridor, walls both sides");

    // Calibrate as the robot does: sides facing along the corridor, turn, the front level against the end wall, turn back
    MotionCommand turn = {};
    turn.action = ACTION_TURN_AROUND_180; turn.param_value = 180.0f;
    turn.max_speed_mm_s = SEARCH_TURN_SPEED_DEG_S; turn.acceleration = SEARCH_TURN_ACCEL_DEG_S2;
    spin(200);
    bool calibrated = g_ir->calibrateInCell(200);
    runMove(turn);
    calibrated = calibrated && g_ir->calibrateInCell(200, true);
    runMove(turn);
    g_motion->resetTracking();
    g_sim.min_clearance = 1e9f;

    static const uint8_t kLevels[] = LIMITS_STEPS;
    const float run_mm = (cells - 1) * 180.0f;
    int best = 0;
    const char* why = calibrated && g_ir->isFrontLevelMeasured() ? "every level passed" : "calibration did not measure the front level";
    for (uint8_t percent : kLevels) {
        if (!calibrated || !g_ir->isFrontLevelMeasured()) break;
        bool level_ok = true;
        for (int leg = 0; leg < 2 && level_ok; leg++) {
            const uint16_t brakings = g_safety.getStatus().brakings;
            MotionCommand straight = {};
            straight.action = ACTION_MOVE_DISTANCE; straight.param_value = run_mm;
            straight.max_speed_mm_s = SPEEDRUN_CRUISE_SPEED_MM_S * percent / 100.0f;
            straight.acceleration = SPEEDRUN_ACCEL_MM_S2 * percent / 100.0f;
            straight.enable_wall_centering = true;
            const bool finished = runMove(straight);
            const bool braked = g_safety.getStatus().brakings != brakings;
            if (braked) spin(600); // Until the supervisor lets go
            const double end_y = (leg == 0 ? y0 + cells - 1 : y0) * 180.0;
            const double stop_error = leg == 0 ? end_y - g_sim.y : g_sim.y - end_y; // + short of the centre
            double heading_error = g_sim.heading * 180.0 / M_PI - (leg == 0 ? 0.0 : 180.0);
            while (heading_error > 180.0) heading_error -= 360.0;
            while (heading_error < -180.0) heading_error += 360.0;
            const double aside = g_sim.x - cx * 180.0;
            const float believed = g_ir->getReadings().front_offset_mm; // What the robot itself would report

            // Onto the centre by its own front sensors, then round, as the robot's test does
            MotionCommand nudge = {};
            nudge.action = ACTION_MOVE_DISTANCE; nudge.param_value = fmaxf(-40.0f, fminf(40.0f, believed));
            nudge.max_speed_mm_s = 80.0f; nudge.acceleration = 400.0f;
            if (fabsf(believed) > 3.0f) runMove(nudge);
            const double before = g_sim.heading;
            runMove(turn);
            const double turn_error = fabs(g_sim.heading - before) * 180.0 / M_PI - 180.0;

            const bool touched = g_sim.min_clearance < MIN_OK_CLEARANCE;
            const bool ok = finished && !braked && !touched && fabs(stop_error) <= LIMITS_STOP_TOLERANCE_MM &&
                            fabs(heading_error) <= LIMITS_HEADING_TOLERANCE_DEG && fabs(turn_error) <= LIMITS_TURN_TOLERANCE_DEG;
            printf("  %3d%% run %d: stopped %+.0f mm from the centre (the robot's own sensors said %+.0f), %+.0f mm aside, heading %+.1f deg, "
                   "turn %+.1f deg, closest %.0f mm, safety %s: %s\n", (int)percent, leg + 1, stop_error, believed, aside, heading_error,
                   turn_error, g_sim.min_clearance, braked ? "BRAKED" : "quiet", ok ? "pass" : "FAIL");
            if (!ok) {
                level_ok = false;
                why = touched ? "touched a wall" : braked ? "the safety supervisor braked" : !finished ? "a move did not finish"
                    : fabs(stop_error) > LIMITS_STOP_TOLERANCE_MM ? "stopped too far from the cell centre"
                    : fabs(heading_error) > LIMITS_HEADING_TOLERANCE_DEG ? "ended pointing off the corridor" : "the turn round was off";
            }
        }
        if (!level_ok) break;
        best = percent;
    }
    traceEnd(hard ? "straights, walls changing" : "straights, walls both sides");
    printf("  straights in a closed corridor (%s): best level that passed %d%% (%s)\n", hard ? "side walls changing" : "walls both sides", best, why);
    if (g_metrics_file) fprintf(g_metrics_file, "{\"experiment\":\"limits\",\"hard\":%d,\"best_percent\":%d,\"ended\":\"%s\"}\n", hard ? 1 : 0, best, why);
}

int main(int argc, char** argv) {
    int random_count = 0;
    const char* experiment = nullptr;
    uint32_t first_seed = 1;
    std::vector<const char*> files;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--verbose") == 0) { g_verbose = true; continue; }
        if (strcmp(argv[i], "--quiet") == 0)   { g_quiet = true; continue; }
        if (strcmp(argv[i], "--no-centring") == 0) { g_wall_centring = false; continue; }
        if (strcmp(argv[i], "--size") == 0 && i + 1 < argc)   { g_maze_size = atoi(argv[++i]); continue; }
        if (strcmp(argv[i], "--random") == 0 && i + 1 < argc) { random_count = atoi(argv[++i]); continue; }
        if (strcmp(argv[i], "--seed") == 0 && i + 1 < argc)   { first_seed = (uint32_t)atoi(argv[++i]); continue; }
        if (strcmp(argv[i], "--trace") == 0 && i + 1 < argc)  { g_trace = fopen(argv[++i], "w"); continue; } // For sim/viewer
        if (strcmp(argv[i], "--metrics") == 0 && i + 1 < argc) { g_metrics_file = fopen(argv[++i], "w"); continue; } // For sim/lab
        if (strcmp(argv[i], "--set") == 0 && i + 1 < argc)    { if (!labSet(P, argv[++i])) return 2; continue; }
        if (strcmp(argv[i], "--experiment") == 0 && i + 1 < argc) { experiment = argv[++i]; continue; }
        if (strcmp(argv[i], "--list-params") == 0) { labList(P, stdout); return 0; }
        if (strcmp(argv[i], "--moves") == 0 && i + 1 < argc)  { g_moves = atoi(argv[++i]); continue; }
        if (strcmp(argv[i], "--search-speed") == 0 && i + 1 < argc) { g_search_speed_scale = atoi(argv[++i]) / 100.0f; continue; } // Percent, as "searchspeed N"
        if (strcmp(argv[i], "--tune") == 0 && i + 1 < argc) {
            const char* arg = argv[++i];
            const char* equals = strchr(arg, '=');
            if (!equals) { printf("  --tune needs name=value\n"); return 2; }
            g_tunes.push_back({ std::string(arg, equals - arg), (float)atof(equals + 1) });
            continue;
        }
        files.push_back(argv[i]);
    }

    g_lab_rng = (uint32_t)P.noise_seed * 2654435761u + 1u;
    if (g_lab_rng == 0) g_lab_rng = 1;
    if (g_metrics_file) { fputs("{\"params\":", g_metrics_file); labDump(P, g_metrics_file); fputs("}\n", g_metrics_file); }
    if (experiment) {
        if      (strcmp(experiment, "brake") == 0) experimentBrake();
        else if (strcmp(experiment, "turns") == 0) experimentTurns();
        else if (strcmp(experiment, "straight") == 0) experimentStraight();
        else if (strcmp(experiment, "ircal") == 0) experimentIrCalibration();
        else if (strcmp(experiment, "limits") == 0) experimentLimits(false);
        else if (strcmp(experiment, "limits_hard") == 0) experimentLimits(true);
        else { printf("  no experiment '%s' (brake, straight, turns, ircal, limits, limits_hard)\n", experiment); return 2; }
        return 0;
    }
    safetySelfCheck();

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


def build(work):
    """Compiles the firmware and the harness into `work`. Returns the program's path, or None."""
    compiler = find_compiler()
    if compiler is None:
        print('No C++ compiler found. Install one with:  pip install ziglang')
        return None
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
                          '-I', os.path.join(work, 'stubs'), '-I', src, '-I', os.path.join(ROOT, 'sim'), harness]
    for folder in ('hardware', 'control', 'navigation'):
        for path, _, names in sorted(os.walk(os.path.join(src, folder))):
            command += sorted(os.path.join(path, name) for name in names if name.endswith('.cpp'))
    command += ['-o', exe]
    print('Compiling the firmware drivers, motion controller and navigator for this PC...')
    result = subprocess.run(command, capture_output=True, text=True)
    if result.returncode != 0:
        print(result.stdout + result.stderr)
        return None
    return exe


def main():
    passthrough = sys.argv[1:]
    work = tempfile.mkdtemp(prefix='mouse_drive_')
    try:
        exe = build(work)
        if exe is None:
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
