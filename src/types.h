#pragma once

// Every data structure shared between subsystems, plus the small math helpers.
// Numbers you might want to tune live in config.h, not here.

// ==============================================================================
// ROBOT STATE & COMMAND TYPES
// ==============================================================================

#include <stdint.h>
#include <stdbool.h>

// Cardinal Maze Directions
enum Direction : uint8_t {
    DIR_NORTH = 0,
    DIR_EAST  = 1,
    DIR_SOUTH = 2,
    DIR_WEST  = 3,
    DIR_INVALID = 255
};

// Navigation Execution Primitives (Straight Sprints, Diagonal Staircases, Slalom Waves)
enum SegmentType : uint8_t {
    SEG_STRAIGHT,
    SEG_DIAGONAL,
    SEG_SLALOM
};

struct PathSegment {
    SegmentType type;
    Direction dir;            // Primary direction (sprint dir, diagonal entry dir, or slalom prog dir)
    Direction secondary_dir;  // Cross direction for diagonal/slalom
    Direction last_dir;       // Exit direction for diagonal
    uint8_t count;            // Sprint count (cells), diagonal length, or slalom length
    int8_t start_x;
    int8_t start_y;
    int8_t end_x;
    int8_t end_y;
};

// Speedrun Strategy Execution Modes
enum SpeedrunStrategy : uint8_t {
    SPEEDRUN_HYBRID_AUTO    = 0, // Auto-optimizer: benchmarks Curves vs Diagonals and executes the fastest
    SPEEDRUN_DIAGONALS_ONLY = 1, // Pure Diagonal Specialist: prioritizes 45° diagonal sprints across all staircases
    SPEEDRUN_CURVES_ONLY    = 2  // Pure Continuous Curves: continuous 90° tangent circular arcs
};

// High-level Actions the Navigator can issue to the Motion Controller
enum MotionAction : uint8_t {
    ACTION_IDLE,
    ACTION_MOVE_FORWARD_CELLS,   // Move forward N cells (180mm * N)
    ACTION_MOVE_DISTANCE,        // Move forward arbitrary mm
    ACTION_MOVE_HALF_CELL,       // Move forward 0.5 cells (90mm approach/exit)
    ACTION_MOVE_DIAGONAL_HALF,   // Move N diagonal half-steps (N * 127.28mm, N may be fractional)
    ACTION_TURN_LEFT_90,         // In-place 90 deg counter-clockwise turn
    ACTION_TURN_RIGHT_90,        // In-place 90 deg clockwise turn
    ACTION_TURN_LEFT_45,         // In-place 45 deg counter-clockwise turn
    ACTION_TURN_RIGHT_45,        // In-place 45 deg clockwise turn
    ACTION_CURVE_LEFT_90,        // Smooth 90 deg left turn, cell edge to cell edge
    ACTION_CURVE_RIGHT_90,       // Smooth 90 deg right turn, cell edge to cell edge
    ACTION_CURVE_LEFT_45,        // Smooth 45 deg left turn onto / off a diagonal
    ACTION_CURVE_RIGHT_45,       // Smooth 45 deg right turn onto / off a diagonal
    ACTION_TURN_AROUND_180,      // In-place 180 deg turnaround
    ACTION_ALIGN_FRONT_WALL,     // Gently tap front wall to zero distance/heading
    ACTION_SQUARE_FRONT_OPTICAL, // Contactless optical squaring against front wall using FL/FR
    ACTION_EMERGENCY_STOP
};

// Motion Command sent from Core 0 to Core 1
struct MotionCommand {
    MotionAction action;
    float param_value;           // Number of cells, distance in mm, or turn angle in deg
    float max_speed_mm_s;        // Speed limit for this trajectory
    float acceleration;          // Acceleration limit
    bool enable_wall_centering;  // Use IR sensors to keep centered in corridor
    float entry_speed_mm_s;      // Initial velocity for smooth chaining (default 0.0)
    float exit_speed_mm_s;       // Desired exit velocity for smooth chaining (default 0.0)
    float start_offset_mm;       // How far past a cell centre a straight begins (default 0.0 = at the centre)
    bool stop_at_front_wall;     // If a wall shows up ahead, finish this move at rest instead of at exit speed
};

// Quadrature Encoder State (N20 30:1 with Magnetic Encoders)
struct EncoderState {
    int32_t left_ticks_total;
    int32_t right_ticks_total;
    int16_t left_delta_ticks;
    int16_t right_delta_ticks;
    float left_dist_mm;
    float right_dist_mm;
    float left_speed_mm_s;
    float right_speed_mm_s;
    float linear_speed_mm_s;
};

// 6-Channel Pulsed IR Wall Sensor Readings (Ambient Subtracted)
struct IRReadings {
    uint16_t left_90;            // Channel 1: Left 90°
    uint16_t left_45;            // Channel 2: Front-Left 45°
    uint16_t front_left;         // Channel 3: Front-Left Center
    uint16_t front_right;        // Channel 4: Front-Right Center
    uint16_t front_center;       // Combined Front Reading (max of FL and FR)
    uint16_t right_45;           // Channel 5: Front-Right 45°
    uint16_t right_90;           // Channel 6: Right 90°

    bool wall_left;              // Detected left wall (via 90° side sensor)
    bool wall_front;             // Detected front wall
    bool wall_right;             // Detected right wall (via 90° side sensor)

    bool post_edge_left;         // Detected left pillar/post falling edge (wall ending)
    bool post_edge_right;        // Detected right pillar/post falling edge (wall ending)
    bool post_rising_left;       // Detected left pillar/post rising edge (wall beginning)
    bool post_rising_right;      // Detected right pillar/post rising edge (wall beginning)

    bool opening_left;           // Anticipated opening on the left (wall ending ahead)
    bool opening_right;          // Anticipated opening on the right (wall ending ahead)

    // Centering error: positive means mouse is biased left (steer right)
    float centering_error;
};

// What the motion controller saw of the cell ahead while driving (search look-ahead).
// "wall" and "open" are both false when the sensors were not sure.
struct WallPreview {
    bool left_wall,  left_open;   // Next cell's left wall, sampled on the way to its edge
    bool right_wall, right_open;  // Next cell's right wall
    bool front_wall, front_open;  // Front wall of the cell just curved through
};

// IMU / BNO055 State
struct IMUState {
    float gyro_z_deg_s;          // Angular rate (deg/s)
    float heading_deg;           // Yaw heading in degrees [-180, 180]
    float heading_rad;           // Yaw heading in radians
    bool is_calibrated;
};

// Estimated Robot Pose
struct RobotPose {
    float x_mm;                  // X position in world
    float y_mm;                  // Y position in world
    float theta_deg;             // Heading angle
    int8_t cell_x;               // Current maze column (0-15)
    int8_t cell_y;               // Current maze row (0-15)
    Direction current_dir;       // Current cardinal direction
};

// High-level Navigator State Machine
enum NavState : uint8_t {
    NAV_STATE_IDLE,
    NAV_STATE_CALIBRATING,
    NAV_STATE_EXPLORING_TO_CENTER,
    NAV_STATE_RETURNING_TO_START,
    NAV_STATE_PREPARING_SPEED_RUN,
    NAV_STATE_SPEED_RUNNING,
    NAV_STATE_FINISHED,
    NAV_STATE_ERROR
};

// Real-Time Control Loop & System Performance Instrumentation
struct TimingStats {
    uint16_t loop_time_us;       // Execution time of last 500 Hz control loop tick in µs
    uint16_t max_loop_time_us;   // Peak execution time observed in µs
    uint32_t loop_overruns;      // Count of loop ticks exceeding the 2000 µs deadline
    uint32_t stack_high_water;   // Core 1 motion task minimum remaining stack (in words)
};

// Shared Global Telemetry for Debugging / Diagnostics
struct RobotTelemetry {
    EncoderState encoders;
    IRReadings ir;
    IMUState imu;
    RobotPose pose;
    NavState nav_state;
    TimingStats timing;
    float vbat_volts;
    bool motion_completed;
    uint32_t loop_count;
};

// ==============================================================================
// MAZE CELL TYPES
// ==============================================================================

#include <stdint.h>

// Cell Wall Bitmasks
#define WALL_NORTH_BIT    (1 << 0)
#define WALL_EAST_BIT     (1 << 1)
#define WALL_SOUTH_BIT    (1 << 2)
#define WALL_WEST_BIT     (1 << 3)
#define CELL_VISITED_BIT  (1 << 4)

// Distance Matrix Sentinel Value
#define DIST_INFINITY     0xFFFF

struct Coordinate {
    int8_t x;
    int8_t y;

    bool operator==(const Coordinate& other) const {
        return (x == other.x && y == other.y);
    }
};

// ==============================================================================
// ANGLE & MATH HELPERS
// ==============================================================================

#include <math.h>

/**
 * @brief Normalizes an angle into the range (-180.0, +180.0] degrees in O(1) time.
 * Eliminates multi-iteration while-loops and handles large positive/negative offsets cleanly.
 */
static inline float normalizeAngle180(float deg) {
    deg = fmodf(deg + 180.0f, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg - 180.0f;
}

/**
 * @brief Normalizes an angle into the range [0.0, 360.0) degrees in O(1) time.
 */
static inline float normalizeAngle360(float deg) {
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) deg += 360.0f;
    return deg;
}

/**
 * @brief Computes the shortest signed angular difference (target - current) in (-180.0, +180.0] degrees.
 * Positive = CCW turn, Negative = CW turn.
 */
static inline float shortestAngularDifference(float target_deg, float current_deg) {
    return normalizeAngle180(target_deg - current_deg);
}

/**
 * @brief Deadband function: zeros out signals smaller than threshold.
 */
static inline float applyDeadband(float value, float threshold) {
    if (fabsf(value) < threshold) return 0.0f;
    return value;
}

/**
 * @brief Constrains a floating point value between min_val and max_val.
 */
static inline float clampFloat(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}
