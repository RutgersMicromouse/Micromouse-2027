#pragma once

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
    ACTION_MOVE_DIAGONAL_HALF,   // Move forward N diagonal half-steps (N * 127.28mm)
    ACTION_TURN_LEFT_90,         // In-place 90 deg counter-clockwise turn
    ACTION_TURN_RIGHT_90,        // In-place 90 deg clockwise turn
    ACTION_TURN_LEFT_45,         // In-place 45 deg counter-clockwise turn
    ACTION_TURN_RIGHT_45,        // In-place 45 deg clockwise turn
    ACTION_CURVE_LEFT_90,        // Smooth continuous 90 deg counter-clockwise arc (R=80mm)
    ACTION_CURVE_RIGHT_90,       // Smooth continuous 90 deg clockwise arc (R=80mm)
    ACTION_CURVE_LEFT_45,        // Smooth continuous 45 deg counter-clockwise arc
    ACTION_CURVE_RIGHT_45,       // Smooth continuous 45 deg clockwise arc
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
};

// Quadrature Encoder State (Faulhaber 1524)
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

// Shared Global Telemetry for Debugging / Diagnostics
struct RobotTelemetry {
    EncoderState encoders;
    IRReadings ir;
    IMUState imu;
    RobotPose pose;
    NavState nav_state;
    float vbat_volts;
    bool motion_completed;
    uint32_t loop_count;
};
