#pragma once

// Deciding where to go:
//   Maze       - the wall map (saved to flash)
//   Floodfill  - search-run solver (explore towards the centre)
//   Dijkstra   - speed-run solver (fastest known path)
//   Decomposer - turns a cell path into straights / diagonals / slaloms
//   Navigator  - the state machine that feeds MotionCommands to the motion controller

#include "config.h"
#include "types.h"

// ==============================================================================
// MAZE MAP
// ==============================================================================

#include <Arduino.h>

class Maze {
public:
    Maze();
    void reset();

    // Wall inspection and updates
    bool hasWall(int8_t x, int8_t y, Direction dir) const;
    void setWall(int8_t x, int8_t y, Direction dir);
    void clearWall(int8_t x, int8_t y, Direction dir);

    // Visited flags
    bool isVisited(int8_t x, int8_t y) const;
    void setVisited(int8_t x, int8_t y);

    // Update cell walls based on robot heading and detected IR walls
    void updateCellWalls(int8_t x, int8_t y, Direction heading, bool wall_left, bool wall_front, bool wall_right);

    // Utility: get absolute direction from relative sensor orientation
    static Direction getAbsoluteDirection(Direction heading, int8_t relative_turn);

    // ASCII visualizer
    void printMazeToSerial(int8_t current_x, int8_t current_y) const;

    // Flash NVS persistence
    void saveToNVS();
    bool loadFromNVS();
    void clearNVS();
    bool hasSavedMaze() const;

private:
    uint8_t cells_[MAZE_WIDTH][MAZE_HEIGHT];
};

// ==============================================================================
// FLOODFILL SOLVER
// ==============================================================================

#include <Arduino.h>

class Floodfill {
public:
    Floodfill(const Maze& maze);

    // Set destination goals: Center (7,7 - 8,8) or Start (0,0)
    void setGoalToCenter();
    void setGoalToStart();

    // Recompute floodfill distance matrix
    void recalculate();

    // Determine the optimal next direction to move from current cell
    Direction getNextDirection(int8_t current_x, int8_t current_y, Direction current_heading);

    // Get calculated distance value for a cell
    uint16_t getDistance(int8_t x, int8_t y) const;

    // Check if the robot has reached its goal
    bool isAtGoal(int8_t x, int8_t y) const;

private:
    const Maze& maze_;
    uint16_t distance_[MAZE_WIDTH][MAZE_HEIGHT];
    bool is_goal_[MAZE_WIDTH][MAZE_HEIGHT];
};

// ==============================================================================
// DIJKSTRA SOLVER
// ==============================================================================

#include <stdint.h>

class Dijkstra {
public:
    Dijkstra(const Maze& maze);

    // Compute fastest path to center goals: (7,7), (7,8), (8,7), (8,8)
    uint8_t findFastestPathToCenter(int8_t start_x, int8_t start_y, Direction start_h,
                                   Coordinate* out_path, uint8_t max_path_len);

    // Compute fastest path to start goal: (0,0)
    uint8_t findFastestPathToStart(int8_t start_x, int8_t start_y, Direction start_h,
                                  Coordinate* out_path, uint8_t max_path_len);

    // General search with custom goal callback/matrix
    uint8_t findFastestPath(int8_t start_x, int8_t start_y, Direction start_h,
                            bool (*is_goal_fn)(int8_t, int8_t),
                            Coordinate* out_path, uint8_t max_path_len);

    struct HeapNode {
        float cost;
        int8_t x;
        int8_t y;
        uint8_t h;
    };

    struct ParentEdge {
        int8_t prev_x;
        int8_t prev_y;
        uint8_t prev_h;
        uint8_t move_type; // 0=turn, 1=straight, 2=diagonal, 3=slalom
        uint8_t count;
        uint8_t d_cross;
        bool has_parent;
    };

private:
    const Maze& maze_;
};

// ==============================================================================
// PATH DECOMPOSER
// ==============================================================================

#include <stdint.h>

class Decomposer {
public:
    // Decomposes a continuous sequence of cells into optimal execution primitives:
    // - SEG_SLALOM: Multi-wave up-and-down / left-and-right continuous zigzags
    // - SEG_DIAGONAL: Diagonal staircases and 45° corner cuts
    // - SEG_STRAIGHT: Multi-cell straight corridor sprints
    static uint8_t decompose(const Coordinate* path, uint8_t path_len,
                             PathSegment* out_segments, uint8_t max_segments,
                             bool allow_diagonals = true);

    static Direction getDirection(Coordinate from, Coordinate to);

private:
    static uint8_t findSlalomLength(const Direction* dirs, uint8_t start_idx, uint8_t n);
};

// ==============================================================================
// NAVIGATOR
// ==============================================================================

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

class Navigator {
public:
    Navigator(QueueHandle_t motion_cmd_queue, QueueHandle_t telemetry_queue);

    void begin();

    // Core 0 navigation step execution
    // `preview` is what the motion controller saw of the cell ahead during the move that just ended
    void step(const IRReadings& ir, const WallPreview& preview = WallPreview());
    void notifyMotionComplete();

    // High level state triggers
    // Every run starts with the robot in the start cell facing into the maze.
    // The search keeps whatever map is already known (clearSavedMaze() forgets it), drives to the
    // centre, then explores its way back to the start.
    void startSearchRun();
    void startSpeedRun(SpeedrunStrategy strategy = SPEEDRUN_HYBRID_AUTO);
    void stop();

    // Fraction of the SPEEDRUN_* speeds and accelerations the next speed run uses (speed tiers)
    void setSpeedScale(float scale) { speed_scale_ = scale; }

    // After a search: true if the shortest possible route to the centre runs only through cells
    // the robot has seen, i.e. no further exploring can find a shorter one
    bool isBestRouteExplored() const { return best_route_explored_; }

    NavState getState() const;
    RobotPose getPose() const;
    const Maze& getMaze() const;
    Maze& getMaze();
    void clearSavedMaze();

private:
    void sendMotionCommand(MotionAction action, float param, float max_speed, float accel,
                           bool wall_centering = true, float entry_speed = 0.0f, float exit_speed = 0.0f,
                           bool stop_at_front_wall = false);

    // Segment & Subcommand Execution Pipeline
    void queueSegment(const PathSegment& seg, float cruise_speed, float accel);
    void pushSubCommand(MotionAction action, float param, float max_speed, float accel,
                        bool wall_centering, float entry_speed = 0.0f, float exit_speed = 0.0f);
    void pushPivot(Direction target, float turn_speed, float turn_accel);
    void processSubcommandQueue();

    QueueHandle_t motion_cmd_queue_;
    QueueHandle_t telemetry_queue_;

    Maze maze_;
    Floodfill floodfill_;
    Dijkstra dijkstra_;

    NavState state_;
    SpeedrunStrategy current_strategy_;
    RobotPose pose_;

    bool waiting_for_motion_;
    float current_search_speed_;

    // Search look-ahead: where in a cell the robot is when the next decision is taken
    enum SearchPhase : uint8_t {
        PHASE_AT_CENTRE,    // Normal: decide at the cell centre with all sensors
        PHASE_AT_EDGE,      // On the entry edge of pose_.cell: curve through it or carry on to its centre
        PHASE_AFTER_CURVE   // Just curved through a cell; now on the entry edge of pose_.cell
    };
    SearchPhase search_phase_;
    int8_t curve_cell_x_, curve_cell_y_;   // Cell that was curved through
    Direction curve_entry_dir_;            // Heading on entering it (its front wall faces this way)
    bool curve_cell_was_known_;

    bool checkBestRouteExplored();
    void saveMazeIfChanged();

    float speed_scale_;
    bool best_route_explored_;
    bool maze_changed_;          // New cells seen since the map was last written to flash
    bool map_reset_this_run_;

    void stepAtCellEdge(const IRReadings& ir, const WallPreview& preview);
    void stepAfterCurve(const WallPreview& preview);
    void driveToCellCentre();

    // Subcommand queue for multi-phase motions (diagonals & slaloms)
    static constexpr uint8_t MAX_SUB_CMDS = 32;
    MotionCommand sub_cmd_queue_[MAX_SUB_CMDS];
    uint8_t sub_cmd_count_;
    uint8_t sub_cmd_idx_;

    // Segment queue for multi-segment routes (return & speedrun)
    static constexpr uint8_t MAX_SEGMENTS = 255;
    PathSegment segment_queue_[MAX_SEGMENTS];
    uint8_t segment_count_;
    uint8_t segment_idx_;

    // Carried from one segment to the next so the robot does not stop in between
    float carry_speed_;   // Speed the previous segment ends at (0 = it stops)
    bool curve_in_;       // Previous segment ended with a smooth 90° curve into this one
};
