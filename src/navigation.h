#pragma once

// Deciding where to go:
//   Maze       - the wall map (saved to flash)
//   Floodfill  - search-run solver (explore towards the centre)
//   Dijkstra   - speed-run solver (fastest known path)
//   Navigator  - the state machine that feeds MotionCommands to the motion controller

#include "config.h"
#include "types.h"

// ==============================================================================
// MAZE MAP
// ==============================================================================

#include <Arduino.h>

// The robot's memory of the maze.
//
// Walls are not simply on or off: each one carries a tally of sensor readings (+1 each time it
// was seen, -1 each time the gap was seen open). That makes the map self-correcting, which
// matters because it is kept between attempts: a single false reading is outvoted next time the
// robot passes, instead of blocking a route for good.
class Maze {
public:
    Maze();

    // Forget everything except the outer walls
    void reset();

    // Is this cell inside the maze? (0 .. MAZE_ACTIVE_SIZE-1 both ways)
    static bool inMaze(int8_t x, int8_t y);

    // Is this one of the cells the robot is trying to reach?
    static bool isGoalCell(int8_t x, int8_t y);

    // Believed to be a wall: seen as a wall more often than as a gap, or an outer wall.
    // An opening nobody has looked at yet counts as open (the search is optimistic).
    bool hasWall(int8_t x, int8_t y, Direction dir) const;

    // Seen open more often than walled. Speed runs only drive through these.
    bool isKnownOpen(int8_t x, int8_t y, Direction dir) const;

    // Record one sensor reading of the wall on side `dir` of cell (x, y)
    void observeWall(int8_t x, int8_t y, Direction dir, bool wall_present);

    // The robot has just driven through this opening, which settles it: counts as the
    // strongest possible "open" reading
    void confirmOpen(int8_t x, int8_t y, Direction dir);

    // Drop walls that rest on a single reading (used when the map says the robot is walled in)
    void forgetWeakWalls();

    // Visited flags
    bool isVisited(int8_t x, int8_t y) const;
    void setVisited(int8_t x, int8_t y);

    // Record the three walls the robot can see from the centre of a cell, and mark it visited
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
    // The tally for one wall, or nullptr for an outer wall (always present, never changes).
    // Each wall is stored once: as the north wall or the east wall of the cell below / left of it.
    const int8_t* votesFor(int8_t x, int8_t y, Direction dir) const;

    int8_t wall_votes_[MAZE_WIDTH][MAZE_HEIGHT][2]; // [0] = north wall of the cell, [1] = east wall
    bool   visited_[MAZE_WIDTH][MAZE_HEIGHT];
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

// Which way one cell is from its neighbour
Direction directionBetween(Coordinate from, Coordinate to);

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
    // Turns a cell path into the whole list of moves for a speed run (in sub_cmd_queue_).
    // Returns the time the run should take, or a negative number if it does not fit.
    float planSpeedRun(const Coordinate* path, uint8_t path_len, bool use_diagonals);
    void pushSubCommand(MotionAction action, float param, float max_speed, float accel,
                        bool wall_centering, float entry_speed = 0.0f, float exit_speed = 0.0f);
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

    uint8_t trap_recovery_;      // How far "walled in" recovery has gone: 0 none, 1 weak walls dropped, 2 map wiped

    void stepAtCellEdge(const IRReadings& ir, const WallPreview& preview);
    void stepAfterCurve(const WallPreview& preview);
    void driveToCellCentre();

    // Moves waiting to be sent to the motion controller, one each time the previous one finishes.
    // A speed run is planned in full up front, so this has to hold a whole run.
    static constexpr uint16_t MAX_SUB_CMDS = 520;
    MotionCommand sub_cmd_queue_[MAX_SUB_CMDS];
    uint16_t sub_cmd_count_;
    uint16_t sub_cmd_idx_;
    bool sub_cmd_overflow_;

};
