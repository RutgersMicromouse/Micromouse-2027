#pragma once

// The state machine that decides where to go and feeds MotionCommands to the motion controller

#include "config.h"
#include "types.h"
#include "navigation/maze/maze.h"
#include "navigation/floodfill/floodfill.h"
#include "navigation/dijkstra/dijkstra.h"

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
#ifdef MAZE_SIZE_SWITCHABLE
    void changeMazeSize(int size); // 3, 5 or 16; loads that size's saved map and goes back to idle in the start cell
#endif

    // Core 0 navigation step execution
    // `preview` is what the motion controller saw of the cell ahead during the move that just ended
    void step(const IRReadings& ir, const WallPreview& preview = WallPreview());
    void notifyMotionComplete();

    // High level state triggers
    // Every run starts with the robot in the start cell facing into the maze.
    // The search keeps whatever map is already known (clearSavedMaze() forgets it), drives to the
    // centre, then explores its way back to the start.
    // With `one_move_at_a_time` the search is exactly the same, except that the robot stops at
    // every cell centre and waits for continueOneMove() before each move (phone app "Drive one cell").
    void startSearchRun(bool one_move_at_a_time = false);
    bool isWaitingForNextMove() const { return step_paused_; }

    // What the search did at the last cell edge and why (curved through, or drove to the centre)
    const char* getEdgeNote() const { return edge_note_; }
    void continueOneMove(const IRReadings& ir, const WallPreview& preview = WallPreview());
    void startSpeedRun(SpeedrunStrategy strategy = SPEEDRUN_HYBRID_AUTO);

    // For tuning speed-run turns: drives the given cell path exactly as a speed run would (same
    // planner, same moves, speeds from setSpeedScale), without looking at the map. The path
    // starts where the robot stands, and its first step must be the way the robot faces.
    // False if the path cannot be planned.
    bool startPathTest(const Coordinate* path, uint8_t path_len, bool use_diagonals);

    // After a speed run has finished in the goal cell: drive back to the start cell with the
    // search stepping (search speeds, sensors on, map kept up to date), and turn to face into the
    // maze there. Returns false, and does nothing, unless a speed run has just finished: only
    // then is it certain which cell the robot is standing in. Call step() afterwards to set off.
    bool startReturnToStart();
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

    const char* edge_note_;
    bool front_fix_done_;        // The front-wall distance check has already been done in this cell
    bool single_step_;           // This search waits for a command before every move
    bool step_allowed_;          // The next move has been asked for and not yet sent
    bool step_paused_;           // Standing at a cell centre, waiting to be asked

    uint8_t trap_recovery_;      // How far "walled in" recovery has gone: 0 none, 1 weak walls dropped, 2 map wiped

    void stepAtCellEdge(const IRReadings& ir, const WallPreview& preview);
    void stepAfterCurve(const IRReadings& ir, const WallPreview& preview);
    void driveToCellCentre();
    bool returnLegGoesStraightOn(Coordinate centre, Direction heading);
    float knownStraightSpeed(const Coordinate* path, uint8_t path_len, Direction heading) const;

    // Moves waiting to be sent to the motion controller, one each time the previous one finishes.
    // A speed run is planned in full up front, so this has to hold a whole run.
    static constexpr uint16_t MAX_SUB_CMDS = 520;
    MotionCommand sub_cmd_queue_[MAX_SUB_CMDS];
    uint16_t sub_cmd_count_;
    uint16_t sub_cmd_idx_;
    bool sub_cmd_overflow_;

};
