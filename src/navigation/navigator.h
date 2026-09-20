#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "maze.h"
#include "floodfill.h"
#include "dijkstra.h"
#include "decomposer.h"
#include "types.h"

class Navigator {
public:
    Navigator(QueueHandle_t motion_cmd_queue, QueueHandle_t telemetry_queue);

    void begin();
    
    // Core 0 navigation step execution
    void step(const IRReadings& ir);
    void notifyMotionComplete();

    // High level state triggers
    void startSearchRun();
    void startReturnRun();
    void startSpeedRun(SpeedrunStrategy strategy = SPEEDRUN_HYBRID_AUTO);
    void stop();

    NavState getState() const;
    RobotPose getPose() const;
    const Maze& getMaze() const;
    Maze& getMaze();
    void clearSavedMaze();

private:
    void sendMotionCommand(MotionAction action, float param, float max_speed, float accel,
                           bool wall_centering = true, float entry_speed = 0.0f, float exit_speed = 0.0f);
    void updateCurrentCoordinates(Direction moved_dir);
    void turnInPlace(Direction target_dir, float speed, float accel);

    // Segment & Subcommand Execution Pipeline
    void queueSegment(const PathSegment& seg, float cruise_speed, float accel);
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

    // Subcommand queue for multi-phase motions (diagonals & slaloms)
    static constexpr uint8_t MAX_SUB_CMDS = 32;
    MotionCommand sub_cmd_queue_[MAX_SUB_CMDS];
    uint8_t sub_cmd_count_;
    uint8_t sub_cmd_idx_;

    // Segment queue for multi-segment routes (return & speedrun)
    static constexpr uint8_t MAX_SEGMENTS = 64;
    PathSegment segment_queue_[MAX_SEGMENTS];
    uint8_t segment_count_;
    uint8_t segment_idx_;
};
