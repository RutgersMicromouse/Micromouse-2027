#include "navigation/navigator/navigator.h"

// Start-up, starting and stopping runs, and sending moves. The search stepping is in
// navigator_search.cpp and the speed-run planner in navigator_speedrun.cpp.

// ==============================================================================
// NAVIGATOR
// ==============================================================================

Navigator::Navigator(QueueHandle_t motion_cmd_queue, QueueHandle_t telemetry_queue)
    : motion_cmd_queue_(motion_cmd_queue),
      telemetry_queue_(telemetry_queue),
      floodfill_(maze_),
      dijkstra_(maze_),
      state_(NAV_STATE_IDLE),
      current_strategy_(SPEEDRUN_HYBRID_AUTO),
      waiting_for_motion_(false),
      current_search_speed_(0.0f),
      speed_scale_(1.0f), best_route_explored_(false), maze_changed_(false),
      edge_note_("nothing yet"), single_step_(false), step_allowed_(false), step_paused_(false), trap_recovery_(0),
      search_phase_(PHASE_AT_CENTRE), front_fix_done_(false),
      curve_cell_x_(0), curve_cell_y_(0), curve_entry_dir_(DIR_NORTH), curve_cell_was_known_(false),
      sub_cmd_count_(0),
      sub_cmd_idx_(0),
      sub_cmd_overflow_(false) {
    memset(&pose_, 0, sizeof(pose_));
    pose_.cell_x = 0;
    pose_.cell_y = 0;
    pose_.current_dir = DIR_NORTH;
}

#ifdef MAZE_SIZE_SWITCHABLE
// Switch between the 3x3 practice maze and a full 16x16 one. Each size has its own saved map,
// which is loaded here; the robot is taken to be in the start cell facing into the maze.
// (Never called during a run: Actions::setMazeSize refuses.)
void Navigator::changeMazeSize(int size) {
    g_maze_size = (size == 16) ? 16 : 3;
    Maze::saveSizeToNVS();
    begin();
    pose_.cell_x = 0;
    pose_.cell_y = 0;
    pose_.current_dir = DIR_NORTH;
    search_phase_ = PHASE_AT_CENTRE;
    maze_changed_ = false;
    best_route_explored_ = false;
    Serial.printf("[NAV] Maze size is now %dx%d. Position back to the start cell (0, 0).\n", g_maze_size, g_maze_size);
}
#endif

void Navigator::begin() {
#ifdef MAZE_SIZE_SWITCHABLE
    Maze::loadSizeFromNVS();
#endif
    maze_.reset();
    if (maze_.loadFromNVS()) {
        Serial.println("[NAV] Found previously saved maze in Flash! Loaded successfully for instant Speedrun.");
    }
    floodfill_.setGoalToCenter();
    floodfill_.recalculate();
    state_ = NAV_STATE_IDLE;
    waiting_for_motion_ = false;
    current_search_speed_ = 0.0f;
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;
}

void Navigator::startSearchRun(bool one_move_at_a_time) {
    // The map is NOT wiped: everything learned in earlier attempts (including ones that ended in
    // a crash) is kept, so each search starts from what is already known and explores further.
    single_step_ = one_move_at_a_time;
    step_allowed_ = one_move_at_a_time; // The command that started the run asks for the first move
    step_paused_ = false;
    trap_recovery_ = 0;
    maze_.setVisited(0, 0); // Start cell (0,0) is visited
    floodfill_.setGoalToCenter();
    floodfill_.recalculate();
    pose_.cell_x = 0;
    pose_.cell_y = 0;
    pose_.current_dir = DIR_NORTH;
    state_ = NAV_STATE_EXPLORING_TO_CENTER;
    search_phase_ = PHASE_AT_CENTRE;
    waiting_for_motion_ = false;
    current_search_speed_ = 0.0f;
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;
    Serial.println("[NAV] Starting search run to the maze centre (keeping the known map).");
}

bool Navigator::startReturnToStart() {
    if (state_ != NAV_STATE_FINISHED) return false;

    // The same as the return leg of a search, begun from where the speed run ended
    single_step_ = false;
    step_allowed_ = false;
    step_paused_ = false;
    trap_recovery_ = 0;
    floodfill_.setGoalToStart();
    floodfill_.recalculate();
    state_ = NAV_STATE_RETURNING_TO_START;
    search_phase_ = PHASE_AT_CENTRE;
    waiting_for_motion_ = false;
    current_search_speed_ = 0.0f;
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;
    Serial.printf("[NAV] Returning to the start cell from (%d, %d).\n", pose_.cell_x, pose_.cell_y);
    return true;
}

void Navigator::saveMazeIfChanged() {
    // Writing flash briefly stalls the processor, so this is only called while standing still
    if (maze_changed_) {
        maze_.saveToNVS();
        maze_changed_ = false;
    }
}

// Follows the floodfill from the start cell to the centre with unexplored cells treated as open.
// If that route never leaves explored cells, no shorter route can be hiding in the unknown.
bool Navigator::checkBestRouteExplored() {
    int8_t x = 0, y = 0;
    Direction heading = DIR_NORTH;
    for (int i = 0; i < 255; ++i) {
        if (!maze_.isVisited(x, y)) return false;
        if (floodfill_.isAtGoal(x, y)) return true;
        Direction next = floodfill_.getNextDirection(x, y, heading);
        if (next == DIR_INVALID) return false;
        if (!maze_.isKnownOpen(x, y, next)) return false; // The way on has never actually been seen open
        heading = next;
        if (next == DIR_NORTH) y++;
        else if (next == DIR_EAST)  x++;
        else if (next == DIR_SOUTH) y--;
        else if (next == DIR_WEST)  x--;
    }
    return false;
}

void Navigator::stop() {
    // Keep what this run learned, even if it ended badly: the next search carries on from it
    if (state_ == NAV_STATE_EXPLORING_TO_CENTER || state_ == NAV_STATE_RETURNING_TO_START) {
        saveMazeIfChanged();
    }
    state_ = NAV_STATE_IDLE;
    search_phase_ = PHASE_AT_CENTRE;
    single_step_ = false;
    step_paused_ = false;
    sub_cmd_count_ = 0;
    sub_cmd_idx_ = 0;
    sendMotionCommand(ACTION_EMERGENCY_STOP, 0.0f, 0.0f, 0.0f);
}

void Navigator::notifyMotionComplete() {
    waiting_for_motion_ = false;
    processSubcommandQueue();
}

void Navigator::sendMotionCommand(MotionAction action, float param, float max_speed, float accel,
                                  bool wall_centering, float entry_speed, float exit_speed,
                                  bool stop_at_front_wall) {
    MotionCommand cmd;
    cmd.action = action;
    cmd.param_value = param;
    cmd.max_speed_mm_s = max_speed;
    cmd.acceleration = accel;
    cmd.enable_wall_centering = wall_centering;
    cmd.entry_speed_mm_s = entry_speed;
    cmd.exit_speed_mm_s = exit_speed;
    cmd.start_offset_mm = 0.0f;
    cmd.stop_at_front_wall = stop_at_front_wall;

    xQueueSend(motion_cmd_queue_, &cmd, portMAX_DELAY);
    waiting_for_motion_ = true;
    step_allowed_ = false; // One move at a time: this was the move that was asked for
}

void Navigator::processSubcommandQueue() {
    if (sub_cmd_idx_ < sub_cmd_count_) {
        // Send next sub-command in current segment
        MotionCommand next_cmd = sub_cmd_queue_[sub_cmd_idx_++];
        xQueueSend(motion_cmd_queue_, &next_cmd, portMAX_DELAY);
        waiting_for_motion_ = true;
        step_allowed_ = false;
        return;
    }

    // Nothing left to send: a speed run ends here (the search carries on in step())
    if (state_ == NAV_STATE_SPEED_RUNNING) {
        Serial.println("[NAV] Speed run complete: centre reached.");
        state_ = NAV_STATE_FINISHED;
    }
}

void Navigator::pushSubCommand(MotionAction action, float param, float max_speed, float accel,
                               bool wall_centering, float entry_speed, float exit_speed) {
    if (sub_cmd_count_ >= MAX_SUB_CMDS) {
        sub_cmd_overflow_ = true;
        return;
    }
    sub_cmd_queue_[sub_cmd_count_++] = { action, param, max_speed, accel, wall_centering, entry_speed, exit_speed, 0.0f, false };
}

NavState Navigator::getState() const {
    return state_;
}

RobotPose Navigator::getPose() const {
    return pose_;
}

const Maze& Navigator::getMaze() const {
    return maze_;
}

Maze& Navigator::getMaze() {
    return maze_;
}

void Navigator::clearSavedMaze() {
    maze_.clearNVS();
    maze_.reset();
    floodfill_.setGoalToCenter();
    floodfill_.recalculate();

    // With no map the robot is back to square one: start cell, facing into the maze, nothing
    // ready to run. (Never called during a run: Actions::clearSavedMaze refuses.)
    pose_.cell_x = 0;
    pose_.cell_y = 0;
    pose_.current_dir = DIR_NORTH;
    state_ = NAV_STATE_IDLE;
    search_phase_ = PHASE_AT_CENTRE;
    maze_changed_ = false;
    best_route_explored_ = false;
    Serial.println("[NAV] Flash NVS maze cleared, grid reset, position back to the start cell (0, 0).");
}
