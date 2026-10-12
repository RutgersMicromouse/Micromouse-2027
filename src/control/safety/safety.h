#pragma once

// The safety supervisor: watches every control tick and may slow or stop the robot, whatever
// the navigator and the motion controller have planned

#include "types.h"

// ==============================================================================
// SAFETY SUPERVISOR
// ==============================================================================

// Independent of the maze planner and of the motion controller. It is handed plain measurements
// once per control tick and answers with what it allows; it never asks for MORE motion than the
// controller wants, only for less. It knows nothing about pins, drivers or the maze, so the same
// code serves any chassis: only the numbers in config.h (SAFETY_*) and the measurements fed in
// are specific to a robot.
//
//   NORMAL    Nothing to object to.
//   CAUTION   Something it relies on is doubtful (heading sensor in fault, the two front sensors
//             disagree, supply low): new straight moves are slowed to SAFETY_CAUTION_SPEED_SCALE.
//   BRAKING   The front wall is inside the distance the robot needs to stop in. The move is
//             ended, the motors brake, and the run is aborted.
//   FAULT     The robot cannot be driven safely at all (motor driver not answering while a move
//             is on). Motors are held braked.
//   RECOVERY  After BRAKING or FAULT: waits until the robot has stood still and the cause has
//             gone, then returns to NORMAL. It never restarts a run: that takes the operator.
//
// Stopping distance (see docs): needed = v^2 / (2 a) + v t + margin, with a = SAFETY_BRAKE_DECEL,
// t = SAFETY_LATENCY_S. It is compared with the measured gap to the front wall less
// SAFETY_MIN_FRONT_GAP_MM. With no trustworthy gap there is no such check, and it says so.
class SafetySupervisor {
public:
    enum State : uint8_t { NORMAL = 0, CAUTION, BRAKING, FAULT, RECOVERY };

    enum Reason : uint8_t {
        REASON_NONE = 0,
        REASON_FRONT_WALL,        // BRAKING: wall inside the stopping distance
        REASON_CONTROLLER_STOP,   // BRAKING: the motion controller tripped one of its own stops
        REASON_HEADING_SENSOR,    // CAUTION: IMU missing or in fault, heading is on wheel odometry
        REASON_FRONT_SENSORS,     // CAUTION: one front sensor sees a wall the other does not
        REASON_LOW_SUPPLY,        // CAUTION: motor supply low
        REASON_MOTOR_DRIVER       // FAULT: driver not answering
    };

    // What it is told each control tick
    struct Inputs {
        float speed_mm_s;         // Forward speed from the wheels (negative = reversing)
        bool  front_gap_known;    // The front sensors give a distance that can be believed
        float front_gap_mm;       // Front sensor tips to the wall ahead (only if known)
        uint16_t front_left;      // The two front readings and the level that means "wall",
        uint16_t front_right;     //   for checking them against each other
        uint16_t front_wall_level;
        bool  heading_sensor_ok;  // IMU present and not in fault
        bool  motor_driver_ok;
        float supply_volts;       // 0 = not measured / motor power off
        bool  move_active;        // The controller is carrying out a move
        bool  controller_stopped; // The controller tripped a safety stop of its own this tick
    };

    // What it allows
    struct Outputs {
        bool brake_now;           // True for one tick: end the move and abort the run
        bool inhibit;             // Hold the motors braked; take no new moves
        float speed_scale;        // New straight moves may use this share of their speed (1 = all)
    };

    // For display and logs: one consistent copy of where it stands
    struct Status {
        State state;
        Reason reason;
        float speed_mm_s;
        float stop_needed_mm;     // v^2 / 2a + v t + margin at this speed
        float stop_room_mm;       // Front gap less the smallest gap allowed; -1 = not known
        float speed_scale;
        uint16_t brakings;        // Times it has braked the robot since power-on
        uint16_t faults;
        uint16_t changes;         // Counts every change of state (to notice one from another task)
    };

    SafetySupervisor();
    void reset();

    Outputs update(const Inputs& in, float dt_seconds);

    // Slows a straight move that is about to start, if the present state calls for it
    void limitCommand(MotionCommand& cmd) const;

    Status getStatus() const { return status_; }
    State getState() const { return status_.state; }

    static const char* stateName(State state);
    static const char* reasonText(Reason reason);

    // The stopping-distance formula on its own (also used by the PC test and the app's numbers)
    static float stoppingDistanceMm(float speed_mm_s);

    // The deceleration and the delay that sum counts on. They start at SAFETY_BRAKE_DECEL_MM_S2
    // and SAFETY_LATENCY_S (guesses); the braking test (LimitsTest::measureBraking) replaces
    // them with what this robot really manages.
    static void setBrakeModel(float decel_mm_s2, float latency_s) { brake_decel_mm_s2_ = decel_mm_s2; latency_s_ = latency_s; }
    static float getBrakeDecel() { return brake_decel_mm_s2_; }
    static float getLatency() { return latency_s_; }

private:
    void enter(State state, Reason reason);

    static float brake_decel_mm_s2_;
    static float latency_s_;
    Status status_;
    float in_state_s_;            // Time in the present state
    float front_trip_s_;          // How long the front wall has been inside the stopping distance
    float doubt_s_;               // How long a reason for CAUTION has been standing
    float clear_s_;               // How long there has been no reason for CAUTION
    float still_s_;               // How long the robot has stood still
    float driver_bad_s_;
    float front_mismatch_s_;
};
