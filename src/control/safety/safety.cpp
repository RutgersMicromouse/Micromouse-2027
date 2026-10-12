#include <math.h>
#include "config.h"
#include "control/safety/safety.h"

// ==============================================================================
// SAFETY SUPERVISOR
// ==============================================================================

SafetySupervisor::SafetySupervisor() {
    reset();
}

void SafetySupervisor::reset() {
    status_ = {};
    status_.state = NORMAL;
    status_.reason = REASON_NONE;
    status_.stop_room_mm = -1.0f;
    status_.speed_scale = 1.0f;
    in_state_s_ = front_trip_s_ = doubt_s_ = clear_s_ = still_s_ = driver_bad_s_ = front_mismatch_s_ = 0.0f;
}

const char* SafetySupervisor::stateName(State state) {
    switch (state) {
        case NORMAL:   return "NORMAL";
        case CAUTION:  return "CAUTION";
        case BRAKING:  return "BRAKING";
        case FAULT:    return "FAULT";
        case RECOVERY: return "RECOVERY";
        default:       return "?";
    }
}

const char* SafetySupervisor::reasonText(Reason reason) {
    switch (reason) {
        case REASON_FRONT_WALL:      return "front wall inside the stopping distance";
        case REASON_CONTROLLER_STOP: return "the motion controller stopped the move (stall, encoder, heading or tilt)";
        case REASON_HEADING_SENSOR:  return "heading sensor in fault: steering on the wheels alone";
        case REASON_FRONT_SENSORS:   return "the two front sensors disagree";
        case REASON_LOW_SUPPLY:      return "motor supply low";
        case REASON_MOTOR_DRIVER:    return "motor driver not answering";
        default:                     return "nothing";
    }
}

// needed = v^2 / (2 a)   the distance braking at a takes
//        + v t           what is covered before the brake bites (sensing + control + driver)
//        + margin        for what the two terms above get wrong
float SafetySupervisor::brake_decel_mm_s2_ = SAFETY_BRAKE_DECEL_MM_S2;

float SafetySupervisor::stoppingDistanceMm(float speed_mm_s) {
    if (speed_mm_s <= 0.0f) return SAFETY_STOP_MARGIN_MM;
    return speed_mm_s * speed_mm_s / (2.0f * brake_decel_mm_s2_)
         + speed_mm_s * SAFETY_LATENCY_S
         + SAFETY_STOP_MARGIN_MM;
}

void SafetySupervisor::enter(State state, Reason reason) {
    if (state == status_.state && reason == status_.reason) return;
    if (state == BRAKING && status_.state != BRAKING) status_.brakings++;
    if (state == FAULT   && status_.state != FAULT)   status_.faults++;
    status_.state = state;
    status_.reason = reason;
    status_.changes++;
    in_state_s_ = 0.0f;
}

SafetySupervisor::Outputs SafetySupervisor::update(const Inputs& in, float dt) {
    in_state_s_ += dt;
    status_.speed_mm_s = in.speed_mm_s;
    status_.stop_needed_mm = stoppingDistanceMm(in.speed_mm_s);
    status_.stop_room_mm = in.front_gap_known ? (in.front_gap_mm - SAFETY_MIN_FRONT_GAP_MM) : -1.0f;

    // ---------------------------------------------------------------- What is wrong, if anything
    // Front wall inside the stopping distance, while driving forward. It has to hold for a few
    // ticks: one reading is one sample of a noisy sensor.
    const bool front_danger = in.front_gap_known && in.speed_mm_s > SAFETY_MIN_SPEED_MM_S &&
                              status_.stop_needed_mm > status_.stop_room_mm;
    front_trip_s_ = front_danger ? front_trip_s_ + dt : 0.0f;

    // One front sensor sees a wall and the other sees next to nothing: one of them is not working,
    // and a dead sensor must not be taken for an open corridor
    const uint16_t front_hi = (in.front_left > in.front_right) ? in.front_left : in.front_right;
    const uint16_t front_lo = (in.front_left > in.front_right) ? in.front_right : in.front_left;
    const bool mismatch = in.front_wall_level > 0 && front_hi > in.front_wall_level &&
                          (float)front_lo < SAFETY_FRONT_PAIR_MIN_SHARE * (float)front_hi;
    front_mismatch_s_ = mismatch ? front_mismatch_s_ + dt : 0.0f;

    driver_bad_s_ = (!in.motor_driver_ok && in.move_active) ? driver_bad_s_ + dt : 0.0f;
    still_s_ = (fabsf(in.speed_mm_s) < SAFETY_STILL_SPEED_MM_S) ? still_s_ + dt : 0.0f;

    Reason doubt = REASON_NONE;
    if (!in.heading_sensor_ok)                              doubt = REASON_HEADING_SENSOR;
    else if (front_mismatch_s_ > SAFETY_PAIR_MISMATCH_S)    doubt = REASON_FRONT_SENSORS;
    else if (in.supply_volts > 1.0f && in.supply_volts < SAFETY_LOW_SUPPLY_V) doubt = REASON_LOW_SUPPLY;
    doubt_s_ = (doubt != REASON_NONE) ? doubt_s_ + dt : 0.0f;
    clear_s_ = (doubt == REASON_NONE) ? clear_s_ + dt : 0.0f;

    // ---------------------------------------------------------------- The state machine
    Outputs out = { false, false, 1.0f };
    const State before = status_.state;

    switch (status_.state) {
        case NORMAL:
        case CAUTION:
            if (driver_bad_s_ > SAFETY_DRIVER_FAULT_S) {
                enter(FAULT, REASON_MOTOR_DRIVER);
            } else if (front_trip_s_ > SAFETY_FRONT_TRIP_S) {
                enter(BRAKING, REASON_FRONT_WALL);
            } else if (in.controller_stopped) {
                enter(BRAKING, REASON_CONTROLLER_STOP);
            } else if (doubt_s_ > SAFETY_CAUTION_ON_S) {
                enter(CAUTION, doubt);
            } else if (status_.state == CAUTION && clear_s_ > SAFETY_CAUTION_OFF_S) {
                enter(NORMAL, REASON_NONE);
            }
            break;

        case BRAKING:
            // Until it has really stopped (or, if the wheels never report standing still, a time limit)
            if (still_s_ > SAFETY_STILL_S || in_state_s_ > SAFETY_BRAKING_MAX_S) {
                enter(RECOVERY, status_.reason);
            }
            break;

        case FAULT:
            if (in.motor_driver_ok && still_s_ > SAFETY_STILL_S) {
                enter(RECOVERY, status_.reason);
            }
            break;

        case RECOVERY:
            if (!in.motor_driver_ok) {
                enter(FAULT, REASON_MOTOR_DRIVER);
            } else if (in_state_s_ > SAFETY_RECOVERY_S && still_s_ > SAFETY_STILL_S) {
                if (doubt != REASON_NONE) enter(CAUTION, doubt);
                else                      enter(NORMAL, REASON_NONE);
                front_trip_s_ = 0.0f;
            }
            break;
    }

    out.brake_now = (status_.state == BRAKING || status_.state == FAULT) && before != status_.state;
    out.inhibit = (status_.state == BRAKING || status_.state == FAULT || status_.state == RECOVERY);
    out.speed_scale = (status_.state == CAUTION) ? SAFETY_CAUTION_SPEED_SCALE : 1.0f;
    status_.speed_scale = out.inhibit ? 0.0f : out.speed_scale;
    return out;
}

void SafetySupervisor::limitCommand(MotionCommand& cmd) const {
    if (status_.state != CAUTION) return;
    const bool straight = (cmd.action == ACTION_MOVE_FORWARD_CELLS || cmd.action == ACTION_MOVE_DISTANCE ||
                           cmd.action == ACTION_MOVE_HALF_CELL || cmd.action == ACTION_MOVE_DIAGONAL_HALF);
    if (!straight) return; // (a turn's "speed" is in degrees per second and is left alone)
    cmd.max_speed_mm_s *= SAFETY_CAUTION_SPEED_SCALE;
    // A move that hands over at speed may not be left faster than it is now allowed to go
    if (cmd.exit_speed_mm_s > cmd.max_speed_mm_s) cmd.exit_speed_mm_s = cmd.max_speed_mm_s;
}
