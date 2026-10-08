#pragma once

// N20 magnetic quadrature encoders, counted by the ESP32 pulse counters

#include "config.h"
#include "types.h"

// ==============================================================================
// ENCODERS
// ==============================================================================

#include <Arduino.h>
#include "driver/pcnt.h"

class Encoders {
public:
    Encoders();
    void begin();

    // Call every control loop tick (e.g. 1ms / 1kHz)
    void update(float dt_seconds);

    // Get current snapshot of encoder state
    EncoderState getState() const;

    // Reset distance & position accumulators
    void reset();

    // Invert channel direction if wiring is reversed
    void setInverted(bool invert_left, bool invert_right);
    void getInverted(bool& invert_left, bool& invert_right) const;

    // Wheel-speed smoothing weight (see ENCODER_SPEED_FILTER_ALPHA); adjustable for live tuning
    void setSpeedFilterAlpha(float alpha) { speed_filter_alpha_ = alpha; }
    float getSpeedFilterAlpha() const { return speed_filter_alpha_; }

    // Distance correction for live tuning: 1.05 makes every move 5 % longer on the floor.
    // Raise it if the robot stops short of a cell, lower it if it overshoots.
    void setDistanceScale(float scale) { distance_scale_ = scale; }
    float getDistanceScale() const { return distance_scale_; }

private:
    void initPcntUnit(pcnt_unit_t unit, int pin_a, int pin_b);

    EncoderState state_;
    float speed_filter_alpha_;
    float distance_scale_;
    bool invert_left_;
    bool invert_right_;

    float left_dist_acc_mm_;
    float right_dist_acc_mm_;
};
