#pragma once

#include <Arduino.h>
#include "config.h"
#include "types.h"
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

private:
    void initPcntUnit(pcnt_unit_t unit, int pin_a, int pin_b);

    EncoderState state_;
    bool invert_left_;
    bool invert_right_;

    float left_dist_acc_mm_;
    float right_dist_acc_mm_;
};
