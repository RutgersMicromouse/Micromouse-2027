#pragma once

// The RGB LED on the ESP32 board (the robot's only indicator)

#include "types.h"

// ==============================================================================
// STATUS LED
// ==============================================================================

#include <Arduino.h>

// The RGB LED on the ESP32-S3 DevKit board: the robot's only indicator.
//
//   Green / Yellow / Cyan / Magenta = selected run mode (see Actions below)
//   White blink                     = a hand wave was counted
//   Red                             = stopped by a fault or low battery
namespace StatusLED {

struct Color {
    bool red;
    bool green;
    bool blue;
};

const Color OFF     = { false, false, false };
const Color RED     = { true,  false, false };
const Color GREEN   = { false, true,  false };
const Color BLUE    = { false, false, true  };
const Color YELLOW  = { true,  true,  false };
const Color CYAN    = { false, true,  true  };
const Color MAGENTA = { true,  false, true  };
const Color WHITE   = { true,  true,  true  };

void begin();

// Solid color
void set(Color color);

// Blink `count` times, ending with the LED off (blocks for count * 2 * delay_ms)
void flash(Color color, int count = 3, int delay_ms = 150);

} // namespace StatusLED
