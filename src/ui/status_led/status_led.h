#pragma once

// The RGB LED on the ESP32 board (the robot's only indicator)

#include "types.h"

// ==============================================================================
// STATUS LED
// ==============================================================================

#include <Arduino.h>

// The RGB LED on the ESP32-S3 DevKit board: the robot's only indicator.
//
//   Solid cyan (dim = wait)         = ready for a hand (see ui/gestures/gestures.h)
//   Yellow, brighter = faster       = choosing the speed of a speed run
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

// Solid color, at the usual brightness (RGB_BRIGHTNESS_LEVEL) or at one given (0-255)
void set(Color color);
void set(Color color, uint8_t level);

// Blink `count` times, ending with the LED off (blocks for count * 2 * delay_ms)
void flash(Color color, int count = 3, int delay_ms = 150);

} // namespace StatusLED
