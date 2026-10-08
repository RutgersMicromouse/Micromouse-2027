#pragma once

// Text console for debugging (USB, Bluetooth, Telnet, phone app)

#include "types.h"

// ==============================================================================
// TEXT COMMAND CONSOLE
// ==============================================================================

#include <Arduino.h>

// Debug console over Bluetooth, Wi-Fi Telnet, or the USB serial monitor: for watching what the
// robot is doing and adjusting settings on the bench. It can stop a run but never start one;
// the competition build has no radios at all. Send "help" for the list.
namespace Console {

// Read and carry out any commands that have arrived. Call from the navigation task.
void poll();

} // namespace Console
