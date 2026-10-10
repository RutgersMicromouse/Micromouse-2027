#pragma once

// Which stage of the owner's tuning plan the team is on, kept across restarts

#include <Arduino.h>

// ==============================================================================
// STAGES
// ==============================================================================

// The owner's plan for getting the robot ready runs in stages A to G (see AGENTS.md), and nobody
// moves on until the owner says a stage is finished. This keeps the place: which stage is being
// worked on and which are finished. The phone app shows it as a checklist with a button per stage,
// and every change is printed as a [STAGE] line, so the Bluetooth recording shows it too.
//
// The robot does nothing differently from one stage to the next: this is a bookmark only.
//
// Console commands (see Console):   stage            say where things stand
//                                   stage c          work on stage C. Going back to a finished
//                                                    stage un-finishes it and every later one.
//                                   stage done c     stage C is finished; move to the next one
namespace Stages {

static const int COUNT = 7; // A..G

void begin();               // Loads the saved place from flash

int current();              // 0 = A ... 6 = G
uint8_t doneMask();         // Bit 0 = A finished, bit 1 = B finished, ...

void goTo(int stage);       // Prints the [STAGE] line and saves
void markDone(int stage);   // The same, then moves on to the next stage
void report();              // Prints where things stand

} // namespace Stages
