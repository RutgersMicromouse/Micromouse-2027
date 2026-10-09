#pragma once

// A question put to the operator on the phone app, answered there with Yes or No

#include <Arduino.h>

// ==============================================================================
// PROPOSAL
// ==============================================================================

// Whoever is reading the robot's recording (tools/run_recorder, over Bluetooth) can send a
// suggested change to the robot, which the phone app shows as a box with Yes and No buttons. The
// answer is printed as an [ANSWER] line, so it comes back through the same recording. The robot
// itself does nothing with either the text or the answer: it only carries them.
//
// Console commands (see Console):   ask new        start a new text (not shown yet)
//                                   ask+ <text>    add to it (a command line is short, so in pieces)
//                                   ask show       show it in the app
//                                   ask clear      take it down
//                                   yes / no       the app's two buttons
namespace Proposal {

void startNew();
void append(const char* text);
void show();
void clear();

// 0 = nothing to show. Otherwise the number of the question on show, which the answer repeats.
uint16_t shownId();
const char* text();

// Prints the [ANSWER] line and takes the question down. False if none was on show.
bool answer(bool yes);

} // namespace Proposal
