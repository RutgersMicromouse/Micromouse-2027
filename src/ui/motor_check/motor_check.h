#pragma once

// Runs each wheel alone, both ways, and reports what the encoders and the heading did

#include <Arduino.h>

// ==============================================================================
// MOTOR CHECK
// ==============================================================================

// The app's "Motor check" button (console: "motorcheck"). For finding a motor or an encoder that
// runs the wrong way, or the two sides swapped. Four steps, each announced before it starts:
// left wheel forward, left wheel backward, right wheel forward, right wheel backward. Only the
// named wheel is driven, with a fixed effort and no feedback of any kind, so nothing can run away.
//
// After each step it prints a [MOTOR] line: how far each encoder says its wheel went, how far the
// heading moved, and what that means. At the end one [MOTORCHECK] line sums up.
//
//   Wheels off the floor: watch which wheel rolls and which way, and compare with what the step
//                         said. The encoders tell the rest.
//   On the floor:         the heading tells which wheel really moved and which way (left wheel
//                         forward swings the robot to the right, so the heading goes down).
//
// It blocks the caller (the operator task) for about 8 s. A STOP ends it at once.
namespace MotorCheck {

void run();

} // namespace MotorCheck
