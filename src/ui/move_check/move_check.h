#pragma once

// One button for stage C: finds which way each motor and encoder runs, then drives and turns

#include <Arduino.h>

// ==============================================================================
// MOVE CHECK
// ==============================================================================

// The app's "Movement check" button (console: "movecheck"). The robot must stand ON THE FLOOR with
// about 30 cm of clear space all round (not between walls). Nobody has to watch the wheels: the
// heading sensor says which way the robot really swung.
//
//   1. Each wheel alone gets a short forward push. The left wheel going forward swings the robot
//      to the right (heading down), the right wheel swings it to the left (heading up). From the
//      swing it knows which way each MOTOR really ran, and from the encoder's count which way
//      each ENCODER counts. Anything that is the wrong way round is switched over at once
//      (until the next restart) and the pushes are repeated to make sure.
//   2. Forward 180 mm (one cell), in a straight line.
//   3. 90 degrees left on the spot, then 90 degrees right.
//
// Every step prints a [MOVECHECK] line with the numbers and PASS or FAIL, and the last line says
// what the four INVERT_* settings in config.h must be. A STOP ends it at once.
namespace MoveCheck {

void run();

} // namespace MoveCheck
