#pragma once

#include <Motoron.h>
#include <Encoder.h>

// call encLeft.read() and encRight.read() to get encoder values
extern Encoder encLeft;
extern Encoder encRight;

void motorSetup();

// Removed IRAM_ATTR because Teensy doesn't use that macro
void updateRightEncoder();
void updateLeftEncoder();

void setLeftPWM(int PWM);
void setRightPWM(int PWM);