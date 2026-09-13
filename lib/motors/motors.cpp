#include "motors.h"

MotoronI2C mc;

// Teensy 4.0 handles these pins automatically with interrupts
Encoder encLeft(14, 15);
Encoder encRight(16, 17);

void motorSetup() {
    mc.reinitialize();
    mc.disableCrc();
    mc.clearResetFlag();
    
    // Note: With the PaulStoffregen/Encoder library, you do NOT 
    // need attachInterrupt() or manual update functions. 
    // It works automatically on Pins 1,2,3,4.
    mc.setMaxAcceleration(1, 200);
    mc.setMaxAcceleration(2, 200);  // was 1, now 2
    mc.setMaxDeceleration(1, 200);
    mc.setMaxDeceleration(2, 200);  // was 1, now 2

    Serial.println("MOTOR SETUP DONE");
}

// These are no longer needed for the Encoder library, 
// but we keep them empty if other files expect them to exist.
void updateRightEncoder() {}
void updateLeftEncoder() {}

void setLeftPWM(int PWM) {
    mc.setSpeed(1, PWM);
}

void setRightPWM(int PWM) {
    mc.setSpeed(2, -PWM);
}
