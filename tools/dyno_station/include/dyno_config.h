#pragma once

#include <Arduino.h>

// ==============================================================================
// 1. HARDWARE PIN DEFINITIONS (Dyno Station Standalone ESP32)
// ==============================================================================

// Optical IR Slot Sensor / Photogate Inputs (Active LOW when beam interrupted, or active HIGH)
#define PIN_GATE_LEFT             18       // Left wheel optical slit sensor (GPIO18, Interrupt on FALLING/RISING)
#define PIN_GATE_RIGHT            19       // Right wheel optical slit sensor (GPIO19, Interrupt on FALLING/RISING)
#define PIN_STATUS_LED            2        // Onboard Status LED

// Debounce lockout threshold to eliminate optical edge bounce (microseconds)
#define OPTICAL_DEBOUNCE_US       1500     // At 2000 RPM (33 rev/s), 1 slit = 30,000 µs period. 1500 µs is completely safe.

// ==============================================================================
// 2. PHYSICAL KINEMATIC SPECIFICATIONS
// ==============================================================================

#define WHEEL_DIAMETER_MM         24.0f    // Calibrated micromouse wheel diameter (mm)
#define SLITS_PER_WHEEL           1        // 1 narrow radial slit per wheel for 360° phase detection

// ==============================================================================
// 3. WIRELESS BLE TARGET CONFIGURATION (Robot Connection)
// ==============================================================================

#define BLE_TARGET_DEVICE_NAME    "Antigrav-Mouse"
#define NORDIC_UART_SERVICE_UUID  "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NORDIC_UART_CHAR_RX_UUID  "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"  // Characteristic we WRITE commands to
#define NORDIC_UART_CHAR_TX_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"  // Characteristic robot NOTIFIES replies on

// ==============================================================================
// 4. AUTO-TUNER ALGORITHM PARAMETERS
// ==============================================================================

// Convergence thresholds
#define MAX_ALLOWED_RPM_DIFF      1.5f     // Steady-state RPM difference tolerance between wheels
#define MAX_ALLOWED_PHASE_DRIFT   0.5f     // Maximum allowable angular drift across a full sprint (degrees)
#define MAX_OPTIMIZATION_STEPS    12       // Maximum iterations per tuning stage
