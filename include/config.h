#pragma once

#include <Arduino.h>

// ==============================================================================
// 1. HARDWARE PIN DEFINITIONS (From Schematic: antigravitieee rev 1.0)
// ==============================================================================

// Faulhaber Encoders (Buffered through SN74LVC125ANS U1, into ESP32 Hardware PCNT)
// Left Motor Encoder (M2 on Motoron)
#define PIN_ENC_L_A            6        // L1_OUT (from LENC1 via U1 pin 3Y)
#define PIN_ENC_L_B            7        // L2_OUT (from LENC2 via U1 pin 4Y)

// Right Motor Encoder (M1 on Motoron)
#define PIN_ENC_R_A            4        // R1_OUT (from RENC1 via U1 pin 1Y)
#define PIN_ENC_R_B            5        // R2_OUT (from RENC2 via U1 pin 2Y)

// Motor Controller: Pololu Motoron M2T256 (I2C)
#define PIN_MOTOR_RST          11       // MRST: Active-low reset line for Motoron (GPIO11 in schematic)
#define PIN_MOTOR_BAT_CTRL     13       // MotorBatControl: NPN drive to PMOS for 12V boost converter (Active HIGH)
#define MOTORON_I2C_ADDR       16       // Default Pololu Motoron I2C address (0x10)
#define MOTORON_MAX_SPEED      800      // Max speed units for Motoron M2T256

// Shared I2C Bus (BNO055 IMU + Motoron M2T256)
#define PIN_I2C_SDA            21       // Pulled up by R13 3.3k to 3.3V
#define PIN_I2C_SCL            20       // Pulled up by R19 3.3k to 3.3V
#define I2C_CLOCK_SPEED        400000   // Fast Mode I2C (400 kHz)

// IMU: Bosch BNO055 (I2C)
#define BNO055_I2C_ADDR        0x28     // Standard I2C address for BNO055 (or 0x29)

// 6-Channel IR Emitter & Receiver System (SFH4545 Emitters + TEFT4300 Phototransistors)
// Emitters (Driven by AO3400A N-MOSFETs with 10k pulldown)
#define PIN_IR_E1              15       // Emitter 1: Left 90°
#define PIN_IR_E2              16       // Emitter 2: Front-Left 45°
#define PIN_IR_E3              17       // Emitter 3: Front-Left Center
#define PIN_IR_E4              18       // Emitter 4: Front-Right Center
#define PIN_IR_E5              19       // Emitter 5: Front-Right 45°
#define PIN_IR_E6              14       // Emitter 6: Right 90°

// Receivers (TEFT4300 with 10k load to GND, sampled on ADC1)
#define PIN_IR_R1              3        // Receiver 1: Left 90° (ADC1_CH2)
#define PIN_IR_R2              8        // Receiver 2: Front-Left 45° (ADC1_CH7)
#define PIN_IR_R3              1        // Receiver 3: Front-Left Center (ADC1_CH0)
#define PIN_IR_R4              2        // Receiver 4: Front-Right Center (ADC1_CH1)
#define PIN_IR_R5              9        // Receiver 5: Front-Right 45° (ADC1_CH8)
#define PIN_IR_R6              10       // Receiver 6: Right 90° (ADC1_CH9)

#define IR_PULSE_SETTLE_US     30       // Phototransistor rise & settling time (microseconds)

// State Controls & UI
#define PIN_BTN_STATE          42       // State Button (SV4 Header, R6 10k pull-up to 3.3V, Active LOW)
#define PIN_BTN_CONFIRM        41       // Confirm Button (SV3 Header, R7 10k pull-up to 3.3V, Active LOW)

// RGB Status LED (TJ-L5FCMXHTCSLCRGB-A5 Common Cathode to GND)
#define PIN_LED_RED            39       // Active HIGH (GPIO 39 in schematic)
#define PIN_LED_GREEN          38       // Active HIGH (GPIO 38 in schematic)
#define PIN_LED_BLUE           37       // Active HIGH (GPIO 37 in schematic)

// Battery Voltage Monitoring
#define PIN_VSENSE_COM         12       // Computer Battery divider (R40=10k, R39=10k -> 2.0x divider)
#define BATTERY_DIVIDER_RATIO  2.0f     // (10k + 10k) / 10k
#define BATTERY_MIN_SAFE_VOLT  3.3f     // Safe low-voltage cutoff (V)

// ==============================================================================
// 2. ROBOT PHYSICAL & KINEMATIC PARAMETERS
// ==============================================================================

#define WHEEL_DIAMETER_MM      24.0f    // Typical micromouse wheel diameter (mm)
#define WHEEL_BASE_MM          72.0f    // Distance between left and right wheels (mm)

// Faulhaber 1524 Motors with Integrated Encoders
#define ENCODER_GEAR_RATIO     1.0f     // Update if using external gear reduction
#define ENCODER_CPR_RAW        16.0f    // Faulhaber encoder base pulses per rev
#define ENCODER_TOTAL_CPR      (ENCODER_CPR_RAW * 4.0f * ENCODER_GEAR_RATIO) // 4x quadrature decoding

#define MM_PER_TICK            ((PI * WHEEL_DIAMETER_MM) / ENCODER_TOTAL_CPR)
#define TICKS_PER_MM           (1.0f / MM_PER_TICK)

// Search & Exploration Kinematic Limits (Safe bringup defaults)
#define SEARCH_SPEED_DEFAULT_MM_S     240.0f   // Safe bringup search cruise speed (mm/s)
#define SEARCH_ACCEL_DEFAULT_MM_S2    1500.0f  // Search linear acceleration / deceleration (mm/s^2)
#define SEARCH_CURVE_SPEED_MM_S       200.0f   // 90° corner arc speed during continuous search (mm/s)
#define SEARCH_TURN_SPEED_DEG_S       360.0f   // In-place turn speed (deg/s)
#define SEARCH_TURN_ACCEL_DEG_S2      1800.0f  // In-place turn acceleration (deg/s^2)

// ==============================================================================
// 3. MAZE GEOMETRY
// ==============================================================================

#define MAZE_WIDTH             16
#define MAZE_HEIGHT            16
#define MAZE_CELL_SIZE_MM      180.0f   // Standard micromouse cell dimension (180 mm)
#define HALF_CELL_SIZE_MM      (MAZE_CELL_SIZE_MM / 2.0f)

// ==============================================================================
// 4. CONTROL LOOP TIMING & PRIORITIES
// ==============================================================================

#define CONTROL_LOOP_FREQ_HZ   500      // 500 Hz control loop (well suited for 400kHz I2C motor updates)
#define CONTROL_DT_S           (1.0f / (float)CONTROL_LOOP_FREQ_HZ)

#define CORE_MOTION_CONTROL    1        // Core 1 dedicated to Motion PID, Encoders & Sensors
#define CORE_NAVIGATION        0        // Core 0 handles Floodfill, Path Planning, Telemetry

#define PRIORITY_MOTION_TASK   24       // Highest application priority
#define PRIORITY_NAV_TASK      5        // Planning task priority
#define PRIORITY_TELEMETRY     2        // Low-priority logging

// ==============================================================================
// 5. DEFAULT SENSOR THRESHOLDS (5 Sensors)
// ==============================================================================

#define WALL_THRESH_L90        300      // Left 90° sensor presence threshold
#define WALL_THRESH_L45        350      // Front-Left 45° sensor presence threshold
#define WALL_THRESH_FRONT      400      // Front Center 0° sensor presence threshold
#define WALL_THRESH_R45        350      // Front-Right 45° sensor presence threshold
#define WALL_THRESH_R90        300      // Right 90° sensor presence threshold

#define NOMINAL_CENTER_L45     850      // Expected reading for FL45 when centered
#define NOMINAL_CENTER_R45     850      // Expected reading for FR45 when centered

// ==============================================================================
// 6. WIRELESS TELEMETRY & BLE DEBUGGING
// ==============================================================================

#define ENABLE_BLE_DEBUG       1        // 1 = Nordic UART Service active, 0 = RF disabled for competition
#define BLE_DEVICE_NAME        "Antigrav-Mouse"

// ==============================================================================
// 7. WIRELESS OTA (OVER-THE-AIR) FLASHING & TELNET DEBUGGING
// ==============================================================================

#define ENABLE_WIFI_OTA        1        // 1 = Wi-Fi OTA & Telnet Active, 0 = RF disabled for competition
#define WIFI_AP_MODE           1        // 1 = Broadcast Hotspot (SoftAP), 0 = Connect to Local Wi-Fi (STA)
#define WIFI_AP_SSID           "Antigrav-Mouse"
#define WIFI_AP_PASS           "micromouse"   // WPA2 Passphrase (minimum 8 chars)
#define WIFI_STA_SSID          "YourHomeWiFi" // Used if WIFI_AP_MODE = 0
#define WIFI_STA_PASS          "YourPassword" // Used if WIFI_AP_MODE = 0
#define OTA_PORT               3232
#define TELNET_PORT            23

