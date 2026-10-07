#pragma once

#include <Arduino.h>

// ==============================================================================
// 1. HARDWARE PIN DEFINITIONS (From Schematic: antigravitieee rev 1.0)
// ==============================================================================

// N20 Motor Magnetic Encoders (Buffered through SN74LVC125ANS U1 / ESP32 Hardware PCNT)
// Left Motor Encoder (M2 on Motoron)
#define PIN_ENC_L_A            6        // L1_OUT (from LENC1 via U1 pin 3Y)
#define PIN_ENC_L_B            7        // L2_OUT (from LENC2 via U1 pin 4Y)

// Right Motor Encoder (M1 on Motoron)
#define PIN_ENC_R_A            4        // R1_OUT (from RENC1 via U1 pin 1Y)
#define PIN_ENC_R_B            5        // R2_OUT (from RENC2 via U1 pin 2Y)

// Motor & Encoder Polarity Inversion (Set true/false to match physical N20 wiring)
#define INVERT_LEFT_MOTOR      false
#define INVERT_RIGHT_MOTOR     false
#define INVERT_LEFT_ENCODER    false
#define INVERT_RIGHT_ENCODER   false

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
#define PIN_IR_E1              15       // Emitter 1: Left 90° (Pin 8)
#define PIN_IR_E2              16       // Emitter 2: Front-Left 45° (Pin 9)
#define PIN_IR_E3              17       // Emitter 3: Front-Left Center (Pin 10)
#define PIN_IR_E4              14       // Emitter 4: Front-Right Center (Pin 11)
#define PIN_IR_E5              48       // Emitter 5: Front-Right 45° (Pin 14, GPIO46 in schematic)
#define PIN_IR_E6              19       // Emitter 6: Right 90° (Pin 20)

// Receivers (TEFT4300 with 10k load to GND, sampled on ADC1)
#define PIN_IR_R1              3        // Receiver 1: Left 90° (Pin 12, GPIO8 / ADC1_CH7 in schematic)
#define PIN_IR_R2              8        // Receiver 2: Front-Left 45° (Pin 13, GPIO3 / ADC1_CH2 in schematic)
#define PIN_IR_R3              1        // Receiver 3: Front-Left Center (Pin 26, GPIO1 / ADC1_CH0 in schematic)
#define PIN_IR_R4              10        // Receiver 4: Front-Right Center (Pin 27, GPIO2 / ADC1_CH1 in schematic)
#define PIN_IR_R5              2        // Receiver 5: Front-Right 45° (Pin 15, GPIO9 / ADC1_CH8 in schematic)
#define PIN_IR_R6              9        // Receiver 6: Right 90° (Pin 16, GPIO10 / ADC1_CH9 in schematic)

#define IR_PULSE_SETTLE_US     30       // Phototransistor rise & settling time (microseconds)

// State Controls & UI
#define PIN_BTN_STATE          42       // State Button (SV4 Header, R6 10k pull-up to 3.3V, Active LOW)
#define PIN_BTN_CONFIRM        41       // Confirm Button (SV3 Header, R7 10k pull-up to 3.3V, Active LOW)

// RGB Status LED: ESP32-S3-DevKitC-1 Onboard WS2812/NeoPixel (GPIO 48)
// Note: External PCB RGB LED on GPIO 39, 38, 37 is defective/inactive on this PCB revision.
#define PIN_ESP32_RGB_LED      48       // Onboard WS2812 NeoPixel on ESP32-S3-DevKitC-1 (RGB_BUILTIN)
#define RGB_BRIGHTNESS_LEVEL   40       // Brightness level (0-255, 40 provides vibrant color without glare)

// Battery Voltage Monitoring
#define PIN_VSENSE_COM         12       // Computer Battery divider (R40=10k, R39=10k -> 2.0x divider)
#define BATTERY_DIVIDER_RATIO  2.0f     // (10k + 10k) / 10k
#define BATTERY_MIN_SAFE_VOLT  3.3f     // Computer battery is a separate 1S 3.7V cell

// ==============================================================================
// 2. ROBOT PHYSICAL & KINEMATIC PARAMETERS
// ==============================================================================

#define WHEEL_DIAMETER_MM      24.0f    // Typical micromouse wheel diameter (mm)
#define WHEEL_BASE_MM          72.0f    // Distance between left and right wheels (mm)

// N20 12V Micro Metal Gearmotors with Magnetic Encoders
#define ENCODER_GEAR_RATIO     30.0f    // 30:1 metal gear reduction ratio
#define ENCODER_CPR_RAW        7.0f     // 7 pulses per channel per motor shaft rev
#define ENCODER_TOTAL_CPR      (ENCODER_CPR_RAW * 4.0f * ENCODER_GEAR_RATIO) // 840.0 ticks/wheel rev (4x quadrature decoding)

#define MM_PER_TICK            ((PI * WHEEL_DIAMETER_MM) / ENCODER_TOTAL_CPR)
#define TICKS_PER_MM           (1.0f / MM_PER_TICK)

// Search & Exploration Kinematic Limits (Safe bringup defaults)
#define SEARCH_SPEED_DEFAULT_MM_S     240.0f   // Safe bringup search cruise speed (mm/s)
#define SEARCH_ACCEL_DEFAULT_MM_S2    1500.0f  // Search linear acceleration / deceleration (mm/s^2)
#define SEARCH_CURVE_SPEED_MM_S       200.0f   // 90° corner arc speed during continuous search (mm/s)
#define SEARCH_TURN_SPEED_DEG_S       360.0f   // In-place turn speed (deg/s)
#define SEARCH_TURN_ACCEL_DEG_S2      1800.0f  // In-place turn acceleration (deg/s^2)

// Speedrun Kinematic Limits for N20 12V 30:1 Gearmotors (Max theoretical no-load ~700 mm/s)
#define SPEEDRUN_CRUISE_SPEED_MM_S    500.0f   // High-speed straight cruise speed for N20 (mm/s)
#define SPEEDRUN_ACCEL_MM_S2          2600.0f  // Maximum achievable acceleration for N20 30:1 (mm/s^2)
#define SPEEDRUN_DIAG_SPEED_MM_S      550.0f   // Diagonal sprint cruise speed (mm/s)
#define SPEEDRUN_CURVE_SPEED_MM_S     350.0f   // Continuous smooth curve arc speed (mm/s)
#define SPEEDRUN_TURN_SPEED_DEG_S     450.0f   // Speedrun in-place turn speed (deg/s)
#define SPEEDRUN_TURN_ACCEL_DEG_S2    2200.0f  // Speedrun in-place turn accel (deg/s^2)

// Fast Return Run Kinematic Limits
#define RETURN_CRUISE_SPEED_MM_S      380.0f   // Return cruise speed (mm/s)
#define RETURN_ACCEL_MM_S2            2000.0f  // Return acceleration (mm/s^2)

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
