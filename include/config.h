#pragma once

#include <Arduino.h>

// =============================================================================
// Ratatouieee Micromouse Configuration Header
// Target: Teensy 4.0 (ARM Cortex-M7 @ 600MHz)
// Schematic: Schematic_ratatouieee_2026-10-02
// =============================================================================

// -----------------------------------------------------------------------------
// 1. PIN ALLOCATION (Directly mapped from schematic)
// -----------------------------------------------------------------------------

// Motor Encoders (Quadrature Channels)
#define PIN_ENC_L_A            2   // LMOTChanA
#define PIN_ENC_L_B            3   // LMOTChanB
#define PIN_ENC_R_A            4   // RMOTChanA
#define PIN_ENC_R_B            5   // RMOTChanB

// I2C Bus (Pololu Motoron M2T256 + Pololu MinIMU-9 v5)
#define PIN_I2C_SDA            18  // SDA1 (Hardware I2C Wire)
#define PIN_I2C_SCL            19  // SCL1 (Hardware I2C Wire)
#define I2C_BUS_SPEED          400000 // 400 kHz Fast I2C

// Analog Distance Sensors (5 IR Sensors)
#define PIN_IR_RIGHT_45        14  // R1IR (A0) - 45° Right Wall Sensor
#define PIN_IR_LEFT_90         15  // L2IR (A1) - 90° Left Wall Sensor
#define PIN_IR_LEFT_45         16  // L1IR (A2) - 45° Left Wall Sensor
#define PIN_IR_FRONT           17  // FIR  (A3) - Front Wall Sensor
#define PIN_IR_RIGHT_90        20  // R2IR (A6) - 90° Right Wall Sensor

// Battery Voltage Sensing
#define PIN_BAT_SENSE          21  // BAT_SENSE (A7) - Resistor divider R1=100k, R2=33k

// Status / Debug LED
#define PIN_STATUS_LED         13  // On-board LED_BUILTIN

// -----------------------------------------------------------------------------
// 2. I2C DEVICE ADDRESSES
// -----------------------------------------------------------------------------
#define MOTORON_I2C_ADDR       16    // 0x10 Default Pololu Motoron M2T256 address
#define LSM6DS33_I2C_ADDR_A    0x6B  // Default SA0 high/pullup
#define LSM6DS33_I2C_ADDR_B    0x6A  // Alternate SA0 low
#define LIS3MDL_I2C_ADDR_A     0x1E  // Default SA0 high/pullup
#define LIS3MDL_I2C_ADDR_B     0x1C  // Alternate SA0 low

// Motoron Motor Channels
#define MOTOR_LEFT_CHANNEL     1     // M1A / M1B
#define MOTOR_RIGHT_CHANNEL    2     // M2A / M2B
#define MOTOR_MAX_SPEED        800   // Max Motoron speed command (-800 to 800)

// -----------------------------------------------------------------------------
// 3. PHYSICAL ROBOT CONSTANTS
// -----------------------------------------------------------------------------
#define CELL_DIMENSION_MM      180.0f  // Standard micromouse cell size
#define HALF_CELL_MM           90.0f
#define WALL_THICKNESS_MM      12.0f
#define CORRIDOR_WIDTH_MM      168.0f  // 180 - 12 mm

// Drive Mechanics (Pololu Micro Metal Gearmotors + Wheels)
#define WHEEL_DIAMETER_MM      32.0f
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 3.1415926535f)
#define TRACK_WIDTH_MM         75.0f   // Distance between wheel contact patches

// Encoder Resolution: 12 CPR motor shaft, ~50:1 gearbox -> ~600 counts per wheel rev
// Counts per mm = 600 / (32.0 * PI) ≈ 5.968 counts/mm
#define ENCODER_TICKS_PER_REV  600.0f
#define TICKS_PER_MM           (ENCODER_TICKS_PER_REV / WHEEL_CIRCUMFERENCE_MM)
#define MM_PER_TICK            (1.0f / TICKS_PER_MM)

// -----------------------------------------------------------------------------
// 4. BATTERY MONITORING
// -----------------------------------------------------------------------------
// Divider: R1 = 100k, R2 = 33k. Vout = Vin * (33 / 133) = Vin * 0.24812
// Vin = Vout * (133 / 33) = Vout * 4.0303
#define BATTERY_DIVIDER_RATIO  ((100.0f + 33.0f) / 33.0f)
#define ADC_REF_VOLTAGE        3.3f
#define ADC_RESOLUTION         1023.0f // 10-bit analogRead default
#define BATTERY_WARN_VOLTAGE   6.8f    // 2S LiPo low warning (or ~10.2V for 3S)
#define BATTERY_CRITICAL_V     6.4f    // Cutoff threshold

// -----------------------------------------------------------------------------
// 5. MOTION CONTROL LOOP
// -----------------------------------------------------------------------------
#define CONTROL_FREQ_HZ        500.0f  // 500 Hz high performance loop
#define CONTROL_DT_S           (1.0f / CONTROL_FREQ_HZ) // 0.002 seconds (2 ms)

// Velocity & Acceleration Profiles
#define SEARCH_SPEED_MM_S      260.0f  // Stable exploration cruising speed
#define FAST_SPEED_MM_S        700.0f  // Optimized speed run velocity
#define MAX_SPEED_MM_S         1000.0f // Physical ceiling
#define MIN_SPEED_MM_S         40.0f

#define SEARCH_ACCEL_MM_S2     1200.0f // Exploration acceleration
#define FAST_ACCEL_MM_S2       2500.0f // Fast run acceleration
#define DECEL_MM_S2            1800.0f // Controlled deceleration

#define TURN_SPEED_DEG_S       360.0f  // In-place pivot turn rate
#define TURN_ACCEL_DEG_S2      2000.0f // Angular acceleration

// -----------------------------------------------------------------------------
// 6. SHARP GP2Y0A51SK0F (0A51SK) SENSOR CALIBRATION & DISTANCES
// Range: 2 cm to 15 cm (20 mm to 150 mm)
// Output: ~2.2V at 20mm down to ~0.4V at 150mm
// Model: Distance_mm = SHARP_A / (Voltage - SHARP_B)
// -----------------------------------------------------------------------------
#define SHARP_0A51SK_A         48.375f // Empirical curve numerator (mm * V)
#define SHARP_0A51SK_B         0.0675f // Voltage offset baseline (V)
#define SHARP_MIN_DIST_MM      15.0f   // Physical close-range threshold
#define SHARP_MAX_DIST_MM      160.0f  // Physical far-range threshold

// Raw ADC Thresholds (Teensy 4.0 10-bit ADC, 3.3V reference)
#define IR_WALL_DETECT_FRONT   180     // Front wall detection threshold (ADC)
#define IR_WALL_DETECT_L45     150     // 45° Left wall detection threshold (ADC)
#define IR_WALL_DETECT_R45     150     // 45° Right wall detection threshold (ADC)
#define IR_WALL_DETECT_L90     130     // 90° Left wall detection threshold (ADC)
#define IR_WALL_DETECT_R90     130     // 90° Right wall detection threshold (ADC)

// Millimeter Distance Thresholds
#define WALL_DETECT_DIST_MM    115.0f  // Objects closer than 115mm classify as a wall
#define NOMINAL_SIDE_WALL_MM   49.0f   // Distance from side sensor to wall when centered in cell
#define FRONT_WALL_STOP_MM     45.0f   // Target distance to front wall when stopped/squaring
#define IR_FRONT_STOP_DIST     420     // Front raw ADC reading when at front stop distance

