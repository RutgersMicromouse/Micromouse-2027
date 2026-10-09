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
#define PIN_I2C_SDA            18  // SDA (Hardware I2C Wire)
#define PIN_I2C_SCL            19  // SCL (Hardware I2C Wire)
#define I2C_BUS_SPEED          400000 // 400 kHz Fast I2C

// Analog distance sensors: front sensor plus parallel side sensors.
#define PIN_IR_FRONT_RIGHT     14  // A0 - Front right side sensor
#define PIN_IR_REAR_LEFT       15  // A1 - Rear left side sensor
#define PIN_IR_FRONT_LEFT      16  // A2 - Front left side sensor
#define PIN_IR_FRONT           17  // A3 - Front-facing sensor
#define PIN_IR_REAR_RIGHT      20  // A6 - Rear right side sensor

// Motor battery voltage sensing
#define PIN_BAT_SENSE          21  // Motor BAT_SENSE (A7) - Resistor divider R1=100k, R2=33k

// Status / Debug LED
#define PIN_STATUS_LED         13  // On-board LED_BUILTIN

// Define DEBUG_IMU_STREAM or DEBUG_MOTOR_COMMAND_STREAM locally when tuning.
// Serial output in the 500 Hz control loop is intentionally disabled by default.

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
#define MOTOR_COMMAND_SCALE    1.0f // PID output is already limited to Motoron's command range
#define MOTOR_RIGHT_COMPENSATION 1.0f // Keep both motor channels on the same command scale
// Positive software speed must move both wheels forward.  The left motor uses
// the original positive channel polarity; the right motor is mirror-mounted.
// Change only the affected value between +1 and -1 after a lifted-wheel test.
#define MOTOR_LEFT_DIRECTION   1
#define MOTOR_RIGHT_DIRECTION  -1
// During a forward left correction, the right wheel needs extra authority on
// this chassis.  This is applied only to translating turns, never pivots.
#define LEFT_TURN_RIGHT_WHEEL_BOOST  1.0f

// -----------------------------------------------------------------------------
// 3. PHYSICAL ROBOT CONSTANTS
// -----------------------------------------------------------------------------
#define CELL_DIMENSION_MM      180.0f  // Standard micromouse cell size
#define HALF_CELL_MM           90.0f
#define WALL_THICKNESS_MM      13.0f
#define CORRIDOR_WIDTH_MM      167.0f  // 180 - 13 mm

// Drive Mechanics (Pololu Micro Metal Gearmotors + Wheels)
#define WHEEL_DIAMETER_MM      40.15f
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 3.1415926535f)
#define TRACK_WIDTH_MM         75.0f   // Distance between wheel contact patches
#define SIDE_SENSOR_SPACING_MM 71.5f   // Front-to-rear spacing on each side

// Encoder Resolution: 12 CPR motor shaft, ~50:1 gearbox -> ~600 counts per wheel rev
// Counts per mm = 600 / (32.0 * PI) ≈ 5.968 counts/mm
#define ENCODER_TICKS_PER_REV  600.0f
#define TICKS_PER_MM           (ENCODER_TICKS_PER_REV / WHEEL_CIRCUMFERENCE_MM)
#define MM_PER_TICK            (1.0f / TICKS_PER_MM)

// -----------------------------------------------------------------------------
// 4. MOTOR BATTERY MONITORING
// -----------------------------------------------------------------------------
// Divider: R1 = 100k, R2 = 33k. Vout = Vin * (33 / 133) = Vin * 0.24812
// Vin = Vout * (133 / 33) = Vout * 4.0303
#define BATTERY_DIVIDER_RATIO  ((100.0f + 33.0f) / 33.0f)
#define ADC_REF_VOLTAGE        3.3f
#define ADC_RESOLUTION         1023.0f // 10-bit analogRead default
#define BATTERY_WARN_VOLTAGE   3.6f    // 1-cell LiPo/Li-ion motor battery low warning
#define BATTERY_CRITICAL_V     3.3f    // 1-cell LiPo/Li-ion motor battery critical threshold

// -----------------------------------------------------------------------------
// 5. MOTION CONTROL LOOP
// -----------------------------------------------------------------------------
#define CONTROL_FREQ_HZ        500.0f  // 500 Hz high performance loop
#define CONTROL_DT_S           (1.0f / CONTROL_FREQ_HZ) // 0.002 seconds (2 ms)

// Velocity & Acceleration Profiles
#define SEARCH_SPEED_MM_S      45.0f   // Slow but high enough to overcome drivetrain friction
#define FAST_SPEED_MM_S        140.0f  // Reduced speed-run velocity
#define MAX_SPEED_MM_S         200.0f  // Reduced physical ceiling
#define MIN_SPEED_MM_S         20.0f

#define SEARCH_ACCEL_MM_S2     120.0f  // Gentle acceleration for wall following
#define FAST_ACCEL_MM_S2       500.0f  // Reduced speed-run acceleration
#define DECEL_MM_S2            360.0f  // Controlled deceleration

#define TURN_SPEED_DEG_S       30.0f   // Deliberately slow in-place pivot turn rate
#define TURN_MAX_MOTOR_COMMAND 100.0f  // Safe pivot-command limit for initial tests
#define TURN_ACCEL_DEG_S2      400.0f  // Reduced angular acceleration

#define ENABLE_IR_WALL_CENTERING

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

// Raw ADC Thresholds (10-bit ADC readings, 3.3V reference)
#define IR_WALL_DETECT_FRONT   220     // Front wall if raw ADC is at or above this value
#define IR_WALL_DETECT_SIDE    200     // Side wall if raw ADC is at or above this value

// Millimeter Distance Thresholds
#define WALL_DETECT_DIST_MM    115.0f  // Side-wall guide validity range
#define NOMINAL_SIDE_WALL_MM   49.0f   // Distance from side sensor to wall when centered in cell
#define FRONT_WALL_STOP_MM     60.0f   // Stop with additional clearance from a front wall
#define IR_FRONT_STOP_DIST     420     // Front raw ADC reading when at front stop distance

// A move is considered complete only after the encoder-measured travel reaches
// this tolerance.  This compensates for one/two tick quantization error.
#define MOTION_DISTANCE_TOLERANCE_MM  2.0f
// Do not classify the expected wall at the end of a cell as a collision.  A
// front sensor placed forward of the axle normally reads about this close when
// the axle is centered in a walled cell.
#define FRONT_WALL_EARLY_STOP_REMAINING_MM  25.0f
