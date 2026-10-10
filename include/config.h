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
#define MOTOR_MAX_SPEED        350   // Safe max speed command (-800 to 800) on 12V boosted rail
#define MOTOR_COMMAND_SCALE    1.0f 
#define MOTOR_LEFT_COMPENSATION 1.0f // Let PID controller handle wheel balance
#define MOTOR_RIGHT_COMPENSATION 0.91f // 9% trim on right motor to balance physical motor strength
#define MOTOR_ACCELERATION_NORMAL 0   // 0 = Disable internal Motoron rate-limiting; let software profile control ramps

// Commented out by default: automatic wheel spin test at boot
// #define CALIBRATE_MOTORS
#define MOTOR_BALANCE_TEST_COMMAND       220
#define MOTOR_BALANCE_TEST_DURATION_MS   1200
#define MOTOR_BALANCE_MAX_COMPENSATION  1.15f

// Motor directional polarity (+1 = forward)
#define MOTOR_LEFT_DIRECTION   1
#define MOTOR_RIGHT_DIRECTION  1
#define MOVING_TURN_STEERING_BOOST  1.60f
#define LEFT_TURN_RIGHT_WHEEL_BOOST 1.0f
#define LEFT_TURN_LEFT_WHEEL_BOOST  1.25f // Boost weaker left motor in reverse during left turns
#define LEFT_TURN_RIGHT_WHEEL_TRIM  0.85f // Trim stronger right motor forward during left turns

// -----------------------------------------------------------------------------
// 3. PHYSICAL ROBOT CONSTANTS
// -----------------------------------------------------------------------------
#define CELL_DIMENSION_MM      170.0f  // Standard micromouse cell size
#define HALF_CELL_MM           90.0f
#define WALL_THICKNESS_MM      13.0f
#define CORRIDOR_WIDTH_MM      167.0f  // 180 - 13 mm

// Drive Mechanics (Pololu Micro Metal Gearmotors + 40mm Wheels)
#define WHEEL_DIAMETER_MM      40.15f
#define WHEEL_CIRCUMFERENCE_MM (WHEEL_DIAMETER_MM * 3.1415926535f)
// Effective kinematic track width calibrated from physical 360° turn test:
// (75.0mm nominal * 360.0 / 540.0 = 50.00mm). Compensates for tire scrub and physical wheel spacing during pivots.
#define TRACK_WIDTH_MM         50.00f  // Effective pivot track width between wheel contact patches
#define SIDE_SENSOR_SPACING_MM 71.5f   // Front-to-rear spacing on each side

// Encoder Resolution: Calibrated to 196.0 counts per wheel rev (~1.554 ticks/mm)
// Compensates for physical tire compression & rolling radius to eliminate distance undervaluing
#define ENCODER_TICKS_PER_REV  196.0f
#define TICKS_PER_MM           (ENCODER_TICKS_PER_REV / WHEEL_CIRCUMFERENCE_MM)
#define MM_PER_TICK            (1.0f / TICKS_PER_MM)

// -----------------------------------------------------------------------------
// 4. MOTOR BATTERY MONITORING (1S 3.7V LiPo)
// -----------------------------------------------------------------------------
// Divider: R1 = 100k, R2 = 33k. Vin = Vout * (133 / 33) = Vout * 4.0303
#define BATTERY_DIVIDER_RATIO  ((100.0f + 33.0f) / 33.0f)
#define ADC_REF_VOLTAGE        3.3f
#define ADC_RESOLUTION         1023.0f // 10-bit analogRead default
#define BATTERY_WARN_VOLTAGE   3.55f   // 1S LiPo warning threshold
#define BATTERY_CRITICAL_V     3.30f   // 1S LiPo critical cut-off threshold
#define BATTERY_DISCONNECTED_V 1.50f   // Motor switch SW2 is physically turned OFF

// -----------------------------------------------------------------------------
// 5. MOTION CONTROL LOOP
// -----------------------------------------------------------------------------
#define CONTROL_FREQ_HZ        500.0f  // 500 Hz control loop
#define CONTROL_DT_S           (1.0f / CONTROL_FREQ_HZ) // 0.002 seconds (2 ms)

// Velocity & Acceleration Profiles
#define SEARCH_SPEED_MM_S      75.0f   // Deliberate, calm exploration crawl speed
#define FAST_SPEED_MM_S        280.0f  // Speed-run sprint speed
#define MAX_SPEED_MM_S         350.0f  // Top physical speed
#define MIN_SPEED_MM_S         28.0f   // Minimum crawl speed

#define SEARCH_ACCEL_MM_S2     150.0f  // Smooth linear acceleration for search
#define FAST_ACCEL_MM_S2       700.0f  // Linear acceleration for speed run
#define DECEL_MM_S2            240.0f  // Crisp, controlled linear deceleration

// Motor Deadband & Breakaway Kick Parameters
#define MOTOR_MIN_PWM                  38    // Minimum operational PWM to sustain crawl
#define MOTOR_RIGHT_STARTUP_OFFSET     0     // Symmetric floor to ensure straight line tracking
#define MOTOR_KICKSTART_PWM            80    // Breakaway friction pulse
#define MOTOR_KICKSTART_DURATION_MS    35    // Duration of kickstart pulse (ms)
#define SEARCH_BREAKAWAY_BOOST_COMMAND 15    // Gentle initial linear motion boost
#define SEARCH_BREAKAWAY_BOOST_MS      40    // Boost duration (ms)

// Closed-Loop Turn Parameters (Gyro Feedback)
#define TURN_MAX_MOTOR_COMMAND 115.0f  // Controlled differential motor PWM for turns (prevents floor stall)
#define TURN_SPEED_DPS         90.0f   // Steady turn yaw rate (deg/s)
#define TURN_ACCEL_DPS2        260.0f  // Smooth turn angular acceleration (deg/s^2)
#define TURN_BREAKAWAY_PWM     58.0f   // Breakaway bias to overcome gearbox & tire scrub friction
#define TURN_KP                2.40f   // Heading error proportional gain
#define TURN_KI                0.04f   // Heading error integral gain
#define TURN_KD                0.10f   // Heading error derivative gain

// IMU Gyro Configuration
// Axis: 0 = X, 1 = Y, 2 = Z (standard horizontal mounting uses Z)
#define IMU_YAW_AXIS           2
#define IMU_YAW_SIGN           1.0f    // Change to -1.0f if clockwise/counter-clockwise inverted
// Turn scaling factor: Calibrated from empirical 540° vs 360° rotation (540.0 / 360.0 = 1.5000)
#define IMU_GYRO_SCALE_FACTOR  1.5000f

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
#define IR_WALL_DETECT_FRONT   210     // Front wall threshold (~80 mm)
#define IR_WALL_DETECT_SIDE    190     // Side wall threshold

// Millimeter Distance Thresholds
#define WALL_DETECT_DIST_MM    125.0f  // Side-wall guide validity range
#define NOMINAL_SIDE_WALL_MM   49.0f   // Distance from side sensor to wall when centered in cell

// Front wall stop thresholds:
// 310 ADC = ~50 mm distance from sensor to front wall (robot bumper is ~28 mm from wall, axle is centered in cell)
#define IR_FRONT_STOP_DIST     310     // Cell-center stop threshold when approaching front wall
#define IR_FRONT_CRITICAL_DIST 365     // Critical proximity threshold (~43 mm from sensor, bumper ~20 mm)
#define FRONT_WALL_STOP_CONFIRM_MS 10  // Fast confirmation (10 ms = 5 control cycles)
#define CELL_REACHED_THRESHOLD_MM 35.0f // Traveled distance threshold to confirm cell arrival

// Motion completion tolerance in mm (matches physical encoder quantization)
#define MOTION_DISTANCE_TOLERANCE_MM  1.5f
