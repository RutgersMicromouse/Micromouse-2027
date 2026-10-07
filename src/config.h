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
// Pin map and channel positions are the ones verified on the robot by bench/ir_test.cpp.
// NOTE: the schematic's IR_E4..E6 / IR_R4..R6 net numbering does NOT match physical position;
//       the channel numbers below (CH1..CH6, left to right) are the ones to trust.
// Emitters (Driven by AO3400A N-MOSFETs with 10k pulldown, active HIGH)
#define PIN_IR_E1              15       // CH1 emitter: Left 90°
#define PIN_IR_E2              16       // CH2 emitter: Front-Left 45°
#define PIN_IR_E3              17       // CH3 emitter: Front-Left Center
#define PIN_IR_E4              14       // CH4 emitter: Front-Right Center
#define PIN_IR_E5              18       // CH5 emitter: Front-Right 45°
#define PIN_IR_E6              19       // CH6 emitter: Right 90°

// Receivers (TEFT4300 with 10k load to GND, sampled on ADC1)
#define PIN_IR_R1              3        // CH1 receiver: Left 90°           (ADC1_CH2)
#define PIN_IR_R2              8        // CH2 receiver: Front-Left 45°     (ADC1_CH7)
#define PIN_IR_R3              1        // CH3 receiver: Front-Left Center  (ADC1_CH0)
#define PIN_IR_R4              10       // CH4 receiver: Front-Right Center (ADC1_CH9)
#define PIN_IR_R5              2        // CH5 receiver: Front-Right 45°    (ADC1_CH1)
#define PIN_IR_R6              9        // CH6 receiver: Right 90°          (ADC1_CH8)

// Emitters fire in two interleaved groups, one group per control tick, so that neighbouring
// sensors never light the same wall at the same time and each tick only pays for one settle delay:
//   Group A (even ticks): CH1 Left 90°, CH3 Front-Left,  CH6 Right 90°
//   Group B (odd ticks) : CH2 Left 45°, CH4 Front-Right, CH5 Right 45°
#define IR_PULSE_SETTLE_US     300      // Emitter-on settling time before sampling (value proven in ir_test)
#define IR_FILTER_ALPHA        0.75f    // Low-pass weight of the newest sample (each channel updates at 250 Hz)

// Status LED: the RGB NeoPixel on the ESP32-S3-DevKitC-1 board itself (the PCB has no LED or buttons).
// DevKitC-1 v1.0 wires it to GPIO 48; if yours is a v1.1 board and the LED stays dark, change this to 38.
#define PIN_ESP32_RGB_LED      48
#define RGB_BRIGHTNESS_LEVEL   40       // Brightness level (0-255, 40 provides vibrant color without glare)

// Hand-Wave Controls (the robot has no buttons: wave a hand in front of the two front IR sensors)
// A "wave" is the front reading rising clearly above its resting level and dropping back again.
#define ENABLE_GESTURE_UI       1       // 0 = ignore waves (debugging only: the robot then needs text commands)
#define GESTURE_MIN_RISE        250     // Front reading must rise at least this far above resting level...
#define GESTURE_RISE_RATIO      0.6f    // ...and at least this fraction of the resting level (matters when facing a wall)
#define GESTURE_MAX_WAVE_MS     1500    // A hand held longer than this is not a wave (and cancels the count)
#define GESTURE_SEQUENCE_GAP_MS 1500    // No new wave for this long = the count is final
#define GESTURE_REBASE_MS       3000    // Reading stuck high this long = scenery changed, learn a new resting level
#define GESTURE_WARMUP_MS       2000    // Waves are ignored this long after power-on / after a run ends
#define GESTURE_LAUNCH_DELAY_MS 2000    // Blinking countdown before the robot moves; cover the sensors to cancel

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

// Smooth Turn Geometry (derivation and clearance check: see docs/INSTRUCTIONS.md section 8.4)
// A smooth turn eases its heading in and out along the path: heading = angle * (3u² - 2u³).
// These lengths make the turns land exactly on the maze grid; do not change one without the others.
#define CURVE_90_LENGTH_MM     148.73f  // 90° turn, cell edge to cell edge (90 mm forward, 90 mm sideways)
#define CURVE_45_LENGTH_MM     70.0f    // 45° turn between a cell centreline and a diagonal
#define DIAG_LEAD_MM           53.52f   // Straight from a cell centre before the 45° turn onto a diagonal
#define DIAG_TRIM_MM           36.48f   // Diagonal distance each 45° turn takes off the diagonal straight
#define DIAG_HALF_STEP_MM      127.279f // One diagonal half-step: edge midpoint to edge midpoint (90 * sqrt 2)

#define SEARCH_PROBE_SPEED_MM_S 120.0f  // Search speed when rolling into a cell it has never seen (may have to stop)

// Motor Feedforward: effort = FF_KS (to overcome friction) + FF_KV * speed + FF_KA * acceleration.
// This supplies most of the motor effort directly from the planned motion; the PID loops only
// correct what is left over. Measure the three numbers from a run log (console command "log").
#define MOTOR_NO_LOAD_SPEED_MM_S 700.0f // Wheel speed at 100% effort on a full supply (N20 30:1, 24 mm wheel)
#define FF_KS                  0.03f    // Effort needed just to keep a wheel turning
#define FF_KV                  (1.0f / MOTOR_NO_LOAD_SPEED_MM_S) // Effort per mm/s of wheel speed
#define FF_KA                  0.0f     // Effort per mm/s² of acceleration (0 until measured)
#define MOTOR_NOMINAL_VOLTS    12.0f    // Motor supply the numbers above assume; efforts are rescaled if it sags

// Run Log: the last RUN_LOG_SAMPLES snapshots of target vs. actual motion, kept in RAM while driving
#define RUN_LOG_SAMPLES        1500     // 30 seconds at 50 Hz (36 kB of RAM)
#define RUN_LOG_DIVIDER        10       // Record every 10th control tick (50 Hz)

#define POST_EDGE_PHASE_MM     90.0f    // Distance past a cell centre at which a side sensor sees a wall start / end

// Search Look-Ahead (smooth turns during the search run)
// The 45° sensors point forward and outward, so while the robot drives from one cell centre to the
// next cell's edge they are already looking at the NEXT cell's side walls. If that says "open" and
// the 90° sensor agrees once the robot reaches the edge, the robot curves straight through the
// corner instead of driving to the centre, stopping, and turning on the spot. Half-way round the
// curve the outer 45° sensor faces that cell's front wall squarely and records it.
#define ENABLE_SEARCH_LOOKAHEAD 1       // 0 = always decide at the cell centre and turn on the spot
#define SEARCH_LOOKAHEAD_START_MM 30.0f // Side walls of the next cell are sampled between these
#define SEARCH_LOOKAHEAD_END_MM   80.0f //   distances after leaving a cell centre
#define SEARCH_OPEN_RATIO      0.6f     // "Definitely open" = every sample below this fraction of the wall threshold
#define SEARCH_FRONT_SAMPLE_FROM 0.40f  // Part of a 90° curve (0..1) during which the outer 45° sensor
#define SEARCH_FRONT_SAMPLE_TO   0.60f  //   faces the front wall of the cell being curved through
#define PREVIEW_MIN_SAMPLES    4        // Fewer samples than this = no opinion

// Diagonal Centring (keeps the robot midway between the posts on a diagonal)
// On a diagonal each 45° sensor points along a maze axis and sees the posts go by one at a time.
// The strongest reading as each post passes tells how close that row of posts is; comparing the
// left and right rows gives how far off the centre line the robot is.
#define DIAG_CENTER_GAIN_DEG   12.0f    // Heading correction per unit of left/right imbalance
#define DIAG_PEAK_MEMORY_MM    90.0f    // Distance over which a post's peak reading is remembered
#define DIAG_PEAK_MIN          0.25f    // Peaks weaker than this fraction of a centred wall are ignored

// Diagonal Post Guard (steers away from a post that gets too close during a diagonal)
#define DIAG_GUARD_RATIO       1.6f     // A 45° sensor reading this many times its centred level = post too close
#define DIAG_GUARD_MAX_TRIM_DEG 8.0f    // Largest heading correction the guard may apply

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

// Fault tolerance (see docs/INSTRUCTIONS.md section 8.3)
#define MOTORON_HEALTH_PERIOD_MS   200  // How often the Motoron status flags are polled for resets / bus faults
#define MOTORON_KEEPALIVE_MS       100  // Re-send interval so the Motoron's 250 ms command timeout never trips
#define IMU_MAX_STEP_DEG           30.0f // A heading jump larger than this between two 100 Hz reads is rejected as a glitch
#define IMU_FAULT_READS            10   // Consecutive bad reads (100 ms) before falling back to encoder odometry
#define ENCODER_FAULT_TICKS        75   // Control ticks (150 ms) one wheel may read zero while the other is moving
#define HEADING_FAULT_DEG          60.0f // Heading this far off course = crashed or picked up (lift + twist to stop a run)
#define HEADING_FAULT_TICKS        50   // ...for this many control ticks (100 ms) before the run is aborted

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

#ifndef ENABLE_BLE_DEBUG                // The `competition` build in platformio.ini sets this to 0
#define ENABLE_BLE_DEBUG       1        // 1 = Nordic UART Service active, 0 = RF disabled for competition
#endif
#define BLE_DEVICE_NAME        "Antigrav-Mouse"

// ==============================================================================
// 7. WIRELESS OTA (OVER-THE-AIR) FLASHING & TELNET DEBUGGING
// ==============================================================================

#ifndef ENABLE_WIFI_OTA                 // The `competition` build in platformio.ini sets this to 0
#define ENABLE_WIFI_OTA        1        // 1 = Wi-Fi OTA & Telnet Active, 0 = RF disabled for competition
#endif
#define WIFI_AP_MODE           1        // 1 = Broadcast Hotspot (SoftAP), 0 = Connect to Local Wi-Fi (STA)
#define WIFI_AP_SSID           "Antigrav-Mouse"
#define WIFI_AP_PASS           "micromouse"   // WPA2 Passphrase (minimum 8 chars)
#define WIFI_STA_SSID          "YourHomeWiFi" // Used if WIFI_AP_MODE = 0
#define WIFI_STA_PASS          "YourPassword" // Used if WIFI_AP_MODE = 0
#define OTA_PORT               3232
#define TELNET_PORT            23
