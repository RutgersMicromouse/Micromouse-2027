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
// All four are true. They were set to false for one night (2026-10-09), when the robot ran
// backwards in a 3D-printed body that has since been replaced. On 2026-10-10, rolled forward by
// hand with them false, both encoders counted DOWN, and turned left by hand they read a right turn
// while the heading read left. "movecheck" (the app's Movement check) finds the right values on
// the floor by itself and prints them.
#define INVERT_LEFT_MOTOR      true
#define INVERT_RIGHT_MOTOR     true
#define INVERT_LEFT_ENCODER    true
#define INVERT_RIGHT_ENCODER   true

// Motor Controller: Pololu Motoron M2T256 (I2C)
#define PIN_MOTOR_RST          11       // MRST: Active-low reset line for Motoron (GPIO11 in schematic)
#define PIN_MOTOR_BAT_CTRL     13       // MotorBatControl: NPN drive to PMOS for 12V boost converter (Active HIGH)
#define MOTORON_I2C_ADDR       16       // Default Pololu Motoron I2C address (0x10)
#define MOTORON_MAX_SPEED      800      // Max speed units for Motoron M2T256
// How the Motoron's supply-voltage reading is scaled. Measured on the robot: the "256" scaling the
// part number suggests reads 27 V on a 12 V supply; the "550" scaling reads 11.8 V, which is right.
#define MOTORON_VIN_TYPE       MotoronVinSenseType::Motoron550

// Shared I2C Bus (BNO055 IMU + Motoron M2T256)
#define PIN_I2C_SDA            21       // Pulled up by R13 3.3k to 3.3V
#define PIN_I2C_SCL            20       // Pulled up by R19 3.3k to 3.3V
#define I2C_CLOCK_SPEED        400000   // Fast Mode I2C (400 kHz)
#define I2C_TIMEOUT_MS         3        // Longest one I2C transfer may take. 1 was too tight: the BNO055
                                        //   holds the clock now and then, and the robot logged 433 failed reads

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

// Only two emitters are ever on at once. They take turns, one pair per control tick:
//   tick 0: the two front sensors          (CH3, CH4)
//   tick 1: the two 90° side sensors       (CH1, CH6)
//   tick 2: the two 45° diagonal sensors   (CH2, CH5)
// so each sensor is read 167 times a second. (The owner asked for front+sides together, but on the
// robot four sensors in one tick overran the 2 ms control loop about one tick in five.)
#define IR_PULSE_SETTLE_US     300      // Emitter-on time before sampling (the value proven in ir_test)

// IR SENSITIVITY. The receivers put out a small voltage; this sets how much of the ADC's 0-4095
// range that voltage is spread over. The owner found a wall right in front of a sensor barely
// registered (2026-10-07), so it is set to the most sensitive range.
//   ADC_0db   = most sensitive: full scale at about 0.95 V (3.5x the reading of ADC_11db)
//   ADC_2_5db = full scale at about 1.25 V (2.6x)
//   ADC_6db   = full scale at about 1.75 V (1.9x)
//   ADC_11db  = least sensitive: full scale at about 3.1 V (what the bench test used)
// If close walls now pin the reading at 4095, step down one line. Recalibrate (5 waves) after any change.
#define IR_ADC_ATTENUATION     ADC_0db
// Weak 45° sensors (found 2026-10-08: the right one reads 50-110 against a wall, the left 230-420)
#define IR_CALIB_90_MIN        100      // At calibration, at least one 90° sensor must read above this, or it is refused (no walls)
#define IR_CALIB_45_MIN        50       // At calibration, a 45° sensor reading at least this is looking at a wall
#define IR_CALIB_45_MIN_SHARE  0.15f    // ...provided it is also this share of what the other side predicts for it
#define IR_45_MIN_WALL_LEVEL  40.0f    // A 45° sensor's "wall" level is never set below this
// 1 = wall steering measures the distance to each side wall with the 90° sensors (owner said yes,
// 2026-10-10: the 45° sensors are too weak on this robot to see a few millimetres of drift, and
// near a front wall they read that wall). 0 = the 45° sensors, as before. Until the robot has been
// calibrated with this firmware the 90° centred readings are unknown and it uses the 45° ones.
// It applies to moves that ask for it (MotionCommand::steer_by_side_sensors): the search and the
// straight test do, speed runs do not yet.
#define ENABLE_SIDE_SENSOR_CENTERING 1
#define IR_CENTER_MIN_NOMINAL  120      // A 45° sensor centred below this is not steered by while the other side has a wall
#define IR_CENTER_TOLERANCE     0.05f    // "In the middle of the cell" = within this fraction (5 %) of the wall distance
                                        //   learned by IR calibration in the start cell; no wall steering inside it
#define IR_FILTER_ALPHA        0.75f    // Low-pass weight of the newest sample (each channel updates at 250 Hz)

// Status LED: the RGB NeoPixel on the ESP32-S3-DevKitC-1 board itself (the PCB has no LED or buttons).
// Board revision v1.0 wires it to GPIO 48 and v1.1 to GPIO 38, and they look the same, so the
// firmware simply drives both. (GPIO 38 otherwise only goes to the unused LED footprint on the PCB.)
#define PIN_ESP32_RGB_LED      48
#define PIN_ESP32_RGB_LED_ALT  38
#define RGB_BRIGHTNESS_LEVEL   40       // Brightness level (0-255, 40 provides vibrant color without glare)

// Hand Controls (the robot has no buttons: put a hand in front of the left, front or right IR sensors)
// A hand is a reading rising clearly above its resting level and dropping back again.
// A hand at the front starts the choosing; the LED blinks each stage's colour, a hand = next
// stage, left alone for GESTURE_CONFIRM_BLINKS blinks = do it (described in ui/gestures/gestures.h).
// Back ON at the owner's request (2026-10-10): remote controls are not allowed at competition.
#define ENABLE_GESTURE_UI       1       // 1 = hand control, 0 = ignore hands

// With waves off: this many seconds after power-on the robot calibrates its IR sensors where it
// stands and starts a search run, once. 0 = never start by itself. Ignored when waves are on.
#define AUTO_START_DELAY_S      0       // Off: the START button in the phone app starts the run instead
#define GESTURE_MIN_RISE        250     // A reading must rise at least this far above its resting level...
#define GESTURE_RISE_RATIO      0.6f    // ...and at least this fraction of the resting level (matters when facing a wall)
#define GESTURE_MIN_DROP        100     // ...or, beside a wall, DROP at least this far below it (the hand hides the wall)...
#define GESTURE_DROP_RATIO      0.3f    // ...and at least this fraction of the resting level
#define GESTURE_REBASE_MS       3000    // Reading stuck high this long = scenery changed, learn a new resting level (nothing happens)
#define GESTURE_WARMUP_MS       2000    // Hands are ignored this long after power-on, after a run ends, and after the robot was moved
#define GESTURE_STILL_DEG       8.0f    // Heading changing by more than this = the robot is being handled: ignore hands and wait again
#define GESTURE_LIFTED_DEG      15.0f   // Tipped this far out of level = lifted: cancels what was just asked for, ignores hands
#define GESTURE_CONFIRM_BLINKS  5       // A stage (or speed level) left alone for this many blinks is carried out
#define GESTURE_BLINK_MS        500     // One blink, on and off
// The speed levels a hand can pick for a speed run, as a percentage of the SPEEDRUN_* speeds and
// accelerations (a hand = the next one, round again after the last), and how bright the LED
// blinks at each (0-255).
#define GESTURE_SPEED_PERCENTS   { 35, 50, 65, 80, 100 }
#define GESTURE_SPEED_BRIGHTNESS {  4, 12, 35, 95, 255 }
#define GESTURE_SPEEDRUN_MODE    Actions::MODE_HYBRID   // The speed run a hand starts: robot picks curves or diagonals

// Battery Voltage Monitoring
#define PIN_VSENSE_COM         12       // Computer Battery divider (R40=10k, R39=10k -> 2.0x divider)
#define BATTERY_DIVIDER_RATIO  2.0f     // (10k + 10k) / 10k

// ==============================================================================
// 2. ROBOT PHYSICAL & KINEMATIC PARAMETERS
// ==============================================================================

#define WHEEL_DIAMETER_MM      40.89f   // Tyre diameter measured by the owner (2026-10-10; was 40.5)
#define WHEEL_BASE_MM          80.5f    // Left wheel centre to right wheel centre, measured by the owner (2026-10-09). Was a guessed 72.

// The 3D-printed base plate (owner's measurements, 2026-10-09). The PCB and its sensors did not
// move. Nothing in the firmware uses these yet; they are here because they decide what fits:
// a cell is 168 mm between walls, so a wall is 84 mm from the cell centre, and when the robot
// turns on the spot its back corners sweep a circle of radius sqrt(77^2 + 47.5^2) = 90.5 mm.
#define ROBOT_WIDTH_MM         87.5f    // Across the outside of the tyres, the widest point (2026-10-10)
#define ROBOT_FRONT_MM         32.56f   // Wheel axle to the front edge (casing of 2026-10-10)
#define ROBOT_BACK_MM          33.69f   // Wheel axle to the back edge (casing of 2026-10-10)

// N20 12V Micro Metal Gearmotors with Magnetic Encoders
#define ENCODER_GEAR_RATIO     30.0f    // 30:1 metal gear reduction ratio
#define ENCODER_CPR_RAW        7.0f     // 7 pulses per channel per motor shaft rev
#define ENCODER_TOTAL_CPR      (ENCODER_CPR_RAW * 4.0f * ENCODER_GEAR_RATIO) // 840.0 ticks/wheel rev (4x quadrature decoding)

// MEASURE THESE ON THE ROBOT: encoder ticks for ONE full turn of each wheel. The two motors on
// this robot do not give the same count, so each side has its own number. To measure: send
// "resetenc", turn one wheel forward by hand exactly 10 full turns, send "enc", and divide that
// side's tick count by 10. Until measured, both use the datasheet value above.
// Measured by the owner on 2026-10-07: 4974 (left) and 2632 (right) ticks in 10 turns by hand.
// Neither is the datasheet 840 and they are not a clean ratio of each other, so the encoders are
// probably missing counts; these are the best numbers available until that is found.
// Counted by the owner on 2026-10-10: 10 full turns of each wheel by hand, twice (left 5699 and
// 5754 ticks, right 2930 and 2937). The right encoder counts about half of what the left does.
// The values before this (497.4 and 263.2) were about 13 % low, which dist_k 1.15 made up for.
#define ENCODER_TICKS_PER_REV_LEFT   572.7f
#define ENCODER_TICKS_PER_REV_RIGHT  293.4f
#define MM_PER_TICK_LEFT       ((PI * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REV_LEFT)
#define MM_PER_TICK_RIGHT      ((PI * WHEEL_DIAMETER_MM) / ENCODER_TICKS_PER_REV_RIGHT)

// Wheel-speed smoothing: weight of the newest 2 ms sample (1 = none). With the tick counts above
// the right wheel gives a tick only every few control cycles at search speed, so its raw speed
// jumps between 0 and about 240 mm/s; at the old 0.6 the speed loop shook the motors with it.
// Lower = smoother but slower to react. Put back toward 0.6 once the encoders give 840 per turn.
#define ENCODER_SPEED_FILTER_ALPHA   0.05f

#define MM_PER_TICK            ((PI * WHEEL_DIAMETER_MM) / ENCODER_TOTAL_CPR)
#define TICKS_PER_MM           (1.0f / MM_PER_TICK)

// Search & Exploration Kinematic Limits (Safe bringup defaults)
// SLOWED DOWN for bring-up at the owner's request (2026-10-07). The normal values are in brackets;
// put them back once the robot drives a search cleanly.
#define SEARCH_SPEED_DEFAULT_MM_S     180.0f   // Search cruise speed (mm/s)                          [240]
#define SEARCH_ACCEL_DEFAULT_MM_S2    600.0f   // Search linear acceleration / deceleration (mm/s^2)  [1500]
#define SEARCH_CURVE_SPEED_MM_S       100.0f   // Smooth 90° turn speed during the search (mm/s)      [200]
#define SEARCH_TURN_SPEED_DEG_S       120.0f   // In-place turn speed (deg/s)                         [360]
#define SEARCH_TURN_ACCEL_DEG_S2      480.0f   // In-place turn acceleration (deg/s^2)                [1800]
#define SEARCH_PROBE_SPEED_MM_S       120.0f   // Search speed when rolling into a cell it has never seen (may have to stop) [120]
// Dynamic search speed: on a straight through cells it has already visited (most of the way back,
// and any second search) the robot speeds up to this, and slows again in time for the first cell
// that is new or where the route turns. Smooth turns are always taken at SEARCH_CURVE_SPEED_MM_S
// or slower. Set it equal to SEARCH_SPEED_DEFAULT_MM_S to switch the speeding-up off.
#define SEARCH_KNOWN_SPEED_MM_S       260.0f   // Top search speed on a straight of known cells (mm/s) [400]
// The same idea for single cells and curves, which is what speeds up the way back in a small maze:
// driving one cell into a cell it has visited, the robot may reach SEARCH_KNOWN_SPEED_MM_S in
// between, and it curves through a visited cell at this instead of SEARCH_CURVE_SPEED_MM_S.
// (140 until 2026-10-09: on the way back in the 5x5 a right-left-left chain of curves at 140 drifted
// wide and hit a wall. The owner wants the curves kept chained, but slower, so the robot can adjust.)
#define SEARCH_KNOWN_CURVE_SPEED_MM_S 100.0f   // Smooth 90° turn through a cell already visited (mm/s)
#define SEARCH_CHAINED_CURVE_SPEED_MM_S 80.0f  // A curve that follows straight after another curve, a U-turn included (mm/s)

// Speedrun Kinematic Limits for N20 12V 30:1 Gearmotors (Max theoretical no-load ~700 mm/s)
#define SPEEDRUN_CRUISE_SPEED_MM_S    500.0f   // High-speed straight cruise speed for N20 (mm/s)
#define SPEEDRUN_ACCEL_MM_S2          2600.0f  // Maximum achievable acceleration for N20 30:1 (mm/s^2)
#define SPEEDRUN_DIAG_SPEED_MM_S      550.0f   // Diagonal sprint cruise speed (mm/s)
#define SPEEDRUN_CURVE_SPEED_MM_S     350.0f   // Continuous smooth curve arc speed (mm/s)
// Whatever the speed tier or the app's speed slider says, smooth curves in a speed run are never
// taken faster than this: the robot brakes into each curve and speeds up again after it. On the
// robot (2026-10-08) curves ended 4° past their heading at 175 mm/s, 8° at 210, 18° at 245 and
// 27° at 262. Raise it once curves at this speed end on their heading.
#define SPEEDRUN_CURVE_MAX_MM_S       180.0f
#define SPEEDRUN_TURN_SPEED_DEG_S     450.0f  // Speedrun in-place turn speed (deg/s)
#define SPEEDRUN_TURN_ACCEL_DEG_S2    2200.0f  // Speedrun in-place turn accel (deg/s^2)

// Smooth Turn Geometry (derivation and clearance check: see docs/INSTRUCTIONS.md section 8.4)
// A smooth turn eases its heading in and out along the path: heading = angle * (3u² - 2u³).
// These lengths make the turns land exactly on the maze grid; do not change one without the others.
#define CURVE_90_LENGTH_MM     148.73f  // 90° turn, cell edge to cell edge (90 mm forward, 90 mm sideways)
#define CURVE_45_LENGTH_MM     70.0f    // 45° turn between a cell centreline and a diagonal
#define DIAG_LEAD_MM           53.52f   // Straight from a cell centre before the 45° turn onto a diagonal
#define DIAG_TRIM_MM           36.48f   // Diagonal distance each 45° turn takes off the diagonal straight
#define DIAG_HALF_STEP_MM      127.279f // One diagonal half-step: edge midpoint to edge midpoint (90 * sqrt 2)
#define CURVE_V90_LENGTH_MM    99.15f   // 90° turn from one diagonal onto the next (a "V"), around a cell-edge midpoint
#define V90_TRIM_MM            60.0f    // Diagonal distance that turn takes off each of the two diagonals (99.15 * 0.60514)
#define V90_SPEED_RATIO        0.8f     // The V turn is tighter than the others, so it is taken this much slower


// Motor Feedforward: effort = FF_KS (to overcome friction) + FF_KV * speed + FF_KA * acceleration.
// This supplies most of the motor effort directly from the planned motion; the PID loops only
// correct what is left over. Measure the three numbers from a run log (console command "log").
// Wheel speed at 100% effort on a full supply: 700 mm/s for an N20 30:1 on a 24 mm wheel, and in
// proportion for the wheel actually fitted (a bigger wheel covers more ground per motor turn)
#define MOTOR_NO_LOAD_SPEED_MM_S (700.0f * WHEEL_DIAMETER_MM / 24.0f)
#define FF_KS                  0.03f    // Effort needed just to keep a wheel turning
#define FF_KV                  (1.0f / MOTOR_NO_LOAD_SPEED_MM_S) // Effort per mm/s of wheel speed
#define FF_KA                  0.0f     // Effort per mm/s² of acceleration (0 until measured)
#define MOTOR_NOMINAL_VOLTS    12.0f    // Motor supply the numbers above assume; efforts are rescaled if it sags

// Run Log: the last RUN_LOG_SAMPLES snapshots of target vs. actual motion, kept in RAM while driving
#define RUN_LOG_SAMPLES        1500     // 30 seconds at 50 Hz (36 kB of RAM)
#define RUN_LOG_DIVIDER        10       // Record every 10th control tick (50 Hz)


// How far the two 90° side sensors sit in front of the wheel axle. It sets where in a cell the
// robot is when a side sensor sees a wall start or end. Observed by the owner (2026-10-08): the
// 90° sensors see into the next cell about 110 mm after a cell centre, i.e. 20 mm past the cell
// edge, which puts them about 20 mm BEHIND the axle (negative). Refine with a ruler when possible.
#define SIDE_SENSOR_AHEAD_MM   -20.0f
#define POST_EDGE_PHASE_MM     (HALF_CELL_SIZE_MM - SIDE_SENSOR_AHEAD_MM) // Distance past a cell centre at that moment

// Front-wall distance check. When the robot has stopped in a cell with a wall in front, the
// front sensors say how far that wall is, compared with the reading learned by calibrating in
// the middle of a cell facing a wall. If it is not standing in the middle, it shuffles forward
// or back before doing anything else. This catches what the wheel encoders get wrong.
// Only active once the front level has been measured against a real wall ("Calibrate sensors").
#define ENABLE_FRONT_WALL_DISTANCE_FIX 1

// Front squaring: turning the robot to face a front wall squarely by comparing the two front
// sensors, both while rolling up to the wall and standing in a dead end before turning round.
// OFF at the owner's request (2026-10-08). While it is 0 there is no wall steering at all while a
// front wall is in view (the heading is held on the IMU), and a dead end is just a 180° turn on
// the spot.
#define ENABLE_FRONT_SQUARING  0        // 1 = square up on front walls, 0 = never
#define FRONT_SENSOR_AHEAD_MM  38.0f    // Front sensor tips ahead of the wheel axle (from the PCB drawing)
#define FRONT_SENSOR_TO_WALL_MM (HALF_CELL_SIZE_MM - 6.0f - FRONT_SENSOR_AHEAD_MM) // ...so this far from the wall at a cell centre
// Stopping at a front wall by the front sensors rather than the wheel encoders: when a wall comes
// into view ahead while the robot rolls into a cell, the rest of the move is set from the measured
// distance to that wall, so it stops in the middle of the cell and the check above has nothing
// left to correct. Like the check, it only acts once the front level has been measured.
#define ENABLE_FRONT_WALL_STOP 1        // 0 = stop by the encoders alone, as before
#define FRONT_STOP_TRUST_MM    30.0f    // The sensors may move the stopping point this far from where the encoders put it
// Where the robot stands in front of a wall: this much further from the wall than the cell centre
// (owner's request, 2026-10-08). Both the stop above and the front-wall check aim for that point,
// and the check's printed "short of / past the cell centre" is measured from it. 0 = the centre.
// Back to 0 on 2026-10-09 (was 5): the robot was by then stopping well short of the centre, and the
// lost distance added up from wall to wall (owner approved the change from the app).
#define FRONT_STOP_EXTRA_MM    0.0f
// How fast the front reading falls off with distance: reading ~ 1 / distance ^ this. 2 would be
// the textbook inverse-square rule. First set to 1.5 from one run; the robot still stopped short,
// and four readings taken one cell back from a wall fitted 1.1 to 1.3, so it is 1.2 now (owner's
// go-ahead, 2026-10-09). Lower it if the robot still stops short of front walls, raise it if it
// gets too close. To pin it down: `ir` at a cell centre facing a wall, and one cell further back.
#define FRONT_FALLOFF_EXPONENT 1.2f
#define FRONT_FIX_GAIN       1.0f     // Share of the measured error that is corrected (was 0.7 while the distance rule was off)
#define FRONT_FIX_MIN_MM       3.0f     // Smaller errors than this are left alone
#define FRONT_FIX_MAX_MM       20.0f    // Never shuffle further than this, whatever the sensors say

// Settling: a move that ends in a stop is not finished until the robot is actually there.
#define SETTLE_DISTANCE_MM     2.0f     // Close enough along the direction of travel
#define SETTLE_HEADING_DEG     1.5f     // Close enough in heading
#define SETTLE_SPEED_MM_S      20.0f    // Slow enough to call it stopped
#define SETTLE_YAW_RATE_DEG_S  20.0f    // Turning slowly enough to call it stopped (else it brakes mid-swing and coasts past)
#define SETTLE_DWELL_TICKS     50       // Must stay on target and still for this many control ticks in a row (0.1 s)
#define SETTLE_TIMEOUT_TICKS   150      // Give up waiting after this many control ticks (0.3 s)
#define SETTLE_TIMEOUT_TURN_TICKS 500    // ...or this many after a turn on the spot (1 s): a wrong heading costs more than a pause
#define TURN_SETTLE_PUSH       0.04f    // Extra effort (on top of ff_ks) that nudges the robot onto its heading after a turn
                                        // on the spot. Raise if turns still end a few degrees off, lower if it twitches. 0 = off.
#define TURN_SETTLE_PUSH_MAX_RATE_DEG_S 10.0f // No nudge while already turning toward the heading faster than this
#define TURN_SETTLE_PUSH_FULL_DEG 6.0f  // The nudge is at full strength this far off the heading and fades to ff_ks alone at zero
// Turn-rate damping (effort per deg/s of turn rate from the gyro, opposing it). For scale: holding
// a turn of 1 deg/s takes about 0.0005 effort. 0 = off. Lower these if the robot buzzes or feels sluggish.
// These two and TURN_SETTLE_PUSH are only starting values: the app's Tuning card changes them live
// (s_damp, t_damp, t_push), and a set saved on the robot overrides them.
#define TURN_SETTLE_DAMPING    0.0004f  // While settling after a turn on the spot
#define STRAIGHT_YAW_DAMPING   0.0f     // On straights and diagonals. OFF at the owner's request (2026-10-08); try 0.0002 from the app (s_damp)
#define CHAIN_TIMEOUT_TICKS    250      // Moves starting within 0.5 s of the last one carry on from where it aimed to end

// Search Look-Ahead (smooth turns during the search run)
// The 45° sensors point forward and outward, so while the robot drives from one cell centre to the
// next cell's edge they are already looking at the NEXT cell's side walls. If that says "open" and
// the 90° sensor agrees once the robot reaches the edge, the robot curves straight through the
// corner instead of driving to the centre, stopping, and turning on the spot. Half-way round the
// curve the outer 45° sensor faces that cell's front wall squarely and records it.
#define ENABLE_SEARCH_LOOKAHEAD 1       // 0 = always decide at the cell centre and turn on the spot
#define SEARCH_CONFIRM_WITH_90  0       // 1 = at the cell edge the 90° sensor must agree with the 45° look-ahead before a
                                        //   curve. Only right if the 90° sensors sit well ahead of the wheel axle, so that at
                                        //   the edge they are already beside the NEXT cell's wall. 0 = trust the 45° alone:
                                        //   on this robot the 45° sees an opening before the 90° reaches it (owner, 2026-10-08)
#define SEARCH_LOOKAHEAD_START_MM 40.0f // Side walls of the next cell are sampled between these
#define SEARCH_LOOKAHEAD_END_MM   80.0f //   distances after leaving a cell centre
#define SEARCH_OPEN_RATIO      0.6f     // "Definitely open" = every sample below this fraction of the wall threshold
#define SEARCH_FRONT_SAMPLE_FROM 0.40f  // Part of a 90° curve (0..1) during which the outer 45° sensor
#define SEARCH_FRONT_SAMPLE_TO   0.60f  //   faces the front wall of the cell being curved through
#define PREVIEW_MIN_SAMPLES    10        // Fewer samples than this = no opinion
// Curve after curve (a U-turn in one motion): at the end of a 90° curve the robot is on the entry
// edge of the next cell and nearly square to it, so its two 45° sensors are reading that cell's
// side walls, just as they do at the end of a straight. They are sampled once the heading is
// within this many degrees of the curve's final heading; with that the search can curve again
// straight away instead of driving to the cell centre and turning on the spot.
#define ENABLE_SEARCH_CHAINED_CURVES 1  // 0 = always drive to the cell centre after a curve
// Whether the 45° readings taken as a curve ends may decide a second curve into a cell the robot
// has never visited. OFF since 2026-10-08: on the robot they called a walled side open twice running
// and it curved into the wall. With 0, curves are only chained through cells it has visited before.
#define SEARCH_CHAIN_CURVES_BY_SENSORS 0
#define SEARCH_CURVE_SIDE_SAMPLE_DEG 8.0f

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

// Speed Tiers: every speed run that finishes moves the next one up a tier; a speed run that is
// aborted (crash, stall, lifted off the maze) moves it back down. Power-on starts at tier 1.
#define SPEED_TIER_1_SCALE     0.35f    // Fraction of the SPEEDRUN_* speeds and accelerations used (slowed for bring-up) [0.60]
#define SPEED_TIER_2_SCALE     0.80f
#define SPEED_TIER_3_SCALE     1.00f


// ==============================================================================
// 3. MAZE GEOMETRY
// ==============================================================================

#define MAZE_WIDTH             16       // Storage is always sized for a full competition maze
#define MAZE_HEIGHT            16

// How much of that grid is actually maze.
//   5  = a 5x5 practice maze in the same corner: start cell (0,0), goal = the centre cell (2,2).
//   3  = the 3x3 practice maze in the bottom-left corner: start cell (0,0), goal = the centre
//        cell (1,1), everything outside it treated as solid wall.
//   16 = a full competition maze, goal = the four centre cells.
// SET TO 3 FOR NOW at the owner's request (2026-10-07), so the ordinary upload runs the practice
// maze. The `competition` build always uses 16 whatever is written here. Change this back to 16
// when moving on to a full maze with the `main` build.
//
// Since 2026-10-08 (owner's request) the size can be switched while the robot is on: the app's
// maze boxes, or "maze 3" / "maze 5" / "maze 16" on the console. The choice is kept in flash, and each size
// keeps its own saved map. MAZE_DEFAULT_SIZE is only what a robot that has never been told uses.
// A build that sets MAZE_ACTIVE_SIZE itself (`competition` = 16, `test3x3` = 3) is fixed at that.
#ifndef MAZE_ACTIVE_SIZE
#define MAZE_SIZE_SWITCHABLE   1
#define MAZE_DEFAULT_SIZE      3
extern int g_maze_size;                 // Lives in navigation/maze/maze.cpp
#define MAZE_ACTIVE_SIZE       g_maze_size
#endif

// Wall Memory: every sensor reading of a wall is a vote, +1 for "wall" and -1 for "open".
// A wall is believed while its votes are above zero, so one bad reading is outvoted the next
// time the robot looks. Speed runs only use openings that have actually been seen open.
#define WALL_VOTE_LIMIT        3        // Votes stop counting at +/- this, so beliefs can still change
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
#define PRIORITY_NAV_TASK      10       // Decides the next move the instant the last one finishes; nothing else
                                        //   of ours on its core may hold it up (the Wi-Fi / Bluetooth stacks still can)
#define PRIORITY_OPERATOR_TASK 3        // Console, phone-app commands, reports, LED, hand waves
#define PRIORITY_TELEMETRY     2        // Low-priority logging and the Wi-Fi page
#define NAV_WATCH_PERIOD_MS    5        // How often the navigation task also checks for a safety stop during a run

// Fault tolerance (see docs/INSTRUCTIONS.md section 8.3)
#define MOTORON_HEALTH_PERIOD_MS   200  // How often the Motoron status flags are polled for resets / bus faults
#define MOTORON_KEEPALIVE_MS       100  // Re-send interval so the Motoron's 250 ms command timeout never trips
#define IMU_MAX_STEP_DEG           30.0f // A heading jump larger than this between two 100 Hz reads is rejected as a glitch
#define IMU_GLITCH_READS           3    // A far-off heading this many reads running is believed, not rejected
#define IMU_FAULT_READS            10   // Consecutive bad reads (100 ms) before falling back to encoder odometry
// Accelerometer on show (owner's request, 2026-10-08): the BNO055's acceleration is read 20 times
// a second and shown in the app, to judge whether it could ever help measure distance. Nothing in
// the robot uses it. It costs one extra I2C read every 25th control tick; set to 0 to drop that.
// Follows the Wi-Fi switch, so the `competition` build (no app to show it in) never makes the read.
#define ENABLE_ACCEL_DISPLAY       ENABLE_WIFI_OTA
#define IMU_FILTER_ALPHA           0.6f  // Heading smoothing: share of each new BNO055 reading that is believed (1 = no smoothing)
#define ENCODER_FAULT_TICKS        75   // Control ticks (150 ms) one wheel may read zero while the other is moving
#define HEADING_FAULT_DEG          60.0f // Heading this far off course = crashed or picked up (lift + twist to stop a run)
#define HEADING_FAULT_TICKS        50   // ...for this many control ticks (100 ms) before the run is aborted
#define TILT_STOP_DEG              45.0f // Tipped this far out of level = picked up (lift + tip it up to stop a run)
#define TILT_STOP_TICKS            150  // ...for this many control ticks (0.3 s, 6 tilt readings) before the run is aborted

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

// ==============================================================================
// 8. DEBUG LOG
// ==============================================================================
// Everything the firmware prints with `Serial` goes to the USB serial monitor as it always did,
// and is also copied into a small buffer that the wireless debug links (Bluetooth, Telnet) send
// on. So the web dashboard and a Telnet session show exactly what the serial monitor shows.
// (Skipped when the code is compiled for the PC tests.)

#define DEBUG_LOG_BUFFER_BYTES 4096     // Recent output kept for the wireless links; oldest is dropped if they fall behind

#ifdef ARDUINO
inline decltype(Serial)& usbSerial() { return Serial; } // The real USB serial port

class DebugLog : public Print {
public:
    void begin(unsigned long baud) { usbSerial().begin(baud); }
    int available() { return usbSerial().available(); }
    String readStringUntil(char terminator) { return usbSerial().readStringUntil(terminator); }

    size_t write(uint8_t c) override { return write(&c, 1); }
    size_t write(const uint8_t* data, size_t length) override;

    // Takes up to `max_length` bytes of not-yet-sent output out of the buffer. Returns how many.
    size_t drain(char* out, size_t max_length);
};
extern DebugLog g_debug_log;

#define Serial g_debug_log // From here on, every Serial.print in the firmware goes through the copy
#endif

