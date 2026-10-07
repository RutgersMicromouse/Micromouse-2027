# Antigravitieee Micromouse — context for AI assistants

Read this first. It is the handoff file: what the robot is, what is actually proven, what the owner wants, and what to do next. Keep it up to date when you change anything it describes.

## The robot

- ESP32-S3-DevKitC-1 (dual core), Pololu Motoron M2T256 motor driver (I2C), two **N20** 12 V 30:1 gearmotors with 7-pulse magnetic encoders, Bosch BNO055 IMU (I2C, same bus), six pulsed IR wall sensors (SFH4545 emitters, TEFT4300 receivers).
- **No buttons, no switches, no LED on the PCB.** The only indicator is the RGB LED on the DevKit board. The only input is a hand waved in front of the front IR sensors.
- The schematic in `docs/` is out of date: it shows Faulhaber 1524 motors (now N20), and its IR channel numbering does not match the physical sensor positions. `src/config.h` is the truth for pins.

## What is proven on hardware, and what is not

**Only two branches contain code that has run correctly on the real robot.** Treat them as ground truth whenever anything disagrees:

| Branch | What it proved |
| :--- | :--- |
| `release/Varun9431/antigravitieee/Peripheral-Test` | Standalone motor, IMU, and IR sketches: I2C pins 21/20, 12 V enable on GPIO 13, Motoron channel 1 = right / 2 = left, IR emitters fired in groups of three with a 300 µs settle |
| `release/shrivastavaii/antigravitieee/tests_and_bluetooth` | Per-peripheral tests (now in `bench/`), BLE telemetry + web dashboard, battery reading, encoder pins (right 4/5, left 6/7), BNO055 heading sign convention, and the **IR channel-to-position map** (the owner said to use this one) |

Everything else in `src/` (motion control, navigation, hand-wave UI, fault recovery) **has never driven the robot.** Do not describe it as working. The navigation code (`navigation.cpp`) is additionally run on the PC against every maze by `sim/tests/test_firmware_nav.py` with an ideal robot, so its logic and path geometry are tested; the motion controller, drivers, and UI are only compiled. PID gains, speeds, IR thresholds, and gesture thresholds are starting values.

Pieces taken straight from the proven branches: the IR pin map and group firing, the Motoron channel/pin setup, encoder pins and PCNT setup, the BNO055 register sequence and heading convention, the BLE services.

## What the owner wants (standing instructions)

1. **Fewest files possible**, and code that is extremely easy to navigate and read. One header + one source per subsystem. Do not add files, folders, planning documents, or abstractions without being asked.
2. **`src/` contains only what goes on the robot.** Tests and calibration live in `bench/`.
3. **Hand waves are the only way to start anything** (runs, mode selection, IR calibration, clearing the maze). The text console is for debugging and watching only: it may report, adjust settings, and stop, but never start. Wireless is not allowed at competition, so nothing may depend on it.
4. **Continuous motion.** The robot should keep rolling through straights, smooth curves, and diagonals, stay on the planned trajectory, and not clip posts on diagonals.
5. More redundancy: a missing or glitching peripheral should degrade the robot, not kill it.

## Layout

```
platformio.ini      build targets (see below)
src/                the firmware, 14 files
  main.cpp          setup + the three FreeRTOS tasks
  config.h          every pin, dimension, speed, threshold      <- edit this first
  types.h           shared structs/enums + angle helpers
  robot.h           the shared global objects and who may touch them
  hardware.*        Encoders, Motors, IRSensors, IMU
  control.*         PIDController, TrapezoidalProfile, MotionController
  navigation.*      Maze, Floodfill, Dijkstra, Decomposer, Navigator
  ui.*              StatusLED, Actions, GestureInput, GestureUI, Console
  wireless.*        BLEDebug, WifiOTA (debug only)
bench/              one file per bench test + calibration.cpp + battery.h
sim/                Python simulators, mazes, sim/tests/ (test_firmware_nav.py runs the real navigation code)
tools/              web_dashboard/ (BLE telemetry page), dyno_station/ (separate firmware)
docs/               INSTRUCTIONS.md (operator manual + engineering constraints), schematic
```

Inside each merged file, sections are separated by `// ====` banners naming the class; search for the banner to jump.

## How it works, in one screen

- **Core 1, `motionControlTask`, 500 Hz:** encoders → IR (one emitter group per tick) → IMU → take a `MotionCommand` from `g_motion_cmd_queue` → `MotionController::update` → motors. Publishes a telemetry snapshot at 50 Hz.
- **Core 0, `navigationTask`:** debug console, safety-stop handling, `Navigator::step` each time a motion finishes, hand-wave UI while idle.
- **Hand waves** (`GestureUI`): 1 search, 2 hybrid speed run, 3 diagonals, 4 curves, 5 IR calibration, 6 clear maze. White blink per wave, count blinked back, 2 s countdown, cover the sensors to cancel. Ignored during a run.
- **Stopping a run:** lift the robot and turn it sideways (lost-heading protection), or `stop` on the debug console. Stall and dead-encoder protection also abort the run.
- **Every run starts in the start cell facing into the maze.** The operator puts the robot there before waving; `startSearchRun` and `startSpeedRun` both reset the pose to (0,0) facing north.
- **The map is kept between attempts.** A search never wipes the maze: it carries on from what earlier attempts learned (in RAM and in flash). The map is written to flash only while the robot is standing still (turns on the spot, dead ends, reaching the centre or the start) and when a run is aborted, because a flash write stalls the processor for a few milliseconds. Six waves forget the map. If the remembered map leaves no way out of the start cell it is forgotten automatically, once.
- **Search = out and back.** The same stepping (`Navigator::step`) drives to the centre, then explores its way back to the start with unexplored cells treated as open, so the return leg covers new ground. At the end `isBestRouteExplored()` says whether the shortest possible route runs only through seen cells; the LED blinks green twice if so, yellow twice if another search (one wave) might find a shorter one.
- **Speed tiers** (`Actions`, `SPEED_TIER_*_SCALE`): speed runs start at tier 1 (60 % of the `SPEEDRUN_*` speeds). Each speed run that reaches the centre moves the next one up a tier; an aborted one moves it down. The tier is blinked in blue (1–3) before the countdown. It resets to 1 at power-on.
- **Search run:** rolls straight through known cells at full search speed and into unknown cells at `SEARCH_PROBE_SPEED_MM_S`. Turns are taken as smooth 90° curves whenever the look-ahead is certain (below); otherwise the robot drives to the cell centre, reads all its sensors, and turns on the spot.
- **Search look-ahead** (`ENABLE_SEARCH_LOOKAHEAD`, `Navigator::stepAtCellEdge`): the 45° sensors point forward and outward, so on the way from a cell centre to the next cell's edge they are already reading the next cell's side walls (`MotionController::samplePreview` keeps the min/max over a travel window). At the edge the 90° sensors must agree with them; a side counts as open or walled only if both sensors say so, anything else falls back to the centre decision. If the floodfill then prefers a turn (with the unseen front wall assumed open, turning is then best whatever the front turns out to be), the robot curves through the cell. Half-way round the curve the outer 45° sensor faces that cell's front wall squarely and records it; the cell is marked visited only if that reading was clear either way. Known cells where the plan turns are curved through the same way without needing the sensors.
- **Return and speed runs** (`Navigator::queueSegment`): straight→straight at right angles uses a smooth 90° curve from cell edge to cell edge; diagonals are entered and left with smooth 45° curves; consecutive moves hand over at speed. Turns on the spot remain only for 180° reversals, sideways diagonal entries, and slalom zigzags. The hybrid mode charges `SPEEDRUN_STOP_PENALTY_S` per such stop when it compares the curve route with the diagonal route.
- **Exact stops in the search:** forward moves carry `stop_at_front_wall`; if the front sensors find a wall while rolling into a cell, the motion controller re-plans the move to come to rest exactly on the cell centre. If the robot has to stop for any other reason after rolling in at speed, the navigator brakes and then backs up the few millimetres of overshoot before turning, so turns on the spot never leave it off-centre.
- **Smooth turns** follow `heading = angle·(3u² − 2u³)` with `u` = distance along the curve, plus wheel-speed feedforward. The lengths in `config.h` (`CURVE_90_LENGTH_MM`, `CURVE_45_LENGTH_MM`, `DIAG_LEAD_MM`, `DIAG_TRIM_MM`) were derived together so turns land exactly on the grid; change them only as a set.
- **Motor feedforward:** `effort = FF_KS + FF_KV·speed + FF_KA·acceleration` (constants in `config.h`) supplies most of the effort for straights, curves, and turns on the spot; the PID loops only trim. `Motors` rescales every effort by `MOTOR_NOMINAL_VOLTS / measured supply` (the Motoron reports its VIN; the motor rail is a 12 V boost converter, so this is normally close to 1).
- **IR processing:** ambient subtraction → median of the last 3 samples (kills single-sample spikes) → low-pass. Wall centring works in relative distance, `sqrt(centred reading / reading)`, so the steering error is proportional to how far off-centre the robot is. This assumes inverse-square fall-off; it has the same small-signal gain as the old raw-difference error, so the centring PD gains carry over.
- **Run log:** `MotionController` records target vs. actual speed, heading, heading error, efforts, and all six IR readings at 50 Hz into a 30 s RAM ring buffer while a move is running. `log` on the debug console (USB or Telnet) dumps it as CSV; `log clear` empties it.
- **Diagonals:** two corrections act on the heading while on a diagonal. Centring: each 45° sensor points along a maze axis there and sees posts pass one at a time; the controller keeps a distance-fading peak per side and steers toward the middle of the two rows (`DIAG_CENTER_GAIN_DEG`). Guard: if either sensor reads a post closer than it should ever be, it steers away hard (`DIAG_GUARD_RATIO`). The combined trim is limited to `DIAG_GUARD_MAX_TRIM_DEG`.
- **Post-edge distance correction:** when a side sensor sees a wall start or end, the distance along the cell is snapped to `POST_EDGE_PHASE_MM`. Works on straights that begin at a cell centre or (via `MotionCommand::start_offset_mm`) at a cell edge after a smooth curve.
- **Redundancy:** Motoron start-up retries, keep-alive, and automatic reconfiguration after a driver reset or bus fault; IMU glitch rejection, encoder-odometry fallback, automatic recovery and gyro-polarity cross-check; stall, dead-encoder, and lost-heading stops; low-battery cutoff. `health` on the console shows the counters.

## Build and check

PlatformIO is not on PATH on the owner's Windows machine: use `~/.platformio/penv/Scripts/pio.exe`.

```
pio run                         # main (debug firmware)
pio run -e competition          # radios off, what runs at the event
pio run -e ota -t upload        # main, over the robot's Wi-Fi
pio run -e calibration          # main + motorcal / motorrpm / drivecal / turncal / dyno commands (bench/calibration.cpp)
pio run -e ir_test              # bench tests: motor_test ir_test imu_test battery_test ble_test wifi_test
python sim/tests/test_firmware_nav.py                 # the real navigation.cpp on all 10 mazes (needs: pip install ziglang)
python sim/tests/test_firmware_nav.py classic --verbose   # one maze, showing what the firmware prints
python sim/tests/test_desktop_suite.py
python sim/tests/test_physics_model.py
python sim/verify_headless.py --mazes 1 --trials 1     # all 10 mazes takes several minutes
```

`test_firmware_nav.py` is the one test that runs firmware code. It compiles `src/navigation.cpp` for the PC with zig (`pip install ziglang`, already installed on the owner's machine; any `g++` / `clang++` on PATH also works), stubs the few ESP32 headers it needs, and drives the real `Navigator` with an ideal robot: commands are carried out exactly, sensors report the true maze, and every 2 mm of path is checked for clearance. It runs each maze twice, with the look-ahead sensing answering truthfully and answering "not sure". **Run it after any change to `navigation.*`, `types.h`, or the constants in `config.h`.** It does not model sensor noise, slip, or the motion controller.

The other Python simulators in `sim/` are a separate implementation of the navigation ideas; they do **not** run the firmware, so passing them says nothing about `src/`.

## What to do next (needs the real robot)

Bring-up order, each step proving the layer below the next:

1. Flash `main`, open the serial monitor, send `health`. Motoron and IMU should say OK.
2. Watch the `[TEL]` line: wave a hand at each IR sensor and confirm L90 / L45 / FL / FR / R45 / R90 respond in the right position. Send `perf` and confirm the loop time is under 1200 µs with no overruns (the IR timing is the main unknown).
3. Push the robot by hand and send `enc`: forward should count up on both sides. Fix with `encinv` / `motorinv`, then make it permanent in `config.h`.
4. Turn the robot by hand: heading should increase turning left.
5. Wave once and confirm the count blink and countdown. Tune `GESTURE_MIN_RISE` if waves are missed or invented.
6. Calibrate IR in a cell (5 waves), then try a search run in a small maze with a hand ready to lift it. Do the first runs with `ENABLE_SEARCH_LOOKAHEAD 0` (stop-and-turn search), then switch it on once straights and on-the-spot turns are trustworthy.
7. With the `calibration` build, measure the wheels: `drivecal` then `drivecal result <measured mm>` gives the true `WHEEL_DIAMETER_MM`; `turncal` gives the true `WHEEL_BASE_MM`. Put both in `config.h`. Every distance and the turn feedforward depend on them.
8. After each run send `log` and plot it. Tune in this order: `FF_KV` until `speed_mm_s` matches `target_mm_s` at cruise with `forward_pct` mostly explained by feedforward; `FF_KS` from the effort needed at very low speed; `FF_KA` from the extra effort during acceleration; then the PID gains in the `MotionController` constructor (straights first, then turns). `POST_EDGE_PHASE_MM` comes from where the L90 / R90 columns step relative to distance travelled.
9. Only then try speed runs. They start at tier 1 (60 %) by themselves; lower `SPEED_TIER_1_SCALE` if even that is too fast.

Known weak spots to check first if something misbehaves: `Wire.setTimeOut(1)` may be too tight for BNO055 clock stretching (watch the IMU dropout counter); the feedforward constants are estimates from the motor's nominal no-load speed, and the velocity PID gains were chosen before feedforward existed, so expect to lower them if speed overshoots; the search look-ahead windows (`SEARCH_LOOKAHEAD_START_MM` / `_END_MM`, `SEARCH_FRONT_SAMPLE_*`) assume the sensors sit roughly 40 mm ahead of the wheel axle, and if they are wrong the robot simply falls back to stop-and-turn rather than misbehaving; the diagonal centring gain is a guess; diagonal routes still stop and turn on the spot where the path doubles back on itself (a V-shaped reversal, or a diagonal entered sideways), because there are no smooth 135° or diagonal-to-diagonal 90° turns yet (the PC harness shows how many stops each route has); the slalom segment still stops at each reversal; the hybrid mode's time estimate for choosing between curves and diagonals is crude (a fixed time per segment plus a penalty per stop), so check its choice against the stop counts the PC harness prints; the diagonal post guard threshold (`DIAG_GUARD_RATIO`) and `POST_EDGE_PHASE_MM` are calculated guesses.

## Ideas considered and deliberately not built yet

Each of these needs measurements from the real robot first; building them blind would add untestable code.

- **Smooth 135° and diagonal-to-diagonal 90° turns**, which would remove the remaining stops on diagonal routes. Pure geometry, so it can be derived and checked with `test_firmware_nav.py` without the robot; it was left out only for time.
- **Front-wall distance correction** before stopping, and full x/y position tracking with wall-based correction. Needs IR readings converted to real distances per sensor (a two-point calibration, not the single centred point available now).
- **Whole-path speed planning** with a grip limit in curves. Needs measured tyre grip and motor limits from run logs.
- **Raw gyro at 1 kHz** instead of the BNO055's 100 Hz fused heading. Only matters above roughly 1 m/s, and would mean reconfiguring the IMU out of fusion mode.
- **Kalman filter.** Not worth it here: one gyro, two encoders, and exact corrections from walls are served well by the simpler mechanisms above.
- **Derivative-on-measurement in the PIDs.** Rejected: these loops track moving targets, where derivative on error is the wanted behaviour.

## Working notes for AI assistants

- When scripting edits to C++ through a shell heredoc on this machine, backslash escapes get mangled (`\n` becomes a real newline). Use the editor tools, or write the script to a file first.
- Do not commit or push unless asked. Feature branches are named `user/<name>/<bot>/<topic>` off `release/<bot>`.
- After any firmware change: build `main`, `competition`, and `calibration` at minimum.
