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

Everything else in `src/` (motion control, navigation, hand-wave UI, fault recovery) **has never driven the robot.** Do not describe it as working. What it has had is PC testing, at two levels (see "Build and check"):

- `navigation.cpp` is run against thousands of mazes with an ideal robot (`test_firmware_nav.py`). Its logic and path geometry are well tested.
- `hardware.cpp` + `control.cpp` + `navigation.cpp` together drive a simulated robot (`test_firmware_drive.py`). It passes on 20 random 3x3 practice mazes, 5 random full-size mazes and 3 stock mazes: IR calibration, searches until the best route is proven, and speed runs at tiers 1 and 2 in both curve and diagonal modes, with no wall contact and no safety stops. Tier 3 (full speed) is reported but not required: on 3 of the 5 random full-size mazes the full-speed diagonal run comes within a millimetre or two of a post in the simulation, which is a tuning matter (see weak spots). Getting it to pass took fixing six real bugs in code that had only ever been compiled, listed under "Bugs the simulated robot found".
- `ui.cpp`, `wireless.cpp`, and `main.cpp` are only compiled.

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
  navigation.*      Maze, Floodfill, Dijkstra, Navigator (search stepping + speed-run planner)
  ui.*              StatusLED, Actions, GestureInput, GestureUI, Console
  wireless.*        BLEDebug, WifiOTA (debug only)
bench/              one file per bench test + calibration.cpp + battery.h
sim/                Python simulators, mazes, sim/tests/ (test_firmware_nav.py runs the real navigation code)
tools/              web_dashboard/ (BLE telemetry page), dyno_station/ (separate firmware)
docs/               INSTRUCTIONS.md (operator manual + engineering constraints), schematic
```

Inside each merged file, sections are separated by `// ====` banners naming the class; search for the banner to jump.

## How it works, in one screen

- **Core 1, `motionControlTask`, 500 Hz:** encoders → IR (front + sides one tick, diagonals the next) → IMU → take a `MotionCommand` from `g_motion_cmd_queue` → `MotionController::update` → motors. Publishes a telemetry snapshot at 50 Hz.
- **Core 0, `navigationTask`:** debug console, safety-stop handling, `Navigator::step` each time a motion finishes, hand-wave UI while idle.
- **Hand waves** (`GestureUI`): 1 search, 2 hybrid speed run, 3 diagonals, 4 curves, 5 IR calibration, 6 clear maze. White blink per wave, count blinked back, 2 s countdown, cover the sensors to cancel. Ignored during a run.
- **Stopping a run:** lift the robot and turn it sideways (lost-heading protection), or `stop` on the debug console. Stall and dead-encoder protection also abort the run.
- **Every run starts in the start cell facing into the maze.** The operator puts the robot there before waving; `startSearchRun` and `startSpeedRun` both reset the pose to (0,0) facing north.
- **Practice maze build (`test3x3`):** sets `MAZE_ACTIVE_SIZE` to 3. The maze is the 3x3 block in the bottom-left corner, start cell (0,0) facing north, goal = the centre cell (1,1), everything outside treated as solid wall. It keeps its own saved map. Everything else is identical to `main`, so what works there is the same code that runs in the full maze. `Maze::isGoalCell` and `Maze::inMaze` are the only places the size matters.
- **Walls are votes, not facts** (`Maze`, `WALL_VOTE_LIMIT`): every sensor reading of a wall adds +1 ("wall") or -1 ("open") to that wall's tally, capped at +/-3. A wall is believed while its tally is above zero; an opening nobody has looked at counts as open for the search. Driving through an opening sets it to -3 (`confirmOpen`). Speed runs and the "best route explored" check only use openings whose tally is below zero (`isKnownOpen`). If the map ever says the robot is walled in, it first drops walls resting on one reading, then forgets the map and explores afresh from where it is. The navigator also never curves toward a side where either sensor currently sees a wall, whatever the map says.
- **The map is kept between attempts.** A search never wipes the maze: it carries on from what earlier attempts learned (in RAM and in flash). The map is written to flash only while the robot is standing still (turns on the spot, dead ends, reaching the centre or the start) and when a run is aborted, because a flash write stalls the processor for a few milliseconds. Six waves forget the map.
- **Search = out and back.** The same stepping (`Navigator::step`) drives to the centre, then explores its way back to the start with unexplored cells treated as open, so the return leg covers new ground. At the end `isBestRouteExplored()` says whether the shortest possible route runs only through seen cells; the LED blinks green twice if so, yellow twice if another search (one wave) might find a shorter one.
- **Speed tiers** (`Actions`, `SPEED_TIER_*_SCALE`): speed runs start at tier 1 (60 % of the `SPEEDRUN_*` speeds). Each speed run that reaches the centre moves the next one up a tier; an aborted one moves it down. The tier is blinked in blue (1–3) before the countdown. It resets to 1 at power-on.
- **Search run:** rolls straight through known cells at full search speed and into unknown cells at `SEARCH_PROBE_SPEED_MM_S`. Turns are taken as smooth 90° curves whenever the look-ahead is certain (below); otherwise the robot drives to the cell centre, reads all its sensors, and turns on the spot.
- **Search look-ahead** (`ENABLE_SEARCH_LOOKAHEAD`, `Navigator::stepAtCellEdge`): the 45° sensors point forward and outward, so on the way from a cell centre to the next cell's edge they are already reading the next cell's side walls (`MotionController::samplePreview` keeps the min/max over a travel window). At the edge the 90° sensors must agree with them; a side counts as open or walled only if both sensors say so, anything else falls back to the centre decision. If the floodfill then prefers a turn (with the unseen front wall assumed open, turning is then best whatever the front turns out to be), the robot curves through the cell. Half-way round the curve the outer 45° sensor faces that cell's front wall squarely and records it; the cell is marked visited only if that reading was clear either way. Known cells where the plan turns are curved through the same way without needing the sensors.
- **Speed runs** (`Navigator::planSpeedRun`): the whole run is planned up front as one unbroken chain of moves, by reading the cell path as a list of turns. A turn on its own is a smooth 90° curve from cell edge to cell edge. Turns that alternate (left, right, left...) are one diagonal, entered and left with smooth 45° curves. Two turns the same way in a row inside a diagonal are a smooth 90° "V" turn from one diagonal onto the next (`CURVE_V90_LENGTH_MM`, taken at `V90_SPEED_RATIO` of the curve speed). The robot only stops if the path reverses on itself, which a shortest path does not. On all ten test mazes every speed run has zero stops. The hybrid mode plans the run both ways (curves only, and with diagonals), adds up the time of every move, and drives the quicker one.
- **Exact stops in the search:** forward moves carry `stop_at_front_wall`; if the front sensors find a wall while rolling into a cell, the motion controller re-plans the move to come to rest exactly on the cell centre. If the robot has to stop for any other reason after rolling in at speed, the navigator brakes and then backs up the few millimetres of overshoot before turning, so turns on the spot never leave it off-centre.
- **Smooth turns** follow `heading = angle·(3u² − 2u³)` with `u` = distance along the curve, plus wheel-speed feedforward. The lengths in `config.h` (`CURVE_90_LENGTH_MM`, `CURVE_45_LENGTH_MM`, `DIAG_LEAD_MM`, `DIAG_TRIM_MM`) were derived together so turns land exactly on the grid; change them only as a set.
- **Motor feedforward:** `effort = FF_KS + FF_KV·speed + FF_KA·acceleration` (constants in `config.h`) supplies most of the effort for straights, curves, and turns on the spot; the PID loops only trim. `Motors` rescales every effort by `MOTOR_NOMINAL_VOLTS / measured supply` (the Motoron reports its VIN; the motor rail is a 12 V boost converter, so this is normally close to 1).
- **IR calibration (5 waves):** done in the start cell. The 45° sensors point forward as well as sideways, so they may be reading the next cell's side wall, which can have a gap; if one of them sees nothing, its level is estimated from the other side scaled by the two 90° sensors. It only fails if both 45° sensors see nothing.
- **IR firing pairs:** two emitters per control tick, three ticks per cycle: fronts, 90° sides, 45° diagonals (`kFiringPairs` in `hardware.cpp`), so each sensor updates at 167 Hz. The owner first asked for fronts+sides together, but on the robot four sensors in one tick took the loop to ~1.2 ms and it overran about one tick in five.
- **FIRST REAL READINGS FROM THE ROBOT (2026-10-07, over USB with the `ir`, `irtest`, `health`, `perf` console commands):** the Motoron supply read 27 V with the "256" scaling and reads 11.8 V with `MotoronVinSenseType::Motoron550` (now `MOTORON_VIN_TYPE`), so it had been scaling all motor effort down to 80 %; the IMU logged 433 failed reads with a 1 ms I2C timeout (now `I2C_TIMEOUT_MS` 3); loop time is ~710 µs with the three firing pairs but still shows roughly 5 % overruns and a 3.7 ms peak, cause not yet found; the logic battery reads 2.8 V (possibly wrong: its pin is on ADC2, which Wi-Fi interferes with). **The IR signal is extremely weak:** even at the most sensitive ADC range a lit emitter raises its own receiver by only 30-120 counts out of 4095, and a 2 ms on-time is no better than 300 µs, so it is not a timing problem. `irtest` (each emitter alone against all six receivers) showed the 45° and front pairs are mapped correctly; the two 90° sensors showed nothing, but it is not known whether any wall was near the robot during that test. Next step: repeat `irtest` with the robot in a cell with walls close on three sides. If readings stay this low it is a hardware problem (emitter supply, resistor values, sensor orientation), not firmware.
- **IR sensitivity** (`IR_ADC_ATTENUATION`, `IR_PULSE_SETTLE_US`): on the real robot the owner found a wall right in front of a sensor barely registered, so the receiver ADC pins are set to the most sensitive range (0 dB, about 3.5x the reading of the 11 dB the bench test used) and the emitter-on time is 500 µs instead of the proven 300. Not yet confirmed to be enough; if the owner still reports weak readings, get actual numbers from the dashboard before changing more, and suspect emitter power or wiring.
- **IR processing:** ambient subtraction → median of the last 3 samples (kills single-sample spikes) → low-pass. Wall centring works in relative distance, `sqrt(centred reading / reading)`, so the steering error is proportional to how far off-centre the robot is. This assumes inverse-square fall-off; it has the same small-signal gain as the old raw-difference error, so the centring PD gains carry over.
- **Run log:** `MotionController` records target vs. actual speed, heading, heading error, efforts, and all six IR readings at 50 Hz into a 30 s RAM ring buffer while a move is running. `log` on the debug console (USB or Telnet) dumps it as CSV; `log clear` empties it.
- **Diagonals:** two corrections act on the heading while on a diagonal. Centring: each 45° sensor points along a maze axis there and sees posts pass one at a time; the controller keeps a distance-fading peak per side and steers toward the middle of the two rows (`DIAG_CENTER_GAIN_DEG`). Guard: if either sensor reads a post closer than it should ever be, it steers away hard (`DIAG_GUARD_RATIO`). The combined trim is limited to `DIAG_GUARD_MAX_TRIM_DEG`.
- **Post-edge distance correction:** when a side sensor sees a wall start or end, the distance along the cell is snapped to `POST_EDGE_PHASE_MM`. Works on straights that begin at a cell centre or (via `MotionCommand::start_offset_mm`) at a cell edge after a smooth curve.
- **Redundancy:** Motoron start-up retries, keep-alive, and automatic reconfiguration after a driver reset or bus fault; IMU glitch rejection, encoder-odometry fallback, automatic recovery and gyro-polarity cross-check; stall, dead-encoder, and lost-heading stops. `health` on the console shows the counters. There is deliberately **no low-battery cutoff** (the owner had it removed): the battery voltage is only reported, by `status`.
- **Status LED on two pins:** the DevKit's RGB LED is on GPIO 48 (board v1.0) or GPIO 38 (v1.1); `StatusLED::set` drives both, since the owner reported seeing no LED at all.
- **Checking hand waves without the LED:** the firmware prints `[UI] Hand seen` and `[UI] Wave N counted` on USB, and `status` adds a `WAVES:` line with the front-sensor reading now, at rest, and the level a hand must exceed.
- **BRING-UP SETTINGS IN FORCE (owner's request, 2026-10-07), all in `config.h`, to be undone when the owner says so:** the default maze size is 3, so plain `main` runs the 3x3 practice maze (`competition` forces 16); search speeds are halved (normal values are in brackets beside each one) and speed tier 1 is 35 %; hand waves are off with an automatic start (next item); there is no low-battery cutoff.
- **HAND WAVES ARE TEMPORARILY OFF** (owner's request, 2026-10-07, for bring-up): `ENABLE_GESTURE_UI` is `0` in `config.h`. While it is 0 the robot starts by itself `AUTO_START_DELAY_S` (5) seconds after power-on: three white blinks, IR calibration where it stands, then one search run. This overrides standing instruction 3 for now; set `ENABLE_GESTURE_UI` back to `1` when the owner asks for waves again.
- **Phone app** (owner's request, 2026-10-07): the robot serves a page at `http://192.168.4.1` on its own Wi-Fi hotspot (`Antigrav-Mouse` / `micromouse`). It shows the state in words, six IR bars, heading, encoders, battery, driver and IMU health, and the robot's output, and has START (calibrate IR, then search), STOP, Speed run, Calibrate, Forget the maze, and a command box. The page is `APP_PAGE_HTML` in `wireless.cpp`; it polls `/data?since=N` four times a second (JSON built by `buildAppStatus` in `main.cpp` plus new log text) and sends buttons to `/cmd?c=...`, which are handled by `Console` like Telnet commands. **This means the console can start runs again** (`start`, `speedrun`, `calib`, `clear`), which overrides standing instruction 3 for builds that have a radio; the `competition` build compiles those commands out. `AUTO_START_DELAY_S` is 0 now that there is a START button.
- **Debug log copy** (`DebugLog`, `config.h` section 8): `Serial` is redefined to an object that prints to USB as before and also copies every byte into a 2 kB ring buffer, which `telemetryTask` forwards to Bluetooth and Telnet every 50 ms. So the wireless links show exactly what the serial monitor shows; console replies are simply printed. The only output that bypasses it is the `log` CSV dump, which is too big for the buffer.
- **Web dashboard** (`tools/web_dashboard`, served with `python -m http.server 8000` and opened at `http://localhost:8000`): the four live values over Bluetooth, a row of six live IR sensor bars with the walls seen and loop time (parsed from the `[TEL]` lines, which the page switches on itself with `stream on` when it connects; **keep the `[TEL]` line format and the page's regular expressions in step**), plus a "Robot output" panel at the bottom showing the rest of the debug log and a box for sending commands. It can listen over Bluetooth or over USB ("Connect by USB", Web Serial; the serial monitor must be closed first).
- **State in words:** the firmware prints `[STATE] ...` on USB whenever the navigator's state changes, and `status` on the console reports the same description plus the selected mode, speed tier and battery (`Actions::stateDescription`).
- **Telemetry stream:** the 5 Hz `[TEL]` line of sensor readings is off at power-on so the console stays readable. `stream` on the debug console toggles it (USB, Bluetooth and Telnet together).

## Build and check

PlatformIO is not on PATH on the owner's Windows machine: use `~/.platformio/penv/Scripts/pio.exe`.

```
pio run                         # main (debug firmware)
pio run -e competition          # radios off, what runs at the event
pio run -e test3x3              # main, for the 3x3 practice maze
pio run -e ota -t upload        # main, over the robot's Wi-Fi
pio run -e calibration          # main + motorcal / motorrpm / drivecal / turncal / dyno commands (bench/calibration.cpp)
pio run -e ir_test              # bench tests: motor_test ir_test imu_test battery_test ble_test wifi_test
python sim/tests/test_firmware_nav.py      # real navigation.cpp, ideal robot: 10 stock mazes, 250 random full-size,
                                           #   600 random 3x3, with and without sensors inventing walls
python sim/tests/test_firmware_drive.py    # real drivers + motion controller + navigator driving a simulated robot
python sim/tests/test_firmware_nav.py classic --verbose          # one maze, showing what the firmware prints
python sim/tests/test_firmware_drive.py --size 3 --random 1 --verbose
python sim/tests/test_desktop_suite.py
python sim/tests/test_physics_model.py
python sim/verify_headless.py --mazes 1 --trials 1     # all 10 mazes takes several minutes
```

Both firmware tests compile `src/` for the PC with zig (`pip install ziglang`, already installed on the owner's machine; any `g++` / `clang++` on PATH also works) and stub the ESP32 headers. **Run them after any change to `src/`.**

- `test_firmware_nav.py` drives the real `Navigator` with an ideal robot: commands are carried out exactly, sensors report the true maze (optionally inventing a wall now and then), and every 2 mm of path is checked for clearance. Random mazes are reproducible from their seed (`--random 1 --seed N`).
- `test_firmware_drive.py` closes the loop: a physics model plays the motors, encoders, BNO055 (answering the driver's I2C reads), IR receivers (ray-cast against the walls with inverse-square reflection) and battery, and the firmware runs its 500 Hz loop on top, from IR calibration through search to speed runs at all three tiers. It catches wrong signs, units, and state-machine mistakes. It does **not** validate tuning: motor lag, friction, sensor positions and reflection strength are guesses written at the top of its harness.

**On the owner's PC, run both firmware tests under WSL**, which already has `g++` and Python (Arch):

```
wsl.exe -d Arch -- bash -lc 'cd /mnt/c/Users/jstnp/Micromouse-2027 && python3 sim/tests/test_firmware_nav.py && python3 sim/tests/test_firmware_drive.py'
```

**Windows may refuse to run the compiled test program** ("An Application Control policy has blocked this file", Smart App Control). It comes and goes between builds. Do not try to work around it on the Windows side; WSL is the route the owner approved. The firmware builds are unaffected.

The other Python simulators in `sim/` are a separate implementation of the navigation ideas; they do **not** run the firmware, so passing them says nothing about `src/`.

## Bugs the simulated robot found (all fixed)

Kept here because each is a kind of mistake to watch for in any new code.

1. **Front-wall calibration.** With an open corridor ahead, a wall one cell further away read just bright enough to be taken as "the wall in front", so afterwards every distant wall looked close and the robot believed it was boxed in. Now a front wall only counts for calibration if it is much brighter than the side walls.
2. **Heading reference after squaring up.** After squaring against a wall the heading was reset to zero whichever way the robot faced, and the controller's memory of the previous IMU reading was not updated, so the next turn was off by a multiple of 90° and tripped the lost-heading stop. Now it snaps to the nearest grid direction (`snapHeadingToGrid`).
3. **Moves finished on time, not on arrival.** A move ending in a stop was declared done when its planned motion ran out, leaving the robot a few millimetres short each time; the shortfall was never recovered and built up to 40-60 mm across a maze. Now the controller holds the target until the robot is there (`SETTLE_*`), and every move starts from where the last one aimed to end.
4. **Old "open" votes beating a wall in plain view.** The robot could drive at a wall its front sensors were seeing, because an earlier reading had voted that gap open and the tally stood at zero. A wall seen directly ahead from the cell centre is now always believed.
5. **IR calibration needing walls it cannot count on.** The 45° sensors look slightly into the next cell, so calibration failed whenever that cell had a side gap. That side is now estimated from the other.
6. **Heading and distance not reset between runs** (found by reading, not by the test). After a run the robot is turned by hand to face into the maze; the controller would have treated that as an error to undo. `prepareForNewRun()` now resets tracking at every launch.

## What to do next (needs the real robot)

Bring-up order, each step proving the layer below the next:

1. Flash `main`, open the serial monitor, send `health`. Motoron and IMU should say OK.
2. Send `stream` to switch on the `[TEL]` line: wave a hand at each IR sensor and confirm L90 / L45 / FL / FR / R45 / R90 respond in the right position. Send `perf` and confirm the loop time is under 1200 µs with no overruns (the IR timing is the main unknown).
3. Push the robot by hand and send `enc`: forward should count up on both sides. Fix with `encinv` / `motorinv`, then make it permanent in `config.h`. (Done by the owner on 2026-10-07: both encoders counted backwards, so `INVERT_LEFT_ENCODER` and `INVERT_RIGHT_ENCODER` are now `true`. The owner has since also set `INVERT_LEFT_MOTOR` and `INVERT_RIGHT_MOTOR` to `true` in `config.h` themselves, so both motors ran backwards too. Leave all four as the owner set them. The simulated robot in `test_firmware_drive.py` reads these four flags and wires itself the same way.) The owner also found the **left encoder gives many more ticks than the right for the same wheel rotation**; each side now has its own `ENCODER_TICKS_PER_REV_LEFT` / `_RIGHT` in `config.h`, both still at the datasheet 840 until the owner measures them (`resetenc`, turn one wheel 10 turns by hand, `enc`, divide by 10). Until those are measured, every distance and turn is wrong. If the ratio is not a clean one, suspect a flaky encoder channel rather than a different motor.
4. Turn the robot by hand: heading should increase turning left.
5. Wave once and confirm the count blink and countdown. Tune `GESTURE_MIN_RISE` if waves are missed or invented.
6. Flash `test3x3`, put the robot in the corner cell of the 3x3 practice maze facing along a wall, calibrate IR (5 waves), then try a search (1 wave) with a hand ready to lift it. Do the first runs with `ENABLE_SEARCH_LOOKAHEAD 0` (stop-and-turn search), then switch it on once straights and on-the-spot turns are trustworthy.
7. Measure with a ruler how far the two 90° side sensors sit ahead of the wheel axle and set `SIDE_SENSOR_AHEAD_MM` in `config.h` (40 is a guess). The post-edge distance correction depends on it.
8. With the `calibration` build, measure the wheels: `drivecal` then `drivecal result <measured mm>` gives the true `WHEEL_DIAMETER_MM`; `turncal` gives the true `WHEEL_BASE_MM`. Put both in `config.h`. Every distance and the turn feedforward depend on them.
9. After each run send `log` and plot it. Tune in this order: `FF_KV` until `speed_mm_s` matches `target_mm_s` at cruise with `forward_pct` mostly explained by feedforward; `FF_KS` from the effort needed at very low speed; `FF_KA` from the extra effort during acceleration; then the PID gains in the `MotionController` constructor (straights first, then turns). `POST_EDGE_PHASE_MM` comes from where the L90 / R90 columns step relative to distance travelled.
10. Only then try speed runs. They start at tier 1 (60 %) by themselves; lower `SPEED_TIER_1_SCALE` if even that is too fast.

Known weak spots to check first if something misbehaves: at full speed (tier 3) the simulated robot runs about 18 mm wide on diagonals, which is all the room there is; expect to need `FF_KA` and the heading PID tuned from run logs before tier 3 diagonals are safe, and until then tiers 1 and 2 are the ones to rely on; `Wire.setTimeOut(1)` may be too tight for BNO055 clock stretching (watch the IMU dropout counter); the feedforward constants are estimates from the motor's nominal no-load speed, and the velocity PID gains were chosen before feedforward existed, so expect to lower them if speed overshoots; the search look-ahead windows (`SEARCH_LOOKAHEAD_START_MM` / `_END_MM`, `SEARCH_FRONT_SAMPLE_*`) assume the sensors sit roughly 40 mm ahead of the wheel axle, and if they are wrong the robot simply falls back to stop-and-turn rather than misbehaving; the diagonal centring gain is a guess; the V turn is the tightest move the robot makes (about 480°/s at full speed-run speed), so if it slides there lower `V90_SPEED_RATIO`; the shortest diagonal straight between two V turns is only 7 mm, which leaves the controller almost no time to settle between them; the Dijkstra path costs were written for the old move set and may not pick the truly quickest path for the new one; the diagonal post guard threshold (`DIAG_GUARD_RATIO`) and `POST_EDGE_PHASE_MM` are calculated guesses.

## Ideas considered and deliberately not built yet

Each of these needs measurements from the real robot first; building them blind would add untestable code.

- **A dedicated 135° turn** (straight to diagonal in one curve). Today that shape is driven as a 45° curve, 31 mm of diagonal, and a V turn, which is continuous but not the quickest line.
- **Retuning the Dijkstra costs to the planner's real move times**, so the path chosen is the quickest to drive rather than the shortest in cells. Checkable on the PC with `test_firmware_nav.py`.
- **Front-wall distance correction** before stopping, and full x/y position tracking with wall-based correction. Needs IR readings converted to real distances per sensor (a two-point calibration, not the single centred point available now).
- **Whole-path speed planning** with a grip limit in curves. Needs measured tyre grip and motor limits from run logs.
- **Raw gyro at 1 kHz** instead of the BNO055's 100 Hz fused heading. Only matters above roughly 1 m/s, and would mean reconfiguring the IMU out of fusion mode.
- **Kalman filter.** Not worth it here: one gyro, two encoders, and exact corrections from walls are served well by the simpler mechanisms above.
- **Derivative-on-measurement in the PIDs.** Rejected: these loops track moving targets, where derivative on error is the wanted behaviour.

## Working notes for AI assistants

- When scripting edits to C++ through a shell heredoc on this machine, backslash escapes get mangled (`\n` becomes a real newline). Use the editor tools, or write the script to a file first.
- Do not commit or push unless asked. Feature branches are named `user/<name>/<bot>/<topic>` off `release/<bot>`.
- After any firmware change: build `main`, `competition`, and `calibration` at minimum.
