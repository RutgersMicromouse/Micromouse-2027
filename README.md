# Micromouse-2027 — Antigravitieee

Firmware, simulator, and tools for the **Antigravitieee** micromouse (ESP32-S3, Pololu Motoron M2T256, N20 gearmotors with magnetic encoders, Bosch BNO055 IMU, 6 IR wall sensors).

Each bot in this repository lives on its own branch, named `release/<bot name>` (this one is `release/antigravitieee`).

## Where things are

| Path | What is in it |
| :--- | :--- |
| [`src/`](src/) | **The firmware: everything that goes on the robot, and nothing else.** |
| [`src/config.h`](src/config.h) | Every pin, dimension, speed limit, and threshold. Start here when something on the robot changes. |
| [`src/main.cpp`](src/main.cpp) | Start-up and the four tasks (500 Hz motion on core 1; navigation, operator console and telemetry on core 0). |
| [`src/hardware/`](src/hardware/) | Drivers, one folder each: `encoders/`, `motors/`, `ir_sensors/`, `imu/`. Every folder holds that part's `.h` and `.cpp`. |
| [`src/control/`](src/control/) | `pid/`, `profile/` (speed ramps), `motion_controller/` (starting a move, and `motion_update.cpp`, the 500 Hz loop: straights, smooth curves, diagonals). |
| [`src/navigation/`](src/navigation/) | `maze/` (the wall map), `floodfill/` (search), `dijkstra/` (speed-run path), `navigator/` (with `navigator_search.cpp` and `navigator_speedrun.cpp`). |
| [`src/ui/`](src/ui/) | `status_led/`, `actions/`, `gestures/` (hand waves), `console/` (debug commands). |
| [`src/wireless/`](src/wireless/) | `ble_debug/`, `wifi_ota/` (with `web_pages.h`, the phone app), `debug_log/`. For debugging only. |
| [`bench/`](bench/) | Bench tests (one file per peripheral, proven on the real robot) and the calibration commands. |
| [`sim/`](sim/) | Desktop simulators, mazes, and Python tests (`sim/run_curve_sim.bat` launches the GUI). |
| [`tools/`](tools/) | Browser dashboard for Bluetooth telemetry, the camera feed server (`camera_feeds/`), and the dyno test-stand firmware. |
| [`docs/`](docs/) | [Operator manual](docs/INSTRUCTIONS.md) (controls, flashing, troubleshooting, engineering constraints) and the schematic. |
| [`AGENTS.md`](AGENTS.md) | Full project context for AI assistants (and a good summary for humans). |

## Using the robot

There are no buttons. Power it on, then **wave a hand in front of the front sensors**:

| Waves | Action | LED |
| :--- | :--- | :--- |
| 1 | Search run | Green |
| 2 | Speed run, hybrid | Yellow |
| 3 | Speed run, diagonals | Cyan |
| 4 | Speed run, curves | Magenta |
| 5 | Calibrate IR sensors | Yellow |
| 6 | Clear the saved maze | Blue |

Always start with the robot in the start cell facing into the maze. The LED blinks the count back, then blinks rapidly for 2 seconds before moving; cover the sensors to cancel. To stop a run, lift the robot and turn it sideways. The map is kept between searches, and speed runs get faster each time one succeeds. Details are in the [operator manual](docs/INSTRUCTIONS.md#4-hand-wave-controls--led-field-guide).

## Build and flash

Everything goes through PlatformIO. `pio run` on its own builds only `main`.

| Command | What it does |
| :--- | :--- |
| `pio run -e competition -t upload` | **Competition firmware: radios off, hand-wave control only** |
| `pio run -e main -t upload` | Same firmware plus the Bluetooth / Wi-Fi debug console |
| `pio run -e test3x3 -t upload` | `main` for the 3x3 practice maze (start in a corner cell, goal = the middle cell) |
| `pio run -e ota -t upload` | `main`, flashed over the robot's Wi-Fi hotspot |
| `pio run -e calibration -t upload` | `main` plus motor and dyno calibration commands |
| `pio run -e motor_test -t upload` | Bench test for one peripheral (also `ir_test`, `imu_test`, `battery_test`, `ble_test`, `wifi_test`) |
| `pio device monitor` | Serial console at 115200 baud |

Before flashing after a change:

```powershell
pio run -e main -e competition -e calibration
python sim/tests/test_firmware_nav.py        # the real navigation code on thousands of mazes (pip install ziglang)
python sim/tests/test_firmware_drive.py      # the real firmware driving a simulated robot
python sim/tests/test_desktop_suite.py
python sim/verify_headless.py --mazes 1 --trials 1
```

## Hardware notes that are not on the schematic

The schematic in `docs/` is slightly behind the robot:

- The motors are **N20 12 V gearmotors** (30:1, 7-pulse magnetic encoders), not the Faulhaber 1524 drawn on the schematic. Left and right encoder wiring is unchanged.
- The **IR channel numbers on the schematic do not match the physical sensor positions**. The pin map in `src/config.h` is the one verified on the robot with `ir_test`:

  | Channel | Position | Emitter GPIO | Receiver GPIO |
  | :--- | :--- | :--- | :--- |
  | CH1 | Left 90° | 15 | 3 |
  | CH2 | Front-left 45° | 16 | 8 |
  | CH3 | Front-left centre | 17 | 1 |
  | CH4 | Front-right centre | 14 | 10 |
  | CH5 | Front-right 45° | 18 | 2 |
  | CH6 | Right 90° | 19 | 9 |

- The external RGB LED on the PCB is not used; the status LED is the DevKit's on-board NeoPixel (GPIO 48).

## Branching conventions

Create your working branch off the bot you are working on. For example, if you are working on ratatouieee, branch off `release/ratatouieee`.

```sh
user/<user-name>/<bot-name>/problem

# example: arfelix is starting the implementation of pid rotation
user/arfelix/ratatouieee/pid-rotation-implementation
```

```sh
git checkout release/ratatouieee
git checkout -b user/arfelix/ratatouieee/pid-rotation-implementation
```

Once your changes are made and tested, open a **pull request**. Instructions are in the [git workshop](https://github.com/RutgersMicromouse/git-workshop).
