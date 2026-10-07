# 🐭 Antigravitieee Micromouse — Operator's Instruction Manual

This manual covers everything you need to build, flash, run, calibrate, and debug the **Antigravitieee** Micromouse robot completely wirelessly.

---

## 📑 Quick Navigation
1. [First-Time USB Setup](#1-first-time-setup-usb-cable-only-once)
2. [Wireless Firmware Flashing (Over-The-Air)](#2-wireless-firmware-flashing-over-the-air-ota)
3. [Mobile Phone Telemetry & Debug Console (Bluetooth BLE)](#3-mobile-phone-telemetry--debug-console-bluetooth-ble)
4. [Hand-Wave Controls & LED Field Guide](#4-hand-wave-controls--led-field-guide)
5. [Laptop Wireless Serial Monitor (Wi-Fi Telnet)](#5-laptop-wireless-serial-monitor-wi-fi-telnet)
6. [Git Version Control Cheat Sheet](#6-git-version-control-cheat-sheet)
7. [Pre-Flight Safety & Troubleshooting](#7-pre-flight-safety--troubleshooting)
8. [Engineering Constraints](#8-engineering-constraints)

---

## 1. First-Time Setup (USB Cable — Only Once)

Before using wireless flashing, the ESP32-S3 needs its initial bootloader and 8 MB partition layout flashed via USB-C:

1. Connect the ESP32-S3 DevKitC-1 to your computer using a USB-C data cable.
2. In PlatformIO terminal (or VS Code terminal), execute:
   ```powershell
   pio run -e main -t upload
   ```
3. Open the serial monitor to verify bootup:
   ```powershell
   pio device monitor -b 115200
   ```
4. You should see:
   ```text
   ==================================================
     ANTIGRAVITIEEE MICROMOUSE - REVISION 1.0       
   ==================================================
   [INIT] Initializing N20 PCNT Hardware Encoders (30:1, 840 CPR)...
   [INIT] Initializing Pololu Motoron M2T256 Motor Driver...
   [INIT] Initializing 6-Channel SFH4545/TEFT4300 IR System...
   [INIT] Initializing Bosch BNO055 IMU...
   [INIT] Starting Nordic UART Bluetooth Low Energy Service...
   [WIFI] Hotspot Started: SSID 'Antigrav-Mouse' | IP: 192.168.4.1
   [READY] Antigravitieee is Ready!
   ```
5. **Unplug the USB cable.** From this point forward, you never need the cable again!

---

## 2. Wireless Firmware Flashing (Over-The-Air / OTA)

Whenever you edit code, you can flash the robot over the air while it sits on the floor or in the maze.

### Method A: 1-Command PlatformIO Upload (Recommended)
1. Turn on the robot battery switch.
2. On your laptop, connect to the robot's Wi-Fi:
   * **SSID**: `Antigrav-Mouse`
   * **Password**: `micromouse`
3. Run:
   ```powershell
   pio run -e ota -t upload
   ```
4. PlatformIO compiles the code, transmits the binary wirelessly across Wi-Fi directly into the ESP32-S3 alternate flash partition (`ota_1`), checks MD5 verification, and reboots in ~8 seconds.

### Method B: Web Browser Upload (Phone or Laptop)
1. Connect your phone or laptop to `Antigrav-Mouse` Wi-Fi.
2. Open any web browser and go to:
   ```text
   http://192.168.4.1/update
   ```
3. Click **Choose File**, select `.pio\build\main\firmware.bin`, and click **Upload & Flash Firmware**.
4. The web page will show progress, flash the chip, and the robot will reboot.

---

## 3. Mobile Phone Telemetry & Debug Console (Bluetooth BLE)

You can view real-time diagnostics from your phone without touching the bot.

### Step 1: Install a BLE Terminal App
* **Android / iOS**: Install **Serial Bluetooth Terminal** (by Kai Morich) or **nRF Connect**.

### Step 2: Connect
1. Open Bluetooth on your phone and open **Serial Bluetooth Terminal**.
2. Go to **Devices** $\rightarrow$ **Bluetooth LE** $\rightarrow$ Scan.
3. Select **`Antigrav-Mouse`** and tap **Connect**.

### Step 3: Live Telemetry Stream
Once connected, the bot streams live sensor and motion data at 5 Hz:
```text
V:3.72V | Spd: 420 | Hdg:  90.0 | IR: 28, 45, 842, 856, 42, 31 | W:[.F.]
```
* `V`: Computer battery voltage (separate 1S cell; nominal 3.7V, fully charged 4.2V).
* `Spd`: Instantaneous linear velocity in mm/s.
* `Hdg`: Bosch BNO055 fusion heading in degrees ($0.0^\circ - 360.0^\circ$).
* `IR`: Readings for all 6 optical channels: `[L90, L45, FL, FR, R45, R90]`.
* `W`: Wall detection flags: `[L]` (Left), `[F]` (Front), `[R]` (Right). `.` indicates open corridor.

### Step 4: Debug Console Commands
The console is for watching and adjusting the robot on the bench. **It cannot start a run, select a mode, calibrate the IR sensors, or clear the maze: those are hand-wave only (section 4).** Wireless is not allowed at competition, so flash the `competition` build there (`pio run -e competition -t upload`), which leaves the radios off.

Type any of the following commands into the terminal:

| Command | Effect | LED Indication |
| :--- | :--- | :--- |
| `stop` or `estop` | **Emergency stop** — instantly brakes motors and aborts run | 🔴 3x Red Flash |
| `motorcal` | **Calibration firmware only** (`pio run -e calibration -t upload`). Automated 3-point motor speed calibration and trim balancing | 🟡 Yellow $\rightarrow$ 🟢 Flash (Pass) / 🔴 Flash (Fail) |
| `motorrpm [duty]` | **Calibration firmware only.** Runs tachometer benchmark at specified duty (default 50%) for 4s | 🌐 Solid Cyan |
| `motortrim [l r]` | Sets or queries motor trim multipliers saved in NVS | — |
| `enc` | Prints live raw encoder ticks, mm traveled, speed, and inversion status | — |
| `motorinv <l:0/1> <r:0/1>` | Sets motor direction inversion at runtime | — |
| `encinv <l:0/1> <r:0/1>` | Sets encoder count direction inversion at runtime | — |
| `status` | Reports battery voltage, active mode, and navigation state | Current profile color |
| `health` | Reports motor driver, IMU, and encoder fault status plus recovery counters (see section 7) | — |
| `resetenc` | Zeroes the encoder distance counters (idle only; used by the web dashboard) | — |
| `resetheading` | Makes the current facing direction 0° (idle only; used by the web dashboard) | — |
| `perf` | Reports real-time 500Hz loop execution latency, peak time, overruns, and stack headroom | — |
| `log` | Prints the last 30 s of driving as CSV (target vs. actual speed, heading error, motor effort, IR readings) for plotting and tuning. USB or Telnet only. `log clear` empties it | — |
| `help` | Lists available commands | — |

---

## 4. Hand-Wave Controls & LED Field Guide

The robot has no buttons or switches. Power it on, wait for the LED to turn solid (about 3 seconds), then **wave a hand in front of the two front sensors** and pause. The number of waves tells it what to do.

| Waves | Action | LED |
| :--- | :--- | :--- |
| **1** | Search run (explore the maze, then return to start) | 🟢 Green |
| **2** | Speed run: Hybrid Auto-Optimizer | 🟡 Yellow |
| **3** | Speed run: Pure Diagonals | 🌐 Cyan |
| **4** | Speed run: Pure Continuous Curves | 🟣 Magenta |
| **5** | In-cell IR calibration (robot centred in a cell between two side walls) | 🟡 Yellow while sampling, then 🟢 3x Green = OK / 🔴 3x Red = failed |
| **6** | Erase the saved maze | 🔵 4x Blue flash |

### What you will see
1. **Each wave** it counts: one short ⚪ white blink.
2. **About 1.5 seconds after your last wave**: it blinks the count back to you in the action's color.
3. **Before a run or calibration**: 2 seconds of rapid blinking. Get your hand out of the way.
4. **The run starts** and the LED stays solid in the mode's color.

### Changing your mind
* **During the count**: hold your hand in front of the sensors for more than 1.5 seconds. The count is thrown away.
* **During the rapid blinking**: cover the front sensors. The LED flashes 🔴 red twice and nothing starts.

### Stopping a run
A hand in front of the sensors looks like a wall to a moving robot, so waves are ignored during a run. To stop it, **lift the robot and turn it sideways**: once it is pointing more than 60° away from where it is trying to go, it brakes and abandons the run within a tenth of a second. It also stops by itself if it drives into something and the wheels stop turning. When debugging with a phone or laptop connected, `stop` on the console works too.

### Tips
* A wave is the hand arriving **and leaving** within 1.5 seconds. Bring it within a few centimetres of the front of the robot.
* Waves are ignored for the first 2 seconds after power-on and after each run, while the robot learns what the sensors see at rest.
* If waves are missed or counted when nobody waved, adjust `GESTURE_MIN_RISE` in `src/config.h`. Watch the `FL` / `FR` numbers in the telemetry stream with and without your hand to pick a value.
* Set `ENABLE_GESTURE_UI` to `0` in `src/config.h` to turn waves off and use text commands only.

### LED colors at a glance
| LED | Meaning |
| :--- | :--- |
| 🔵 Solid blue | Booting |
| 🟢 🟡 🌐 🟣 Solid | Idle or running, in that mode |
| ⚪ White blink | Wave counted |
| 🔴 5x Red flash at boot | Motor driver not responding |
| 🟡 3x Yellow flash at boot | IMU not found (running on encoder heading) |
| 🔴 5x Red flash during a run | Run aborted by stall / encoder protection |
| 🔴 Solid red | Low battery, motors disabled |

---

## 5. Laptop Wireless Serial Monitor (Wi-Fi Telnet)

To view serial debug logs on your laptop without any USB wire:

1. Connect your laptop to `Antigrav-Mouse` Wi-Fi.
2. In your terminal, run:
   ```powershell
   pio device monitor --port socket://192.168.4.1:23
   ```
   *(Or using any telnet client: `telnet 192.168.4.1 23`)*
3. You will receive the exact same 115200 baud console stream and can send text commands directly.

---

## 6. Git Version Control Cheat Sheet

All modifications are recorded with clean commit history:

```powershell
# Check current branch and modified files:
git status

# View commit history:
git log --oneline

# Stage changes:
git add .

# Commit with a meaningful message:
git commit -m "feat: tuned turn PID and wall thresholds"

# (Optional) Push to your GitHub repository:
git remote add origin https://github.com/<your-username>/antigravitieee.git
git push -u origin master
```

---

## 7. Pre-Flight Safety & Troubleshooting

### Battery Safety Checklist
* **Computer Battery Chemistry**: Separate 1S Li-ion/LiPo cell (3.7V nominal); the schematic's 10k/10k divider feeds its voltage to GPIO12 (`VSenseCom`).
* **Low Battery Alert**: If the computer battery remains below **3.3V for 200 ms**, the robot automatically cuts motor power, engages emergency stop, and turns the LED **Solid Red**. Recharge immediately.
* The motor battery is a separate supply and is not monitored by this firmware cutoff. Check computer-battery voltage via BLE `status` or the telemetry stream.

### Sensor Calibration Check
Before a run in a new arena:
1. Place the robot in the starting cell centered between the walls.
2. Wave 5 times in front of the front sensors.
3. Ensure you see **3x Green flashes** indicating baseline calibration is stored in Flash NVS.

### Built-In Fault Recovery
The firmware keeps running through the most likely hardware hiccups. Send `health` to see whether any of them have happened:

| What goes wrong | What the robot does | What you see |
| :--- | :--- | :--- |
| Motor driver does not answer at power-on | Retries 3 times, then keeps retrying in the background | 🔴 5x Red flash at boot, `Motoron=NOT RESPONDING` |
| Motor driver reboots mid-run (12V rail sag) | Detects it within 200 ms, reconfigures it, and resumes driving | `resets recovered` counter goes up |
| IMU missing at power-on | Heading runs on wheel encoders instead | 🟡 3x Yellow flash at boot, `IMU=MISSING` |
| IMU glitches, drops off the bus, or resets | Bad readings are rejected, heading continues on wheel encoders, and the IMU is picked up again when it recovers | `dropouts` counter goes up |
| One encoder stops counting or a wheel jams | Emergency stop and the run is aborted | 🔴 5x Red flash, `Encoder faults` counter goes up |
| Robot ends up pointing more than 60° off course (crash, or lifted and turned) | Emergency stop and the run is aborted | 🔴 5x Red flash |
| Motors stall against a wall | Emergency stop and the run is aborted | 🔴 5x Red flash |

A counter that keeps climbing points at a loose connector or a weak battery, so check wiring before the next run.

### Tuning From a Run Log
1. Do a short run, then connect over USB or Telnet and send `log`.
2. Copy the CSV into a spreadsheet and plot `target_mm_s` against `speed_mm_s`, and `heading_err_deg`.
3. If the robot is consistently slower than the target at steady speed, raise `FF_KV` in `src/config.h`; if faster, lower it. If it lags only while accelerating, raise `FF_KA`. If it hesitates when starting from rest, raise `FF_KS`.
4. Once speed follows the target, tune the PID gains to remove what error is left.

### Emergency Stop Readiness
* There is no stop button. Be ready to lift the robot and turn it sideways, which stops it within a tenth of a second.

---

## 8. Engineering Constraints

The timing, safety, and geometry rules the firmware is built around. Any change to the code must keep these true.

### 8.1 Real-Time Timing Constraints (Core 1)

| Parameter | Constraint | Threshold / Target | Violation Action |
| :--- | :--- | :--- | :--- |
| **Control Loop Frequency** | Strictly periodic | $500\text{ Hz}\ (\pm 0.5\%)$ | FreeRTOS task delay warning |
| **Control Loop Period ($T$)** | Fixed | $2000\,\mu\text{s}$ | — |
| **Max Execution Latency** | Must not exceed | $< 1200\,\mu\text{s}\ (60\%\text{ budget})$ | Alert via telemetry & `perf` command |
| **Loop Overrun Tolerance** | Hard deadline | $0\text{ overruns in normal run}$ | Increment `timing_stats.loop_overruns` |
| **I2C Fast Mode Speed** | Clock rate | $400\text{ kHz}$ | Hardware bus lockup check |
| **IMU Polling Rate** | Dedicated interval | $100\text{ Hz}$ (every 5th tick) | Prevents I2C bus congestion |
| **IR Sampling** | One emitter group per tick | Each channel at $250\text{ Hz}$, $300\,\mu\text{s}$ settle | Keeps IR cost to one settle delay per tick |

### 8.2 Memory & Concurrency Constraints

* **Zero Dynamic Heap Allocation in Core 1**:
  * No `malloc()`, `free()`, `new`, `delete`, or dynamic `std::vector` / `String` mutations inside `motionControlTask`. All buffers must be pre-allocated statically or on the stack.
* **Stack High-Water Margin**:
  * FreeRTOS tasks must maintain at least **512 words ($2\text{ KB}$)** of remaining stack headroom under maximum load (`uxTaskGetStackHighWaterMark`).
* **Inter-Task Communication**:
  * Core 0 and Core 1 communication is strictly mediated via non-blocking queues (`g_motion_cmd_queue`) and mutex-guarded telemetry snapshots (`g_telemetry_mutex`).

### 8.3 Electrical & Failsafe Constraints

* **Computer Battery Voltage Cutoff**:
  * **Chemistry**: Separate 1S Li-ion/LiPo cell ($3.7\text{ V}$ nominal).
  * **Cutoff Voltage**: $3.30\text{ V}$.
  * GPIO12 (`VSenseCom`) measures this battery through the 10k/10k divider.
  * If voltage drops below $3.30\text{ V}$ for $>200\text{ ms}$, the robot cuts motor power, engages emergency stop, and turns the LED **Solid Red**.
  * The separate motor battery is not monitored by this firmware cutoff.
* **Motor Coil Stall Protection**:
  * If commanded velocity exceeds $80\text{ mm/s}$ while measured linear speed $< 15\text{ mm/s}$ at $>35\%$ effort for $>200\text{ ms}$ ($100\text{ ticks}$), the robot triggers immediate emergency stop to prevent motor burn-out.
* **Encoder Fault Protection**:
  * On a straight, if one wheel reads $< 5\text{ mm/s}$ while the other reads $> 80\text{ mm/s}$ for $>150\text{ ms}$ ($75\text{ ticks}$), the robot triggers immediate emergency stop and the navigator aborts the run.
* **Motor Driver Supervision**:
  * The Motoron's $250\text{ ms}$ command timeout stays enabled, so the motors stop on their own if the ESP32 hangs. Speed and brake commands are re-sent at least every $100\text{ ms}$ to keep it satisfied.
  * Motoron status flags are polled every $200\text{ ms}$. A driver reset or I2C fault is recovered by reconfiguring the driver without stopping the control loop.
* **Motor Supply Compensation**:
  * Motor effort is defined as a fraction of $12\text{ V}$. The Motoron's measured supply voltage rescales every command (limited to $0.8\times$–$1.3\times$) so behaviour does not change as the supply sags.
* **Heading Redundancy**:
  * BNO055 readings that step more than $30^\circ$ in $10\text{ ms}$ are rejected. After $100\text{ ms}$ without a valid reading, heading continues on differential encoder odometry until the IMU returns.
* **Lost-Heading Protection**:
  * If the heading is more than $60^\circ$ from the commanded heading for $>100\text{ ms}$ ($50\text{ ticks}$), the robot triggers immediate emergency stop and the navigator aborts the run. With no buttons on the robot, lifting it and turning it sideways is the operator's stop.
* **Operator Input**:
  * Runs, mode selection, IR calibration, and clearing the maze can be started by hand waves only. The debug console may report, adjust settings, and stop, but never start. The `competition` build compiles Bluetooth and Wi-Fi out entirely.
* **Emergency Stop Latency**:
  * Receiving `stop` wirelessly must brake motors within $\le 2\text{ ms}$.

### 8.4 Kinematic & Actuator Invariants

* **Motors**: N20 12V DC Metal Gearmotors (30:1 gear reduction ratio).
* **Encoders**: Magnetic quadrature encoders on rear shaft ($7\text{ pulses/channel/rev}$).
* **Total Quadrature CPR**: $7.0 \times 4 \times 30 = 840.0\text{ counts/wheel rev}$.
* **Wheel Diameter**: $24.0\text{ mm}$.
* **Linear Distance Resolution**: $\approx 0.0898\text{ mm/tick}$ ($11.14\text{ ticks/mm}$).
* **Search Kinematics Limits**:
  * Max Search Speed: $240\text{ mm/s}$
  * Max Search Acceleration: $1500\text{ mm/s}^2$
  * Max Search In-Place Turn: $360^\circ/\text{s}$ ($1800^\circ/\text{s}^2$)

* **Smooth Turn Geometry** (return and speed runs):
  * A smooth turn of angle $A$ over path length $L$ follows $\theta(u) = A\,(3u^2 - 2u^3)$ with $u = s/L$, so yaw rate is zero at both ends. Its end point is $L\,(F_{fwd}, F_{lat})$ with $F = 0.60514$ both ways for $90^\circ$, and $F_{fwd} = 0.88961$, $F_{lat} = 0.36849$ for $45^\circ$.
  * $90^\circ$ turn, cell edge to cell edge: $L = 90 / 0.60514 = 148.73\text{ mm}$. Stays $90\text{ mm}$ from every post centre.
  * $45^\circ$ turn onto a diagonal: $L = 70\text{ mm}$, begun $53.52\text{ mm}$ past the cell centre, joining the diagonal $36.48\text{ mm}$ beyond the cell-edge midpoint. The diagonal then passes $63.6\text{ mm}$ from the posts on both sides, the most the maze allows.
  * `CURVE_90_LENGTH_MM`, `CURVE_45_LENGTH_MM`, `DIAG_LEAD_MM`, and `DIAG_TRIM_MM` in `src/config.h` are one consistent set; changing one alone moves the robot off the grid.
* **Search Look-Ahead**:
  * A smooth turn during the search is taken only when two different sensors agree the side is open: the $45^\circ$ sensor sampled between $30$ and $80\text{ mm}$ after the previous cell centre, and the $90^\circ$ sensor at the cell edge. Any doubt falls back to stopping at the cell centre.
  * A cell that was curved through is marked explored only if its front wall was read clearly (wall or open) by the outer $45^\circ$ sensor mid-curve.
* **Diagonal Corrections**:
  * Centring between the two rows of posts and the post guard together may trim the heading by at most $8^\circ$.
* **Search Kinematics**:
  * Decisions are otherwise taken at cell centres. The robot enters a cell it has not seen at no more than $120\text{ mm/s}$ (`SEARCH_PROBE_SPEED_MM_S`), from which it can stop in under $7\text{ mm}$, and turns on the spot.

### 8.5 Verification & Test Gates

Before committing or flashing firmware to physical hardware:
1. **Compilation Gate**: The robot firmware and every bench test must compile with 0 errors:
   ```powershell
   pio run -e main -e competition -e ota -e calibration
   pio run -e motor_test -e ir_test -e imu_test -e battery_test -e ble_test -e wifi_test
   ```
2. **Algorithm Test Suite**: Desktop test suite must pass with 100% green assertions:
   ```powershell
   python sim/tests/test_desktop_suite.py
   ```
3. **Simulation Verification**: Full 10-maze championship tournament run must complete collision-free:
   ```powershell
   python sim/verify_headless.py
   ```
