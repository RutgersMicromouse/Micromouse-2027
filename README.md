# Ratatouieee - Rutgers Micromouse 2026-2027

Firmware repository for the **Ratatouieee** micromouse bot, engineered for the Rutgers Micromouse competition.

---

## 1. Hardware Architecture

| Subsystem | Component | Interface | Pin / Address | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **Microcontroller** | **Teensy 4.0** | ARM Cortex-M7 @ 600MHz | — | 1MB RAM, 2MB Flash, Hardware FPU |
| **Motor Driver** | **Pololu Motoron M2T256** | I2C (`Wire`) | Addr `0x10` (16) | Dual-channel DC driver, 12V motor supply |
| **Left Motor (LMOT)** | Pololu Micro Metal Gearmotor | Motoron Ch 1 | M1A / M1B | Channel 1 on Motoron |
| **Right Motor (RMOT)**| Pololu Micro Metal Gearmotor | Motoron Ch 2 | M2A / M2B | Channel 2 on Motoron (direction inverted) |
| **Left Encoder** | Magnetic Quadrature | Pin Interrupts | Pin 2 (`LMOTChanA`), Pin 3 (`LMOTChanB`) | High-speed quadrature counting via PaulStoffregen/Encoder |
| **Right Encoder** | Magnetic Quadrature | Pin Interrupts | Pin 4 (`RMOTChanA`), Pin 5 (`RMOTChanB`) | Quadrature counting |
| **IMU** | **Pololu MinIMU-9 v5** | I2C (`Wire`) | Addr `0x6B` (LSM6DS33) | 3-axis gyro (±1000 dps) + 3-axis accel (±2g) |
| **Front IR Sensor** | Analog Distance (`FIR`) | ADC (Analog) | Pin 17 (`A3`) | Front wall detection & squaring |
| **Front-left side IR Sensor**| Analog Distance | ADC (Analog) | Pin 16 (`A2`) | Parallel left-side wall sensing |
| **Rear-left side IR Sensor**| Analog Distance | ADC (Analog) | Pin 15 (`A1`) | Parallel left-side wall and heading alignment |
| **Front-right side IR Sensor**| Analog Distance | ADC (Analog) | Pin 14 (`A0`) | Parallel right-side wall sensing |
| **Rear-right side IR Sensor**| Analog Distance | ADC (Analog) | Pin 20 (`A6`) | Parallel right-side wall and heading alignment |
| **Motor Battery Sense** | Resistor Divider (100k/33k) | ADC (Analog) | Pin 21 (`A7`) | Monitors only the motor battery; ratio: 4.0303x, critical threshold: 6.4V |
| **Status LED** | On-Board LED | GPIO Output | Pin 13 (`LED_BUILTIN`) | Startup diagnostics, mode select, blink feedback |

---

## 2. Directory Layout

```
Micromouse-2027/
    ├── platformio.ini               # PlatformIO Teensy 4.0 and 3x3 debug build configurations
├── include/
│   ├── config.h                 # Global hardware pins, dimensions, rates, and thresholds
│   ├── maze_constants.h         # Maze dimensions, bitmasks, and direction utilities
│   └── types.h                  # Common structs (Coordinate, Pose, SensorReadings, Profiles)
├── src/
│   ├── main.cpp                 # Boot sequence, gesture mode selector, serial console
│   ├── hardware/
│   │   ├── motors.h / .cpp       # Pololu Motoron M2T256 I2C motor driver
│   │   ├── encoders.h / .cpp     # PaulStoffregen/Encoder high-resolution odometry
│   │   ├── imu.h / .cpp          # MinIMU-9 v5 (LSM6DS33) gyro heading & bias ZUPT
│   │   ├── ir_sensors.h / .cpp   # 5-channel analog distance sensors & wall centering
│   │   └── battery.h / .cpp      # Motor battery voltage monitor & safety cutoff
│   ├── control/
│   │   ├── pid.h / .cpp          # Discrete PID with derivative filter & anti-windup
│   │   ├── profile.h / .cpp      # Real-time trapezoidal / S-curve motion profiling
│   │   └── motion_controller.h/.cpp # 500 Hz closed-loop motion controller
│   └── navigation/
│       ├── maze.h / .cpp         # Configurable-size bitpacked maze map
│       ├── floodfill.h / .cpp    # Wavefront floodfill solver (Center & Start goals)
│       ├── optimizer.h / .cpp    # High-speed path generator (multi-cell straight sprints)
│       └── navigator.h / .cpp    # High-level state machine (explore, map, return, speed run)
└── README.md
```

---

## 3. Control & Navigation Engine

1. **500 Hz Synchronous Loop**:
   - Updates encoders, gyroscope yaw integration, analog IR distance sensors, and motor battery voltage.
   - Closed-loop linear velocity PID tracking desired velocity from the trapezoidal motion profiler.
   - Angular heading PID fusing target heading, IMU gyro rate, and IR wall-centering error.
   - The default PlatformIO environment is `teensy40`, configured for the regular 16x16 maze build. Use `pio run -e teensy40-debug-3x3` for manual 3x3 maze mapping with motors and IMU disabled. The motor battery can remain off in that debug environment; power the Teensy and IR sensors from USB/component power.
2. **Autonomous Maze Exploration**:
   - Wavefront BFS floodfill dynamically updates distances to the center cells for the configured maze size.
   - Preferential straight-line movement tie-breaker minimizes turn overhead.
   - In-cell front wall squaring nulls longitudinal and angular odometry drift.
3. **Optimized Speed Run**:
   - Compresses known corridor paths into continuous multi-cell straightaways.
   - Reaches speeds up to 700–1000 mm/s.

---

## 4. How to Operate

### Gesture-Based Start (Using Front IR Sensor):
Hold your hand in front of the front sensor at boot:
- **Hold for 1 sec**: Mode 1 - Explore to Center
- **Hold for 2 sec**: Mode 2 - Full Autonomous Run (Explore -> Return -> Speed Run)
- **Hold for 3 sec**: Mode 3 - In-Cell Sensor Auto-Calibration

### Serial Console Commands (115200 Baud):
- `e`: Explore to Center (Floodfill)
- `r`: Return to Start `(0, 0)`
- `f`: Execute High-Speed Speed Run
- `a`: Full Autonomous Run
- `c`: In-Cell Sensor Auto-Calibration
- `d`: Toggle Real-Time Diagnostic Telemetry Stream
- `m`: Print ASCII Maze Map
- `t`: Test 90° In-Place Turn
- `w`: Test 1-Cell Forward Move (180 mm)
- `s`: Emergency Stop

### Manual Maze Debug Build (`teensy40-debug-3x3`):
Open the Serial Monitor at 115200 baud and set its line ending to Newline or Both NL & CR. The debug build does not initialize or use the IMU, and waits for a command before each scan. Physically move the robot first, then enter `s` to record one straight cell, `r` for 90° clockwise, `l` for 90° counterclockwise, or `ll` for 180°, and press Enter. The turn commands update the logical heading while keeping the current cell; `s` advances the logical position by one cell. These commands apply only to the manual debug build; in the regular build, `r` still means Return to Start.
