# 🏎 Antigravitieee Autonomous Optical Dyno & Tuning Station

A standalone, automated hardware dyno and calibration test bench for the **Antigravitieee** Micromouse platform. 

The Dyno Station uses its own dedicated **ESP32**, dual **optical slot photogates**, and **wireless BLE** to measure ground-truth wheel RPM and transient angular phase lag, auto-tune the robot's motor deadbands, feedforward trims, velocity PID, and differential phase-lock loop, and permanently burn the calibrated parameters into the robot's NVS Flash.

---

## 1. Physical Hardware Setup

```
                     [ Standalone Dyno ESP32 ]
                     ▲           ▲           │
     Optical Gate L  │           │           │  Wireless BLE (NUS)
        (GPIO 18) ───┘           └─── (GPIO 19) Optical Gate R
                                             │
                                             ▼
  ┌─────────────────────────────────────────────────────────────┐
  │                        PEDESTAL STAND                       │
  │                                                             │
  │     [Left IR Gate]                       [Right IR Gate]    │
  │       (ITR9608)                            (ITR9608)        │
  │          │                                    │             │
  │    ┌─────▼─────┐                        ┌─────▼─────┐       │
  │    │ Slit Wheel│                        │ Slit Wheel│       │
  │    └─────┬─────┘                        └─────┬─────┘       │
  │          │                                    │             │
  │    ╔═════╧════════════════════════════════════╧═════╗       │
  │    ║             MICROMOUSE ROBOT BODY              ║       │
  │    ║             (Elevated, Wheels Free)            ║       │
  │    ╚════════════════════════════════════════════════╝       │
  └─────────────────────────────────────────────────────────────┘
```

### Components Required
1. **1x ESP32 Dev Board** (ESP32 DevKit V1, ESP32-S3, or ESP32-C3).
2. **2x Optical Slot Interrupters / Photogates** (e.g. **ITR9608**, **EE-SX670**, or standard 3-pin photo-interrupter modules).
3. **2x Slit Wheels**: 3D-printed 24 mm diameter wheels or press-fit disks with a single radial slit ($0.8\text{ mm}$ width).
4. **1x Elevated Pedestal Stand**: 3D printed stand to hold the robot chassis elevated so both wheels spin completely free in air.

### Wiring Diagram
| Dyno ESP32 Pin | Optical Sensor / Peripheral | Function |
| :--- | :--- | :--- |
| **GPIO 18** | Left Slot Interrupter Signal | Interrupt on pulse edge (`FALLING`) |
| **GPIO 19** | Right Slot Interrupter Signal | Interrupt on pulse edge (`FALLING`) |
| **3.3V / 5V** | Optical Sensors VCC | Sensor power |
| **GND** | Optical Sensors GND | Ground common |
| **GPIO 2** | Built-in LED | Illuminates solid when BLE linked to robot |

---

## 2. Automated 5-Stage Tuning Pipeline

When you run `start`, the Dyno Station executes a fully automated closed-loop optimization cycle:

```
  ┌────────────────────────────────────────────────────────┐
  │ 1. Dual-Wheel Stiction Sweep                           │
  │    Ramps duty cycle on each motor to find minimum      │
  │    starting breakaway voltage (V_deadband,L/R)         │
  └───────────────────────────┬────────────────────────────┘
                              ▼
  ┌────────────────────────────────────────────────────────┐
  │ 2. Multi-Point Kv RPM Linearization                    │
  │    Sweeps 25% to 95% duty, records optical RPMs,       │
  │    computes trim scaling ratio to equalize motors      │
  └───────────────────────────┬────────────────────────────┘
                              ▼
  ┌────────────────────────────────────────────────────────┐
  │ 3. Championship Sprint Acceleration / Decel Test       │
  │    Executes 500 mm/s @ 2600 mm/s² trapezoidal sprint,  │
  │    measures dynamic transient phase lag                │
  └───────────────────────────┬────────────────────────────┘
                              ▼
  ┌────────────────────────────────────────────────────────┐
  │ 4. Cross-Coupled Phase-Lock Loop (PLL) Tuning          │
  │    Iteratively tunes differential sync gain K_sync     │
  │    until slit pulse drift Δθ < 0.5° across full sprint │
  └───────────────────────────┬────────────────────────────┘
                              ▼
  ┌────────────────────────────────────────────────────────┐
  │ 5. Permanent NVS Flash Commit                          │
  │    Transfers calibrated trims, deadbands, and sync     │
  │    gains wirelessly into robot's Flash memory          │
  └────────────────────────────────────────────────────────┘
```

---

## 3. How to Build & Flash

### Flashing the Dyno ESP32
Open a terminal in the project directory:
```powershell
# For generic ESP32 DevKit:
pio run -d tools/dyno_station -e dyno_esp32 -t upload

# Or for ESP32-S3 DevKit:
pio run -d tools/dyno_station -e dyno_esp32s3 -t upload
```

---

## 4. Operating Instructions

1. **Power on the Micromouse Bot**: It starts advertising as `"Antigrav-Mouse"`.
2. **Mount the Robot**: Place the robot onto the pedestal stand with the slit wheels centered inside the IR photogates.
3. **Power on the Dyno ESP32** and open the Serial Monitor @ 115200 baud:
   ```powershell
   pio device monitor -d tools/dyno_station -b 115200
   ```
4. The Dyno station will automatically scan for `"Antigrav-Mouse"`, connect via BLE, and turn the LED **Solid ON**.
5. Type `start` into the Serial Monitor.
6. Watch the automated test cycle run. Once complete, the robot flashes green and saves the calibration directly to its internal NVS Flash.
7. Unmount the robot — it is now calibrated and ready to run straight in the maze!

---

## 5. Serial Console Commands
| Command | Description |
| :--- | :--- |
| `start` / `tune` | Runs the full 5-stage automated calibration and commits to robot Flash |
| `stiction` | Runs deadband / breakaway stiction sweep only |
| `kv` | Runs steady-state multi-point RPM linearization only |
| `sync` | Runs dynamic sprint acceleration phase-lock test |
| `rpm` | Prints instantaneous optical RPM and pulse counts |
| `save` | Sends commit command to burn current settings to robot NVS Flash |
| `scan` | Scans and reconnects to robot over BLE |
