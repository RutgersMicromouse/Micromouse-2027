# 🛡 Antigravitieee Micromouse — Engineering Quality & System Constraints

This document defines the strict non-negotiable operational, timing, safety, and architectural constraints for the **Antigravitieee** Micromouse platform. All future changes and autonomous agent actions must adhere to these contracts.

---

## 1. Real-Time Timing Constraints (Core 1)

| Parameter | Constraint | Threshold / Target | Violation Action |
| :--- | :--- | :--- | :--- |
| **Control Loop Frequency** | Strictly periodic | $500\text{ Hz}\ (\pm 0.5\%)$ | FreeRTOS task delay warning |
| **Control Loop Period ($T$)** | Fixed | $2000\,\mu\text{s}$ | — |
| **Max Execution Latency** | Must not exceed | $< 1200\,\mu\text{s}\ (60\%\text{ budget})$ | Alert via telemetry & `perf` command |
| **Loop Overrun Tolerance** | Hard deadline | $0\text{ overruns in normal run}$ | Increment `timing_stats.loop_overruns` |
| **I2C Fast Mode Speed** | Clock rate | $400\text{ kHz}$ | Hardware bus lockup check |
| **IMU Polling Rate** | Dedicated interval | $100\text{ Hz}$ (every 5th tick) | Prevents I2C bus congestion |

---

## 2. Memory & Concurrency Constraints

* **Zero Dynamic Heap Allocation in Core 1**:
  * No `malloc()`, `free()`, `new`, `delete`, or dynamic `std::vector` / `String` mutations inside `motionControlTask`. All buffers must be pre-allocated statically or on the stack.
* **Stack High-Water Margin**:
  * FreeRTOS tasks must maintain at least **512 words ($2\text{ KB}$)** of remaining stack headroom under maximum load (`uxTaskGetStackHighWaterMark`).
* **Inter-Task Communication**:
  * Core 0 and Core 1 communication is strictly mediated via non-blocking queues (`g_motion_cmd_queue`) and mutex-guarded telemetry snapshots (`g_telemetry_mutex`).

---

## 3. Electrical & Failsafe Constraints

* **Computer Battery Voltage Cutoff**:
  * **Chemistry**: Separate 1S Li-ion/LiPo cell ($3.7\text{ V}$ nominal).
  * **Cutoff Voltage**: $3.30\text{ V}$.
  * GPIO12 (`VSenseCom`) measures this battery through the 10k/10k divider.
  * If voltage drops below $3.30\text{ V}$ for $>200\text{ ms}$, the robot cuts motor power, engages emergency stop, and turns the LED **Solid Red**.
  * The separate motor battery is not monitored by this firmware cutoff.
* **Motor Coil Stall Protection**:
  * If commanded velocity exceeds $80\text{ mm/s}$ while measured linear speed $< 15\text{ mm/s}$ at $>35\%$ effort for $>200\text{ ms}$ ($100\text{ ticks}$), the robot triggers immediate emergency stop to prevent motor burn-out.
* **Emergency Stop Latency**:
  * Tapping either physical button or receiving `stop` wirelessly must brake motors within $\le 2\text{ ms}$.

---

## 4. Kinematic & Actuator Invariants

* **Motors**: N20 12V DC Metal Gearmotors (30:1 gear reduction ratio).
* **Encoders**: Magnetic quadrature encoders on rear shaft ($7\text{ pulses/channel/rev}$).
* **Total Quadrature CPR**: $7.0 \times 4 \times 30 = 840.0\text{ counts/wheel rev}$.
* **Wheel Diameter**: $24.0\text{ mm}$.
* **Linear Distance Resolution**: $\approx 0.0898\text{ mm/tick}$ ($11.14\text{ ticks/mm}$).
* **Search Kinematics Limits**:
  * Max Search Speed: $240\text{ mm/s}$
  * Max Search Acceleration: $1500\text{ mm/s}^2$
  * Max Search In-Place Turn: $360^\circ/\text{s}$ ($1800^\circ/\text{s}^2$)

---

## 5. Verification & Test Gates

Before committing or flashing firmware to physical hardware:
1. **Compilation Gate**: All 5 PlatformIO environments must compile with 0 warnings and 0 errors:
   ```powershell
   pio run
   ```
2. **Algorithm Test Suite**: Desktop test suite must pass with 100% green assertions:
   ```powershell
   python test/test_desktop_suite.py
   ```
3. **Simulation Verification**: Full 10-maze championship tournament run must complete collision-free:
   ```powershell
   python sim/verify_headless.py
   ```
