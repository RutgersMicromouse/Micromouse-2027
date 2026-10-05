# Curve simulator model and limits

The GUI and `verify_headless.py` share a fixed-step, 500 Hz differential-drive
simulation. It includes approximate wheel-speed/acceleration limits, motor
response and mismatch, traction variation, six ray-cast IR channels, and
swept-footprint wall/post collision checks. A cell-based spatial index narrows
sensor and collision queries to nearby obstacles without reducing geometric
collision precision. The headless verifier does not snap the robot onto the
goal.

Run the focused physical-model tests and a small headless trial from the
repository root:

```powershell
python test\test_physics_model.py
python sim\verify_headless.py --mazes 1 --trials 1
```

Use `--mazes 10` for all included mazes. `--trials 2` and `--trials 3` also
exercise deterministic lower-motor-output and lower-traction scenarios.

## Calibration required for hardware correlation

Several values are configuration-based or initial geometric assumptions, not
measurements of this particular robot: the 700 mm/s nominal wheel-speed cap,
motor/traction response, 34 x 40 mm body half-dimensions, wall/post dimensions,
IR sensor mounting offsets, and ADC distance/angle response. The configured
speed and acceleration limits are planner targets, not proof that the robot
can safely achieve them. IR defaults in firmware may also differ from values
persisted during on-robot calibration.

For useful hardware correlation, measure loaded wheel speed and acceleration
at representative battery voltages, braking distance, left/right mismatch,
wheelbase and chassis outline, maze wall/post geometry, each IR channel's
response against distance and angle, sensor latency, and traction on the
competition floor. Replace the corresponding simulator assumptions with
those measured distributions, then compare logged firmware runs against
simulator traces.

## Scope

The route generator uses the complete maze layout; exploration is not a
simulation of the firmware's sensor-driven mapping/navigation state machine.
The simulated IR readings and opening estimates are not fed into firmware
navigation. This is a path-physics and collision sandbox, not firmware-in-the-
loop, and a simulated pass cannot guarantee a real-world pass or collision-
free run. Validate new speeds progressively on the actual robot in a
controlled test area.
