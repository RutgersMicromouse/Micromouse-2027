#pragma once

// Simulation lab: every number of the simulated robot and its surroundings that can be changed
// for an experiment. Included by the drive-test harness (sim/tests/test_firmware_drive.py).

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ==============================================================================
// LAB PARAMETERS
// ==============================================================================

// NONE OF THESE WERE MEASURED ON THE ROBOT unless the comment says so. They are starting guesses
// with sensible units, there to be varied: a result that only holds for one value of an unmeasured
// number is not a result.
//
// With `dynamics` 0 (the default) the harness uses its original model and every regression test
// behaves exactly as before: wheel speed follows the command through a fixed lag, nothing has
// mass. With `dynamics` 1 the forces below act.
struct LabParams {
    // ---- which model
    double dynamics = 0;            // 0 = original lag model; 1 = forces, mass, traction, slip

    // ---- robot mechanics (dynamics 1)
    double mass_kg = 0.120;         // Whole robot without a fan. Guess.
    double inertia_kg_m2 = 0.00012; // About the vertical axis through the axle centre. Guess (a 120 g disc of 45 mm radius).
    double drive_weight_share = 0.80; // Share of the weight carried by the two drive wheels (the rest is on the caster). Guess.
    double friction = 0.90;         // Tyre on floor, coefficient. Guess.
    double rolling_n = 0.010;       // Rolling resistance per wheel, newtons. Guess.
    double yaw_drag_nms = 0.00002;  // Scrub and caster drag against turning, N m per rad/s. Guess.
    double stall_force_n = 1.50;    // Push of one wheel at standstill and full effort, newtons. Guess from an N20 30:1.
    double wheel_mass_kg = 0.020;   // Each wheel with the motor's rotor as felt at the tyre. Guess.
    double tyre_stiffness = 21.0;   // Tyre force per slip speed, N per (m/s), until grip runs out. Guess (grip used up at 20 mm/s of slip on the nominal robot).
    double driver_lag_s = 0.010;    // Motor driver and winding: effort follows the command with this lag. Guess.
    double deadband = 0.030;        // Effort lost before a wheel pushes at all (the original model's friction figure)
    double motor_left = 1.0;        // Strength of each motor against nominal (1 = as the firmware assumes)
    double motor_right = 1.0;
    double com_height_mm = 15.0;    // Centre of mass above the floor. Guess.
    double com_behind_axle_mm = 8.0; // ...and behind the axle (the caster is at the back, nothing holds the nose up). Guess.

    // ---- supply
    double supply_volts = 12.0;     // What the motor driver sees and reports (a 12 V boost converter on the robot)
    double sag_volts = 0.0;         // Drop in that at full effort on both motors (dynamics 1 only)

    // ---- suction fan (dynamics 1). An approximation: a fixed extra downward force, nothing about airflow.
    double fan_on = 0;
    double fan_mass_kg = 0.015;     // Guess
    double fan_down_n = 1.0;        // Extra downward force with a perfect seal, newtons. Guess.
    double fan_seal = 0.7;          // Share of that force actually got on this floor, 0..1. Guess.
    double fan_power_w = 3.0;       // For the energy figure only. Guess.

    // ---- encoders
    double enc_miss_left = 0.0;     // Share of pulses lost, 0..1
    double enc_miss_right = 0.0;

    // ---- IR sensors. Channel order: L90, L45, FL, FR, R45, R90.
    double ir_gain[6] = { 1, 1, 1, 1, 1, 1 }; // Strength of each emitter / receiver pair against nominal
    double ir_drop[6] = { 0, 0, 0, 0, 0, 0 }; // Chance per reading that the pair gives no signal at all
    double ir_ambient = 60.0;       // Reading with the emitter off, ADC counts (the firmware subtracts it)
    double ir_noise = 4.0;          // Noise on every reading, counts (uniform, plus or minus)
    double ir_ambient_noise = 0.0;  // Extra noise per count of ambient light: bright light is noisier light
    double ir_falloff = 2.0;        // Reading falls with distance to this power. The robot's front pair measured about 1.2 on brown walls.
    double ir_angle_power = 0.0;    // Reading times cos(angle off square) to this power. 0 = the original model (angle ignored).

    // ---- IMU
    double imu_noise_deg = 0.0;     // Noise on each heading reading, degrees (uniform, plus or minus)
    double imu_drift_deg_s = 0.0;   // Heading bias growing at this rate
    double imu_drop = 0.0;          // Chance per read of an unusable answer (the robot has had 0.5 to 0.8)
    double imu_flip = 0;            // 1 = heading and turn rate reported with the wrong sign

    // ---- how a collision is judged
    double robot_radius_mm = 40.0;  // Half-width used for collisions. The robot is 87.5 mm across the tyres: 43.75.

    // ---- experiments (--experiment)
    double exp_speed = 300.0;       // mm/s for the braking experiment
    double noise_seed = 1;          // Seed of the random draws above (not of the maze)
};

struct LabField { const char* name; double LabParams::* field; double lo, hi; };
static const LabField kLabFields[] = {
    { "dynamics", &LabParams::dynamics, 0, 1 },
    { "mass_kg", &LabParams::mass_kg, 0.03, 1.0 },
    { "inertia_kg_m2", &LabParams::inertia_kg_m2, 0.00001, 0.01 },
    { "drive_weight_share", &LabParams::drive_weight_share, 0.3, 1.0 },
    { "friction", &LabParams::friction, 0.05, 3.0 },
    { "rolling_n", &LabParams::rolling_n, 0, 0.5 },
    { "yaw_drag_nms", &LabParams::yaw_drag_nms, 0, 0.01 },
    { "stall_force_n", &LabParams::stall_force_n, 0.1, 10 },
    { "wheel_mass_kg", &LabParams::wheel_mass_kg, 0.002, 0.2 },
    { "tyre_stiffness", &LabParams::tyre_stiffness, 2, 200 },
    { "driver_lag_s", &LabParams::driver_lag_s, 0.001, 0.3 },
    { "deadband", &LabParams::deadband, 0, 0.3 },
    { "motor_left", &LabParams::motor_left, 0.3, 1.5 },
    { "motor_right", &LabParams::motor_right, 0.3, 1.5 },
    { "com_height_mm", &LabParams::com_height_mm, 2, 80 },
    { "com_behind_axle_mm", &LabParams::com_behind_axle_mm, 0.5, 60 },
    { "supply_volts", &LabParams::supply_volts, 6, 14 },
    { "sag_volts", &LabParams::sag_volts, 0, 4 },
    { "fan_on", &LabParams::fan_on, 0, 1 },
    { "fan_mass_kg", &LabParams::fan_mass_kg, 0, 0.2 },
    { "fan_down_n", &LabParams::fan_down_n, 0, 10 },
    { "fan_seal", &LabParams::fan_seal, 0, 1 },
    { "fan_power_w", &LabParams::fan_power_w, 0, 50 },
    { "enc_miss_left", &LabParams::enc_miss_left, 0, 0.9 },
    { "enc_miss_right", &LabParams::enc_miss_right, 0, 0.9 },
    { "ir_ambient", &LabParams::ir_ambient, 0, 3500 },
    { "ir_noise", &LabParams::ir_noise, 0, 300 },
    { "ir_ambient_noise", &LabParams::ir_ambient_noise, 0, 1 },
    { "ir_falloff", &LabParams::ir_falloff, 1.0, 2.5 },
    { "ir_angle_power", &LabParams::ir_angle_power, 0, 4 },
    { "imu_noise_deg", &LabParams::imu_noise_deg, 0, 20 },
    { "imu_drift_deg_s", &LabParams::imu_drift_deg_s, -5, 5 },
    { "imu_drop", &LabParams::imu_drop, 0, 0.95 },
    { "imu_flip", &LabParams::imu_flip, 0, 1 },
    { "robot_radius_mm", &LabParams::robot_radius_mm, 20, 70 },
    { "exp_speed", &LabParams::exp_speed, 50, 1500 },
    { "noise_seed", &LabParams::noise_seed, 1, 1e9 },
};
static const char* const kIrNames[6] = { "l90", "l45", "fl", "fr", "r45", "r90" };

// "name=value". The six-channel ones are ir_gain_r45=0.33, ir_drop_fl=0.2 and so on.
// False (and a message) if the name is not known or the value is outside its bounds.
static bool labSet(LabParams& p, const char* assignment) {
    const char* equals = strchr(assignment, '=');
    if (!equals) { printf("  --set needs name=value, got '%s'\n", assignment); return false; }
    const size_t len = (size_t)(equals - assignment);
    const double value = atof(equals + 1);
    for (const LabField& f : kLabFields) {
        if (strlen(f.name) == len && strncmp(f.name, assignment, len) == 0) {
            if (value < f.lo || value > f.hi) { printf("  %s must be %g .. %g\n", f.name, f.lo, f.hi); return false; }
            p.*(f.field) = value;
            return true;
        }
    }
    for (int i = 0; i < 6; i++) {
        char name[24];
        snprintf(name, sizeof(name), "ir_gain_%s", kIrNames[i]);
        if (strlen(name) == len && strncmp(name, assignment, len) == 0 && value >= 0 && value <= 5) { p.ir_gain[i] = value; return true; }
        snprintf(name, sizeof(name), "ir_drop_%s", kIrNames[i]);
        if (strlen(name) == len && strncmp(name, assignment, len) == 0 && value >= 0 && value <= 1) { p.ir_drop[i] = value; return true; }
    }
    printf("  no lab parameter '%.*s' (or its value is out of bounds)\n", (int)len, assignment);
    return false;
}

// Every parameter with its value and bounds, as a JSON list (--list-params; the viewer's lab bench builds its form from it)
static void labList(const LabParams& p, FILE* out) {
    fputc('[', out);
    for (const LabField& f : kLabFields) fprintf(out, "{\"name\":\"%s\",\"value\":%g,\"lo\":%g,\"hi\":%g},", f.name, p.*(f.field), f.lo, f.hi);
    for (int i = 0; i < 6; i++) {
        fprintf(out, "{\"name\":\"ir_gain_%s\",\"value\":%g,\"lo\":0,\"hi\":5},{\"name\":\"ir_drop_%s\",\"value\":%g,\"lo\":0,\"hi\":1}%s",
                kIrNames[i], p.ir_gain[i], kIrNames[i], p.ir_drop[i], i < 5 ? "," : "");
    }
    fputs("]\n", out);
}

// Every parameter as one JSON object, so a result can always be traced back to its settings
static void labDump(const LabParams& p, FILE* out) {
    fputc('{', out);
    for (const LabField& f : kLabFields) fprintf(out, "\"%s\":%g,", f.name, p.*(f.field));
    for (int i = 0; i < 6; i++) fprintf(out, "\"ir_gain_%s\":%g,\"ir_drop_%s\":%g%s", kIrNames[i], p.ir_gain[i], kIrNames[i], p.ir_drop[i], i < 5 ? "," : "");
    fputc('}', out);
}
