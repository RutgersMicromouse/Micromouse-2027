#!/usr/bin/env python3
"""
Checks that each of the simulation lab's models (sim/lab/params.h, and the harness in
test_firmware_drive.py) moves the result the way physics says it must. It does NOT check that
any number matches the real robot: none of them was measured.

Run (under WSL on the owner's PC, like the other firmware tests):
    python3 sim/tests/test_lab_models.py
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import test_firmware_drive as drive  # noqa: E402

PHYSICS = ['dynamics=1']
failures = 0


def experiment(exe, work, kind, sets):
    metrics = os.path.join(work, 'm.jsonl')
    command = [exe, '--experiment', kind, '--metrics', metrics]
    for item in sets:
        command += ['--set', item]
    subprocess.run(command, capture_output=True, text=True)
    with open(metrics, encoding='utf-8') as f:
        return [json.loads(line) for line in f if '"experiment"' in line]


def check(ok, what, detail):
    global failures
    print('  %s  %s  (%s)' % ('ok  ' if ok else 'FAIL', what, detail))
    failures += not ok


def main():
    work = tempfile.mkdtemp(prefix='mouse_labtest_')
    try:
        exe = drive.build(work)
        if exe is None:
            return 1

        print('Braking from 500 mm/s')
        brake = lambda *sets: experiment(exe, work, 'brake', PHYSICS + ['exp_speed=500'] + list(sets))[0]  # noqa: E731
        nominal, heavy = brake(), brake('mass_kg=0.3')
        check(heavy['stop_mm'] > nominal['stop_mm'] * 1.3, 'a heavier robot takes longer to stop',
              '%.0f mm at 120 g, %.0f mm at 300 g' % (nominal['stop_mm'], heavy['stop_mm']))
        slippery = brake('friction=0.3')
        check(slippery['stop_mm'] > nominal['stop_mm'] and slippery['slip_s'] > 0.05, 'a slippery floor: longer stop, wheels slip',
              '%.0f mm, slipping %.2f s' % (slippery['stop_mm'], slippery['slip_s']))
        fan = brake('friction=0.3', 'fan_on=1')
        check(fan['stop_mm'] < slippery['stop_mm'], 'the fan shortens that stop (more weight on the tyres)',
              '%.0f mm with the fan, %.0f mm without' % (fan['stop_mm'], slippery['stop_mm']))
        sealed, leaky = brake('friction=0.3', 'fan_on=1', 'fan_seal=1'), brake('friction=0.3', 'fan_on=1', 'fan_seal=0')
        check(leaky['stop_mm'] > sealed['stop_mm'], 'a fan that does not seal helps less (it is then only extra mass)',
              '%.0f mm unsealed, %.0f mm sealed' % (leaky['stop_mm'], sealed['stop_mm']))

        tall = brake('com_height_mm=40')
        check(tall['nose_dive_s'] > nominal['nose_dive_s'], 'a higher centre of mass tips onto its nose sooner under braking',
              '%.3f s nose-diving at 40 mm, %.3f s at 15 mm' % (tall['nose_dive_s'], nominal['nose_dive_s']))

        print('IR readings facing a wall')
        at = lambda rows, d, a=0: next(r for r in rows if r['distance_mm'] == d and r['angle_deg'] == a)  # noqa: E731
        normal = experiment(exe, work, 'ircal', [])
        check(at(normal, 46)['fr'] > at(normal, 90)['fr'] > at(normal, 226)['fr'], 'readings fall with distance',
              'FR %d, %d, %d at 46, 90, 226 mm' % (at(normal, 46)['fr'], at(normal, 90)['fr'], at(normal, 226)['fr']))
        weak = experiment(exe, work, 'ircal', ['ir_gain_r45=0.33'])
        check(at(weak, 46)['r45'] < 0.5 * at(normal, 46)['r45'] and abs(at(weak, 46)['l45'] - at(normal, 46)['l45']) < 0.1 * at(normal, 46)['l45'] + 10,
              'a weak right 45 pair reads low; the left one is untouched',
              'R45 %d against %d; L45 %d against %d' % (at(weak, 46)['r45'], at(normal, 46)['r45'], at(weak, 46)['l45'], at(normal, 46)['l45']))
        bright = experiment(exe, work, 'ircal', ['ir_ambient=3500'])
        check(at(bright, 30)['fr'] < at(normal, 30)['fr'], 'very bright light costs range at the top of the scale (the reading clips)',
              'FR at 30 mm: %d against %d' % (at(bright, 30)['fr'], at(normal, 30)['fr']))
        check(abs(at(experiment(exe, work, 'ircal', ['ir_ambient=600']), 90)['fr'] - at(normal, 90)['fr']) < 0.1 * at(normal, 90)['fr'],
              'moderate ambient light is subtracted out, as the firmware intends', 'FR at 90 mm within 10 %')
        shallow = experiment(exe, work, 'ircal', ['ir_falloff=1.2'])
        check(at(shallow, 226)['fr'] > at(normal, 226)['fr'], 'a gentler fall-off reads higher far away',
              'FR at 226 mm: %d against %d' % (at(shallow, 226)['fr'], at(normal, 226)['fr']))
        angled = experiment(exe, work, 'ircal', ['ir_angle_power=2'])
        check(at(angled, 90, 30)['fr'] < at(normal, 90, 30)['fr'], 'with the angle model on, a wall met at 30 degrees reads lower',
              'FR %d against %d' % (at(angled, 90, 30)['fr'], at(normal, 90, 30)['fr']))
        dropping = experiment(exe, work, 'ircal', ['ir_drop_fl=0.9'])
        check(at(dropping, 46)['fl'] < 0.6 * at(normal, 46)['fl'], 'a front-left pair that mostly gives nothing reads low',
              'FL %d against %d' % (at(dropping, 46)['fl'], at(normal, 46)['fl']))

        print('A straight of 900 mm on an empty floor')
        straight = lambda *sets: experiment(exe, work, 'straight', PHYSICS + list(sets))[0]  # noqa: E731
        true = straight()
        check(abs(true['went_mm'] - 900) < 25, 'the nominal robot goes about as far as it counts', '%.0f mm' % true['went_mm'])
        missing = straight('enc_miss_left=0.15', 'enc_miss_right=0.15')
        check(missing['went_mm'] > true['went_mm'] + 80, 'encoders that lose pulses make it drive too far',
              '%.0f mm against %.0f mm' % (missing['went_mm'], true['went_mm']))
        lopsided = straight('motor_left=0.7', 'imu_drop=0.95')
        check(abs(lopsided['left_mm']) > abs(true['left_mm']) or abs(lopsided['turned_deg']) > abs(true['turned_deg']),
              'a weak left motor with a nearly blind IMU pulls it off line',
              '%.1f mm aside, %.2f deg, against %.1f mm, %.2f deg' % (lopsided['left_mm'], lopsided['turned_deg'], true['left_mm'], true['turned_deg']))
        sagging = straight('supply_volts=9', 'sag_volts=4', 'mass_kg=0.3')
        check(sagging['took_s'] > true['took_s'], 'a heavy robot on a sagging supply takes longer over the same straight',
              '%.2f s against %.2f s' % (sagging['took_s'], true['took_s']))

        print('Turns on the spot')
        turns = lambda *sets: experiment(exe, work, 'turns', PHYSICS + list(sets))[0]  # noqa: E731
        good = turns()
        check(abs(good['left_worst_deg']) < 10 and abs(good['right_worst_deg']) < 10, 'the nominal robot turns about 90 degrees',
              'worst %+.1f left, %+.1f right' % (good['left_worst_deg'], good['right_worst_deg']))
        flipped = turns('imu_flip=1')
        check(max(abs(flipped['left_worst_deg']), abs(flipped['right_worst_deg'])) > 20, 'an IMU with the wrong sign wrecks the turns',
              'worst %+.1f left, %+.1f right' % (flipped['left_worst_deg'], flipped['right_worst_deg']))
        drifting = turns('imu_drift_deg_s=2')
        check(abs(drifting['left_mean_deg'] - good['left_mean_deg']) > 0.5 or abs(drifting['right_mean_deg'] - good['right_mean_deg']) > 0.5,
              'a drifting IMU shifts where turns end', 'left mean %+.2f against %+.2f' % (drifting['left_mean_deg'], good['left_mean_deg']))

        print('\nALL LAB MODEL CHECKS PASSED' if not failures else '\n%d LAB MODEL CHECK(S) FAILED' % failures)
        return 1 if failures else 0
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
