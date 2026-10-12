#!/usr/bin/env python3
"""
Simulation lab: runs named scenarios, repeated trials, comparisons and sweeps with the REAL
firmware on the simulated robot (the harness in sim/tests/test_firmware_drive.py), and judges
every run against fixed limits, not just "did it reach the goal".

    python3 sim/lab/lab.py list
    python3 sim/lab/lab.py run baseline weak_r45 --trials 5        each on 5 mazes (seeds 1..5)
    python3 sim/lab/lab.py compare fan_off fan_on --set friction=0.4
    python3 sim/lab/lab.py sweep mass_kg 0.08,0.12,0.2,0.3
    python3 sim/lab/lab.py regression                              every scenario once
    python3 sim/lab/lab.py drift worst_case --size 16 --trials 3   does position error build up through a maze?
    python3 sim/lab/lab.py experiment brake                        braking distance at several speeds
    python3 sim/lab/lab.py experiment turns --set mass_kg=0.2
    python3 sim/lab/lab.py experiment ircal --set ir_ambient=1500
    python3 sim/lab/lab.py run baseline --trials 5 --robot-tuning   with the tuning saved on the robot, not the code's

Options: --size 3|5|8|16 (default 5), --maze sim/mazes/x.num (a full-size file instead of random
mazes), --trials N, --set name=value (any parameter of sim/lab/params.h, on top of the scenario).

Every result is written to sim/lab/results/ with the parameters, the seed and the firmware's
git commit, and one line is added to results/history.jsonl. The trace of the last trial (the
first failing one, if any failed) is left at sim/viewer/trace.jsonl for the viewer; traces of
failed trials are also kept under results/traces/.

Nothing here touches the real robot's saved tuning. It is a simulation: the physics numbers are
guesses (see params.h), so read differences between scenarios, not absolute values.
"""
import argparse
import datetime
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, os.path.join(ROOT, 'sim', 'tests'))
import test_firmware_drive as drive  # noqa: E402

RESULTS = os.path.join(HERE, 'results')
VIEWER_TRACE = os.path.join(ROOT, 'sim', 'viewer', 'trace.jsonl')

# The lab's own robot: forces and mass on, and collisions judged at the robot's real half-width
PHYSICS = ['dynamics=1', 'robot_radius_mm=43.75']

# The tuning set saved on the robot on 2026-10-10 (stages D and E, AGENTS.md). Since 2026-10-11 the code's
# own values are this set, so --robot-tuning changes nothing until one of the two moves; keep this list
# as what is saved on the robot. dist_k is left out: it makes up for the real tyres' size, and the
# simulated wheels are exactly the configured size.
ROBOT_TUNING = ['h_kp=0.005', 'h_ki=0.03', 'h_kd=0', 'h_imax=0.05', 'w_max=10', 'w_gyro=0.025',
                'turn_ff=0.84', 't_ka=0.000125', 't_push=0.06', 't_damp=0.0004']

# name: (what it is, settings on top of PHYSICS). 'ideal' is the original model the regression tests use.
SCENARIOS = {
    'ideal':           ('The original model: no mass, perfect sensors (what the regression tests run)', None),
    'baseline':        ('Force model, nominal robot, healthy sensors', []),
    'bright':          ('Bright ambient light: high, noisy background on every receiver', ['ir_ambient=1500', 'ir_ambient_noise=0.02']),
    'dim':             ('Dim light: almost no background', ['ir_ambient=10']),
    'noisy_ir':        ('Ten times the receiver noise', ['ir_noise=40']),
    'weak_r45':        ('The robot\'s known fault: right 45 degree pair at a third of its strength', ['ir_gain_r45=0.33']),
    'flaky_fl':        ('The robot\'s other known fault: left front pair giving nothing 3 readings in 10', ['ir_drop_fl=0.3']),
    'heavy':           ('Mass up from 120 g to 200 g, turning inertia with it', ['mass_kg=0.2', 'inertia_kg_m2=0.0002']),
    'low_friction':    ('Slippery floor or dusty tyres: friction 0.4', ['friction=0.4']),
    'high_grip':       ('Grippy tyres: friction 1.6', ['friction=1.6']),
    'fan_off':         ('No suction fan (same as baseline; the partner of fan_on)', []),
    'fan_on':          ('Suction fan on: 15 g heavier, 0.7 N more on the floor', ['fan_on=1']),
    'low_battery':     ('Supply down to 9.5 V and sagging 1.5 V under load', ['supply_volts=9.5', 'sag_volts=1.5']),
    'unequal_motors':  ('Left motor 15 % weaker than the right', ['motor_left=0.85']),
    'imu_faults':      ('IMU: 6 reads in 10 unusable, half a degree of noise', ['imu_drop=0.6', 'imu_noise_deg=0.5']),
    'encoder_faults':  ('Right encoder losing 10 % of its pulses, on a slippery floor', ['enc_miss_right=0.1', 'friction=0.5']),
    'curves':          ('Speed runs with smooth 90 degree curves (the firmware\'s move set 1), nominal robot', ['moves=1']),
    'diagonals':       ('Speed runs with curves, diagonals and V turns (move set 2), nominal robot', ['moves=2']),
    'diagonals_true_size': ('The same, weak R45 and flaky FL as on the robot', ['moves=2', 'ir_gain_r45=0.33', 'ir_drop_fl=0.3']),
    'worst_case':      ('Weak R45, flaky FL, bright light, IMU faults, low friction, heavy, unequal motors, low battery',
                        ['ir_gain_r45=0.33', 'ir_drop_fl=0.3', 'ir_ambient=1500', 'ir_ambient_noise=0.02', 'imu_drop=0.6',
                         'friction=0.5', 'mass_kg=0.2', 'inertia_kg_m2=0.0002', 'motor_left=0.9', 'supply_volts=10', 'sag_volts=1']),
}


def program_arguments(sets):
    """Settings as arguments of the drive-test program. 'moves=N' is the firmware's move set for
    speed runs (0 turns on the spot, 1 curves, 2 diagonals), not a physics parameter."""
    arguments = []
    for item in sets:
        arguments += ['--moves', item[6:]] if item.startswith('moves=') else ['--set', item]
    return arguments


# A run fails if any of these is broken, whether or not it reached the goal.
# (metric, 'max' or 'min', limit, what it means). The limits are the lab's choice, not measurements.
LIMITS = [
    ('collisions', 'max', 0, 'touched a wall or post'),
    ('safety_brakings', 'max', 0, 'the safety supervisor had to brake'),
    ('nose_dive_s', 'max', 0.02, 'braked hard enough to tip onto its nose (by the guessed centre of mass)'),
    # (30, not less: the firmware's own wall steering may ask for up to 25 deg, w_max in the code)
    ('heading_err_max_deg', 'max', 30.0, 'pointed more than 30 deg off its axis on a straight'),
    ('lateral_max_mm', 'max', 35.0, 'more than 35 mm off the middle of the corridor'),
    ('left_turn_worst_deg', 'abs', 8.0, 'a left turn more than 8 deg off 90'),
    ('right_turn_worst_deg', 'abs', 8.0, 'a right turn more than 8 deg off 90'),
    ('off_centre_at_turn_max_mm', 'max', 30.0, 'began a turn more than 30 mm from the cell centre'),
    ('ir_accuracy', 'min', 0.97, 'called a wall wrongly more than 3 times in 100'),
]
SHOWN = ['time_s', 'min_clearance_mm', 'heading_err_max_deg', 'lateral_max_mm', 'left_turn_worst_deg', 'right_turn_worst_deg',
         'off_centre_at_turn_max_mm', 'slip_s', 'nose_dive_s', 'saturated_s', 'energy_j', 'ir_accuracy']


def run_failures(run):
    """Why one run (a search or a speed run) fails, as a list of reasons. Empty = it passed."""
    reasons = []
    if run['abort']:
        reasons.append('aborted: ' + run['abort'])
    elif not run['finished']:
        reasons.append('ran out of time')
    wanted_state = 4 if run['run'].startswith('search') else 6  # NavState: search done / speed run finished
    if not reasons and run['nav_state'] != wanted_state:
        reasons.append('did not end where it should (navigation state %d)' % run['nav_state'])
    for metric, kind, limit, meaning in LIMITS:
        value = run[metric]
        if (kind == 'max' and value > limit) or (kind == 'min' and value < limit) or (kind == 'abs' and abs(value) > limit):
            reasons.append('%s (%s = %g)' % (meaning, metric, value))
    return reasons


def read_lines(path):
    with open(path, encoding='utf-8') as f:
        return [json.loads(line) for line in f if line.strip()]


def commit():
    try:
        return subprocess.run(['git', '-C', ROOT, 'rev-parse', '--short', 'HEAD'], capture_output=True, text=True).stdout.strip()
    except OSError:
        return ''


def scenario_sets(name, extra):
    what, sets = SCENARIOS[name]
    return ([] if sets is None else PHYSICS + sets) + list(extra)


def run_scenario(exe, work, name, sets, opts):
    """Runs `opts.trials` mazes of one scenario. Returns the result record (also written to disk)."""
    stamp = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    trials, params, first_failed_trace = [], None, None
    for seed in range(1, opts.trials + 1):
        metrics, trace = os.path.join(work, 'metrics.jsonl'), os.path.join(work, 'trace.jsonl')
        command = [exe, '--quiet', '--metrics', metrics, '--trace', trace, '--set', 'noise_seed=%d' % seed]
        command += program_arguments(sets)
        for item in (ROBOT_TUNING if opts.robot_tuning else []):
            command += ['--tune', item]
        command += [opts.maze] if opts.maze else ['--size', str(opts.size), '--random', '1', '--seed', str(seed)]
        subprocess.run(command, capture_output=True, text=True)
        lines = read_lines(metrics)
        params = lines[0]['params']
        runs = [line for line in lines[1:] if 'run' in line]
        failures = ['%s: %s' % (run['run'], reason) for run in runs for reason in run_failures(run)]
        if not runs:
            failures = ['nothing ran (calibration or start-up failed)']
        trials.append({'seed': seed, 'runs': runs, 'failures': failures})
        if failures:
            os.makedirs(os.path.join(RESULTS, 'traces'), exist_ok=True)
            shutil.copy(trace, os.path.join(RESULTS, 'traces', '%s_%s_seed%d.jsonl' % (stamp, name, seed)))
            first_failed_trace = first_failed_trace or os.path.join(RESULTS, 'traces', '%s_%s_seed%d.jsonl' % (stamp, name, seed))
        shutil.copy(first_failed_trace or trace, VIEWER_TRACE)
        print('  %-16s seed %d: %s' % (name, seed, 'pass' if not failures else 'FAIL  ' + failures[0] +
                                       (' (+%d more)' % (len(failures) - 1) if len(failures) > 1 else '')))

    failed = sum(1 for trial in trials if trial['failures'])
    summary = {'trials': len(trials), 'failed': failed, 'failure_rate': failed / len(trials)}
    if len(trials) >= 5:  # Wilson interval: where the true failure rate plausibly lies, 95 %
        n, p, z = len(trials), failed / len(trials), 1.96
        centre, spread = (p + z * z / (2 * n)) / (1 + z * z / n), z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
        summary['failure_rate_95'] = [max(0.0, centre - spread), min(1.0, centre + spread)]
    for metric in SHOWN:
        values = [run[metric] for trial in trials for run in trial['runs']]
        if values:
            mean = sum(values) / len(values)
            worst = min(values) if metric in ('min_clearance_mm', 'ir_accuracy') else max(values, key=abs)
            summary[metric] = {'mean': mean, 'sd': math.sqrt(sum((v - mean) ** 2 for v in values) / len(values)), 'worst': worst}

    name_shown = name + (' +robot tuning' if opts.robot_tuning else '')
    record = {'scenario': name_shown, 'what': SCENARIOS.get(name, ('', None))[0], 'when': stamp, 'commit': commit(),
              'maze': opts.maze or 'random %dx%d, seeds 1..%d' % (opts.size, opts.size, opts.trials),
              'sets': sets, 'tuning': ROBOT_TUNING if opts.robot_tuning else 'the values in the code', 'params': params, 'limits': LIMITS, 'summary': summary, 'trials': trials}
    os.makedirs(RESULTS, exist_ok=True)
    path = os.path.join(RESULTS, '%s_%s.json' % (stamp, name))
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(record, f, indent=1)

    # Regression: the same scenario on the same mazes and firmware settings passed everything last time
    history_path = os.path.join(RESULTS, 'history.jsonl')
    before = [h for h in (read_lines(history_path) if os.path.exists(history_path) else [])
              if h['scenario'] == name_shown and h['maze'] == record['maze'] and h['sets'] == sets]
    regression = bool(before) and before[-1]['failed'] == 0 and failed > 0
    with open(history_path, 'a', encoding='utf-8') as f:
        f.write(json.dumps({'when': stamp, 'scenario': name_shown, 'maze': record['maze'], 'sets': sets, 'commit': record['commit'],
                            'trials': len(trials), 'failed': failed, 'regression': regression,
                            'first_failure': next((t['failures'][0] for t in trials if t['failures']), ''),
                            'worst_clearance_mm': summary.get('min_clearance_mm', {}).get('worst'),
                            'file': os.path.basename(path)}) + '\n')
    if regression:
        print('  REGRESSION: %s passed every trial on %s (commit %s) and fails now' % (name, before[-1]['when'], before[-1]['commit']))
    return record


def table(records):
    names = [r['scenario'] for r in records]
    print('\n%-28s' % '' + ''.join('%-22s' % n[:21] for n in names))
    print('%-28s' % 'trials failed' + ''.join('%-22s' % ('%d of %d' % (r['summary']['failed'], r['summary']['trials'])) for r in records))
    for metric in SHOWN:
        cells = []
        for r in records:
            m = r['summary'].get(metric)
            cells.append('%-22s' % ('-' if m is None else '%.3g (worst %.3g)' % (m['mean'], m['worst'])))
        print('%-28s' % metric + ''.join(cells))
    print('\nmean (worst) over every run of every trial. Details: sim/lab/results/. Viewer: sim/viewer (trace of the last or first failing trial).')


def drift(exe, work, sets, opts):
    """Does position error build up through a maze? Drives the first search of one full-size
    maze per trial and looks at where the robot REALLY stood each time it began a turn on the
    spot, in order: millimetres off the cell centre, split into across the old heading (what a
    straight leaves behind, and what clips a wall in the turn) and along it (stopping short or
    long). If the later turns are no worse than the early ones, nothing is building up."""
    turns_on_the_spot = (5, 6, 13)  # MotionAction: left 90, right 90, round 180 (src/types.h)
    worst = 0.0
    for seed in range(1, opts.trials + 1):
        trace = os.path.join(work, 'drift.jsonl')
        command = [exe, '--quiet', '--size', str(opts.size), '--random', '1', '--seed', str(seed), '--trace', trace]
        command += program_arguments(sets + ['noise_seed=%d' % seed])
        subprocess.run(command, capture_output=True, text=True)
        offsets, previous = [], None
        for frame in read_lines(trace):
            if 'end' in frame:
                break  # (the first search only: the speed runs follow it in the trace)
            if 't' not in frame:
                continue
            if frame['act'] in turns_on_the_spot and (previous is None or previous['act'] != frame['act']):
                dx = frame['x'] - round(frame['x'] / 180.0) * 180.0
                dy = frame['y'] - round(frame['y'] / 180.0) * 180.0
                heading = math.radians(round(frame['h'] / 90.0) * 90.0)  # 0 = up the maze, counted to the left
                along = -dx * math.sin(heading) + dy * math.cos(heading)
                across = dx * math.cos(heading) + dy * math.sin(heading)
                offsets.append((abs(across), abs(along)))
            previous = frame
        if len(offsets) < 6:
            print('  maze %d: only %d turns on the spot, too few to say' % (seed, len(offsets)))
            continue
        third = len(offsets) // 3
        parts = [offsets[:third], offsets[third:2 * third], offsets[2 * third:]]
        text = ' | '.join('%s third: across %.1f (worst %.1f), along %.1f (worst %.1f)' % (
            name, sum(a for a, _ in part) / len(part), max(a for a, _ in part), sum(b for _, b in part) / len(part), max(b for _, b in part))
            for name, part in zip(('first', 'middle', 'last'), parts))
        worst = max(worst, max(max(a, b) for a, b in offsets))
        print('  maze %d, %d turns on the spot, mm off the cell centre when each began: %s' % (seed, len(offsets), text))
    print('  worst of all: %.1f mm (a wall is %.1f mm from the body when the robot is centred)' % (worst, 84.0 - 6.0 - 43.75))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=['list', 'run', 'compare', 'sweep', 'regression', 'experiment', 'drift'])
    parser.add_argument('names', nargs='*')
    parser.add_argument('--trials', type=int, default=1)
    parser.add_argument('--size', type=int, default=5, choices=[3, 5, 8, 16])
    parser.add_argument('--maze')
    parser.add_argument('--set', action='append', default=[], dest='sets')
    parser.add_argument('--base', default='baseline')
    parser.add_argument('--robot-tuning', action='store_true', help='use the tuning set saved on the robot, not the values in the code')
    opts = parser.parse_args()

    if opts.command == 'list':
        for name, (what, sets) in SCENARIOS.items():
            print('%-16s %s\n%-16s   %s' % (name, what, '', 'original model' if sets is None else ' '.join(PHYSICS + sets)))
        return 0

    work = tempfile.mkdtemp(prefix='mouse_lab_')
    try:
        exe = drive.build(work)
        if exe is None:
            return 1
        if opts.command == 'experiment':
            kind = opts.names[0] if opts.names else 'brake'
            sets = scenario_sets(opts.base, opts.sets)
            speeds = [150, 250, 350, 500, 700] if kind == 'brake' else [None]
            os.makedirs(RESULTS, exist_ok=True)
            out = os.path.join(RESULTS, '%s_experiment_%s.jsonl' % (datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'), kind))
            with open(out, 'w', encoding='utf-8') as collected:
                for speed in speeds:
                    command = [exe, '--experiment', kind, '--metrics', os.path.join(work, 'm.jsonl')]
                    command += program_arguments(sets + (['exp_speed=%d' % speed] if speed else []))
                    for item in (ROBOT_TUNING if opts.robot_tuning else []):
                        command += ['--tune', item]
                    print(subprocess.run(command, capture_output=True, text=True).stdout.rstrip())
                    collected.write(open(os.path.join(work, 'm.jsonl'), encoding='utf-8').read())
            print('written to', os.path.relpath(out, ROOT))
            return 0

        if opts.command == 'drift':
            return drift(exe, work, scenario_sets(opts.names[0] if opts.names else opts.base, opts.sets), opts)

        if opts.command == 'sweep':
            parameter, values = opts.names[0], opts.names[1].split(',')
            records = []
            for value in values:
                name = '%s=%s' % (parameter, value)
                SCENARIOS[name] = ('sweep of %s on %s' % (parameter, opts.base), None)
                records.append(run_scenario(exe, work, name, scenario_sets(opts.base, opts.sets) + [name], opts))
        else:
            names = list(SCENARIOS) if opts.command == 'regression' else opts.names
            unknown = [n for n in names if n not in SCENARIOS]
            if unknown or not names:
                print('unknown or missing scenario:', ', '.join(unknown) or '(none given)', '- try: lab.py list')
                return 2
            records = [run_scenario(exe, work, name, scenario_sets(name, opts.sets), opts) for name in names]
        table(records)
        return 1 if any(r['summary']['failed'] for r in records) else 0
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
