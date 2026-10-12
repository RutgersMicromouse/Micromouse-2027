"""
The robot's own checklist code, run start to finish on the simulated robot.

src/ui/ladder, src/ui/limits_test and src/ui/turn_test cannot be exercised by the other PC tests:
they are the code that decides, on the robot, whether a step passed. Here they are compiled as
they are (see the stand-in robot.h in test_firmware_drive.py) and taken through everything the
owner would do: braking measured, the six steps, the search-speed trial. It passes if the code
runs to the end, every step finishes, and nothing hangs or is stopped by the safety supervisor.

It does NOT show that the real robot will pass: the physics are guesses. It shows the code works.

    python3 sim/tests/test_checklist_sim.py        (takes a few minutes)
"""

import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(os.path.dirname(HERE), 'lab'))
import lab  # noqa: E402
import test_firmware_drive as drive  # noqa: E402


def main():
    work = tempfile.mkdtemp(prefix='mouse_checklist_')
    try:
        exe = drive.build(work)
        if exe is None:
            return 1
        metrics = os.path.join(work, 'm.jsonl')
        command = [exe, '--experiment', 'checklist', '--verbose', '--metrics', metrics] + lab.program_arguments(lab.PHYSICS + lab.REAL_IR)
        output = subprocess.run(command, capture_output=True, text=True).stdout
        lines = lab.checklist_lines(output)
        print('\n'.join(line for line in lines if not line.lstrip().startswith(('truth:', '(that'))))

        result = [line for line in lab.read_lines(metrics) if line.get('experiment') == 'checklist']
        failures = []
        if not result:
            failures.append('the checklist did not run to its end')
        elif result[0]['open_step'] != 7:
            failures.append('the checklist stopped with step %d still open' % result[0]['open_step'])
        for bad, what in (('OUT OF TIME', 'a run never finished'), ('[SAFE] BRAKING', 'the safety supervisor braked a run'),
                          ('[SAFE] FAULT', 'the safety supervisor reported a fault'), ('did not finish', 'a search did not finish')):
            if any(bad in line for line in lines):
                failures.append(what)
        searches_touched = [line for line in lines if 'truth: search' in line and 'IT TOUCHED' in line]
        if searches_touched:
            failures.append('a search touched a wall')
        runs = [line for line in lines if 'truth: speed run' in line]
        touched = [line for line in runs if 'IT TOUCHED' in line]
        print('\n  %d speed runs judged by the robot, of which %d really touched something' % (len(runs), len(touched)))
        if result:
            print('  as the robot left it:', json.dumps(result[0]['state']))
        for failure in failures:
            print('    FAIL', failure)
        print('\nTHE CHECKLIST CODE RUNS TO THE END' if not failures else '\n%d CHECK(S) FAILED' % len(failures))
        return 1 if failures else 0
    finally:
        shutil.rmtree(work, ignore_errors=True)


if __name__ == '__main__':
    sys.exit(main())
