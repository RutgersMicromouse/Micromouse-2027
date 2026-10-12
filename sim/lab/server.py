#!/usr/bin/env python3
"""
The simulation lab's local server: serves the viewer (sim/viewer/) and runs simulations for its
"Lab bench" tab, so settings can be changed in the page and tried at once.

    python3 sim/lab/server.py            then open http://localhost:8001/viewer/
    (from Windows: sim\\viewer\\run_viewer.bat does both)

It compiles the firmware and the harness once at start, listens on this computer only, and runs
nothing but that one program, with arguments it has checked. Nothing here reaches the robot.
"""
import functools
import http.server
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, HERE)
import lab  # noqa: E402

PORT = 8001
SETTING = re.compile(r'^[a-z0-9_]+=-?[0-9.]+(e-?[0-9]+)?$')  # name=number, nothing else reaches the command line
EXPERIMENTS = ('brake', 'straight', 'turns', 'ircal', 'limits', 'limits_hard')
MAZES = ('curves_5x5', 'diagonals_5x5')  # The layouts the checklist tells the owner to build
work = tempfile.mkdtemp(prefix='mouse_bench_')
exe = None


def simulate(request):
    """One run for the bench. `request`: sets, size, seed, robot_tuning, experiment (or none)."""
    sets = [s for s in request.get('sets', []) if SETTING.match(s)]
    if len(sets) != len(request.get('sets', [])):
        return {'error': 'a setting was not name=number'}
    size = request.get('size', 5)
    seed = int(request.get('seed', 1))
    experiment = request.get('experiment')
    maze = request.get('maze')
    if size not in (3, 5, 8, 16) or not 1 <= seed <= 1000000 or experiment not in (None,) + EXPERIMENTS or maze not in (None,) + MAZES:
        return {'error': 'size must be 3, 5, 8 or 16; seed 1 to 1000000; experiment and maze from the lists in server.py'}

    metrics = os.path.join(work, 'metrics.jsonl')
    command = [exe, '--metrics', metrics, '--set', 'noise_seed=%d' % seed]
    command += lab.program_arguments(sets)
    for item in (lab.ROBOT_TUNING if request.get('robot_tuning') else []):
        command += ['--tune', item]
    if experiment:
        # (a floor of 8 x 8 cells is room enough for the turns and the straight and still easy to see;
        #  braking from speed needs the full 16)
        command += ['--experiment', experiment, '--size', '16' if experiment == 'brake' else '8']
        if experiment != 'ircal':  # (that one places the robot by hand: nothing to watch)
            command += ['--trace', lab.VIEWER_TRACE]
    else:
        command += ['--quiet', '--trace', lab.VIEWER_TRACE, '--size', str(size)]
        command += [os.path.join(lab.ROOT, 'sim', 'mazes', 'checklist', maze + '.num')] if maze else ['--random', '1', '--seed', str(seed)]
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=900)
    except subprocess.TimeoutExpired:
        return {'error': 'the simulation did not finish in 15 minutes'}
    lines = lab.read_lines(metrics) if os.path.exists(metrics) else []
    runs = [dict(line, failures=lab.run_failures(line)) for line in lines if 'run' in line]
    return {'output': done.stdout[-4000:], 'runs': runs, 'rows': [line for line in lines if 'experiment' in line],
            'failed': not experiment and (not runs or any(run['failures'] for run in runs))}


class Handler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):  # (or the browser keeps showing yesterday's page and trace)
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

    def reply(self, body):
        data = json.dumps(body).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == '/api/params':
            listed = subprocess.run([exe, '--list-params'], capture_output=True, text=True).stdout
            self.reply({'params': json.loads(listed), 'physics': lab.PHYSICS, 'robot_tuning': lab.ROBOT_TUNING,
                        'scenarios': {name: {'what': what, 'sets': None if sets is None else lab.PHYSICS + sets}
                                      for name, (what, sets) in lab.SCENARIOS.items()}})
        else:
            super().do_GET()

    def do_POST(self):
        if self.path != '/api/run':
            self.send_error(404)
            return
        try:
            request = json.loads(self.rfile.read(min(int(self.headers.get('Content-Length', 0)), 100000)))
            self.reply(simulate(request))
        except (ValueError, TypeError) as error:
            self.reply({'error': 'bad request: %s' % error})

    def log_message(self, *args):  # (quiet: one line per request is noise here)
        pass


def main():
    global exe
    try:
        exe = lab.drive.build(work)
        if exe is None:
            return 1
        handler = functools.partial(Handler, directory=os.path.join(ROOT, 'sim'))
        with http.server.ThreadingHTTPServer(('127.0.0.1', PORT), handler) as server:
            print('Simulation lab ready: http://localhost:%d/viewer/   (Ctrl+C to stop)' % PORT)
            sys.stdout.flush()
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        shutil.rmtree(work, ignore_errors=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
