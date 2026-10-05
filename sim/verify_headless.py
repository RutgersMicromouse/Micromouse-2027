"""Run collision checks using the same unsnapped 500 Hz dynamics as the GUI."""

import argparse
import math
import os
import sys

os.environ["SDL_VIDEODRIVER"] = "dummy"
sys.path.insert(0, os.path.dirname(__file__))

import curve_sim


def run_phase(sim, phase, max_sim_seconds):
    sim.set_phase(phase, keep_running=True)
    max_steps = math.ceil(max_sim_seconds / curve_sim.CONTROL_DT_S)
    for step in range(max_steps):
        sim.traj_idx, collision, reached_goal = curve_sim.simulation_tick(
            sim.bot, sim.collider, sim.traj_points, sim.traj_idx,
            curve_sim.CONTROL_DT_S)
        if collision[0]:
            return f"CRASH: {collision[1]} at ({collision[2][0]:.1f}, {collision[2][1]:.1f})"
        if reached_goal:
            return f"PASS ({(step + 1) * curve_sim.CONTROL_DT_S:.2f}s)"
    final = sim.traj_points[-1]
    position_error = math.hypot(final.x - sim.bot.x, final.y - sim.bot.y)
    heading_error = abs(curve_sim.normalize_angle(final.theta - sim.bot.theta))
    return (
        f"TIMEOUT at traj_idx={sim.traj_idx}/{len(sim.traj_points)} "
        f"(position_error={position_error:.1f}mm, "
        f"heading_error={math.degrees(heading_error):.1f}deg, "
        f"speed={sim.bot.v:.1f}mm/s)"
    )


def verify_all_mazes(trials, maze_limit):
    all_pass = True
    for trial in range(trials):
        motor_scale = (1.0, 0.90, 0.95)[trial % 3]
        friction_scale = (1.0, 0.80, 0.65)[trial % 3]
        print(
            f"Trial {trial + 1}/{trials}: motor scale={motor_scale:.2f}, "
            f"traction scale={friction_scale:.2f}",
            flush=True,
        )
        sim = curve_sim.MMSAdvancedSimulator(
            seed=trial, motor_scale=motor_scale, friction_scale=friction_scale)

        maze_files = sim.maze_list[:maze_limit]
        for maze_index, maze_file in enumerate(maze_files):
            sim.current_maze_idx = maze_index
            sim.load_active_maze()
            print(f"[{maze_index + 1}/{len(maze_files)}] {maze_file}:", flush=True)
            for phase in ("EXPLORATION", "RETURN", "SPEEDRUN"):
                result = run_phase(sim, phase, max_sim_seconds=600.0)
                passed = result.startswith("PASS")
                all_pass &= passed
                print(f"  {phase}: {result}", flush=True)

        curve_sim.pygame.quit()

    print(
        "RESULT: " + ("ALL PHYSICS TRIALS PASSED" if all_pass else "FAILURES DETECTED"),
        flush=True,
    )
    return all_pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trials", type=int, default=1,
        help="repeat the selected mazes with deterministic motor/traction variations",
    )
    parser.add_argument(
        "--mazes", type=int, default=10,
        help="verify the first N mazes (1-10)",
    )
    args = parser.parse_args()
    if args.trials < 1:
        parser.error("--trials must be at least 1")
    if args.mazes < 1 or args.mazes > 10:
        parser.error("--mazes must be between 1 and 10")
    raise SystemExit(0 if verify_all_mazes(args.trials, args.mazes) else 1)
