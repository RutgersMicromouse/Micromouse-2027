import os
import sys
import math

# Set headless environment for Pygame before import
os.environ["SDL_VIDEODRIVER"] = "dummy"

import curve_sim

def test_all_mazes():
    print("Testing MMSAdvancedSimulator across all mazes...", flush=True)
    sim = curve_sim.MMSAdvancedSimulator()
    
    results = {}
    for idx, maze_file in enumerate(sim.maze_list):
        sim.current_maze_idx = idx
        sim.load_active_maze()
        print(f"[{idx+1}/{len(sim.maze_list)}] {maze_file}:", flush=True)
        
        maze_res = {}
        for phase in ["EXPLORATION", "RETURN", "SPEEDRUN"]:
            sim.set_phase(phase, keep_running=True)
            dt = 1.0 / 60.0
            n_sub = 4
            sub_dt = dt / n_sub
            max_steps = max(30 * 60 * n_sub, int(len(sim.traj_points) * 1.30))
            step_count = 0
            
            while sim.running and not sim.crashed and not sim.finished and step_count < max_steps:
                step_count += 1
                if sim.traj_idx >= len(sim.traj_points) - 15:
                    sim.finished = True
                    sim.running = False
                    break
                
                best_d = float('inf')
                best_i = sim.traj_idx
                search_end = min(len(sim.traj_points), sim.traj_idx + 80)
                for i in range(sim.traj_idx, search_end):
                    dx = sim.traj_points[i].x - sim.bot.x
                    dy = sim.traj_points[i].y - sim.bot.y
                    dth = curve_sim.normalize_angle(sim.traj_points[i].theta - sim.bot.theta)
                    d = dx * dx + dy * dy + (35.0 * dth) ** 2
                    if d < best_d:
                        best_d = d
                        best_i = i
                sim.traj_idx = max(sim.traj_idx, best_i)
                
                lookahead_idx = min(len(sim.traj_points) - 1, sim.traj_idx + 6)
                target_pt = sim.traj_points[lookahead_idx]
                
                sim.bot.read_sensors(sim.collider.wall_segs, sim.collider.posts)
                sim.bot.step_physics(sub_dt, target_pt)
                
                has_col, hit_name, hit_pt = sim.collider.check_collision(sim.bot.x, sim.bot.y, sim.bot.theta)
                if has_col:
                    sim.crashed = True
                    sim.running = False
                    maze_res[phase] = f"CRASH: Hit {hit_name} at ({hit_pt[0]:.1f}, {hit_pt[1]:.1f})! traj_idx={sim.traj_idx}/{len(sim.traj_points)}"
                    break
            
            if not sim.crashed:
                if sim.finished:
                    maze_res[phase] = f"PASS ({step_count*sub_dt:.2f}s)"
                else:
                    maze_res[phase] = f"TIMEOUT at traj_idx={sim.traj_idx}/{len(sim.traj_points)}"
            print(f"  {phase}: {maze_res[phase]}", flush=True)
        
        results[maze_file] = maze_res

    print("\n" + "="*60, flush=True)
    print("CHAMPIONSHIP TOURNAMENT VERIFICATION SUMMARY", flush=True)
    print("="*60, flush=True)
    all_pass = True
    for maze, res in results.items():
        m_pass = all("PASS" in v for v in res.values())
        if not m_pass:
            all_pass = False
        print(f"[{'PASS' if m_pass else 'FAIL'}] {maze:25s} | Exp: {res.get('EXPLORATION','')} | Ret: {res.get('RETURN','')} | Run: {res.get('SPEEDRUN','')}", flush=True)
    print("="*60, flush=True)
    print(f"OVERALL RESULT: {'[SUCCESS] 100% COLLISION-FREE ALL PHASES PASS!' if all_pass else '[FAIL] SOME PHASES FAILED'}", flush=True)

if __name__ == "__main__":
    test_all_mazes()


