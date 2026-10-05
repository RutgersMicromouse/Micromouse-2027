"""
==============================================================================
ANTIGRAVITIEEE HIGH-SPEED CONTINUOUS CURVE SIMULATOR (CHAMPIONSHIP EDITION)
==============================================================================
Features:
- Windows Snap & Auto Split Screen Compatible (pygame.RESIZABLE + VIDEORESIZE)
- Windows High-DPI Awareness (100% Crisp Vector Anti-Aliased Typography)
- Multiple Mazes Selector: example5, classic, speedway, japan_finals
- Full Competition State Machine: EXPLORATION -> RETURN -> CHAMPIONSHIP SPEEDRUN
- Stanley Lookahead Path Tracking & Differential Kinematics (v, w, vL, vR)
- In-Place Cell-Center Pivots on Turnarounds (37.5mm safety clearance)
- Zero-Crash Goal Arrival & Smooth Motor Braking
- Dual View Split Screen (16x16 Maze Canvas + Zoomed 3.6x Live IR Chase Cam)
==============================================================================
"""

import sys, os, math, time
import random
import ctypes
import pygame
import heapq

# ------------------------------------------------------------------------------
# 0. WINDOWS HIGH-DPI AWARENESS (ELIMINATES BLUR ON 1080P/1440P/4K)
# ------------------------------------------------------------------------------
try:
    ctypes.windll.shcore.SetProcessDpiAwareness(2) # Per-monitor DPI aware
except Exception:
    try:
        ctypes.windll.user32.SetProcessDPIAware()
    except Exception:
        pass

# Directions
DIR_NORTH = 0
DIR_EAST  = 1
DIR_SOUTH = 2
DIR_WEST  = 3
DIR_DELTAS = [(0, 1), (1, 0), (0, -1), (-1, 0)]
DIR_CHARS  = ['n', 'e', 's', 'w']

CELL_SIZE_MM = 180.0
HALF_CELL_MM = 90.0
CONTROL_DT_S = 1.0 / 500.0
N20_NO_LOAD_SPEED_MM_S = 700.0
N20_SPEEDRUN_ACCEL_MM_S2 = 2600.0
ROBOT_HALF_WIDTH_MM = 34.0
ROBOT_HALF_LENGTH_MM = 40.0
WALL_HALF_THICKNESS_MM = 3.0
COLLISION_CLEARANCE_MM = 2.0
POST_RADIUS_MM = 6.0

def normalize_angle(a):
    while a > math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a

def ccw(A, B, C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

def segments_intersect(A, B, C, D):
    return (ccw(A, C, D) != ccw(B, C, D)) and (ccw(A, B, C) != ccw(A, B, D))

def dist_pt_to_seg(P, A, B):
    vx, vy = B[0] - A[0], B[1] - A[1]
    L2 = vx * vx + vy * vy
    if L2 == 0: return math.hypot(P[0] - A[0], P[1] - A[1])
    t = max(0.0, min(1.0, ((P[0] - A[0]) * vx + (P[1] - A[1]) * vy) / L2))
    return math.hypot(P[0] - (A[0] + t * vx), P[1] - (A[1] + t * vy))

def ray_segment_intersect(ray_origin, ray_dir, p1, p2, max_dist=250.0):
    ox, oy = ray_origin
    dx, dy = ray_dir
    x1, y1 = p1
    x2, y2 = p2
    sx, sy = x2 - x1, y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-6:
        return None
    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
    if 0.0 <= t <= max_dist and 0.0 <= u <= 1.0:
        seg_len = math.hypot(sx, sy)
        if seg_len > 1e-6:
            nx, ny = -sy / seg_len, sx / seg_len
            cos_alpha = abs(dx * nx + dy * ny)
        else:
            cos_alpha = 1.0
        return t, (ox + t * dx, oy + t * dy), cos_alpha
    return None

def ray_post_intersect(ray_origin, ray_dir, px, py, r_post=6.0, max_dist=250.0):
    """Ray-circle intersection against micromouse cell corner wooden posts."""
    ox, oy = ray_origin
    dx, dy = ray_dir
    vx = px - ox
    vy = py - oy
    t_proj = vx * dx + vy * dy
    if t_proj < 0.0 or t_proj > max_dist + r_post:
        return None
    d2 = (vx * vx + vy * vy) - t_proj * t_proj
    if d2 < r_post * r_post:
        dt = math.sqrt(max(0.0, r_post * r_post - d2))
        t = t_proj - dt
        if 0.0 <= t <= max_dist:
            hit_x = ox + t * dx
            hit_y = oy + t * dy
            nx = (hit_x - px) / r_post
            ny = (hit_y - py) / r_post
            cos_alpha = abs(dx * (-nx) + dy * (-ny))
            return t, (hit_x, hit_y), cos_alpha
    return None

# ==============================================================================
# 1. MAZE MODEL & NAVIGATION ALGORITHMS
# ==============================================================================
class Maze:
    def __init__(self, width=16, height=16):
        self.width = width
        self.height = height
        self.walls = [[[False]*4 for _ in range(height)] for _ in range(width)]
        self.visited = [[False]*height for _ in range(width)]
        self.distances = [[-1]*height for _ in range(width)]
        for x in range(width):
            self.walls[x][0][2] = True
            self.walls[x][height-1][0] = True
        for y in range(height):
            self.walls[0][y][3] = True
            self.walls[width-1][y][1] = True

    def load_num(self, filename):
        if not os.path.exists(filename):
            return False
        self.walls = [[[False]*4 for _ in range(self.height)] for _ in range(self.width)]
        for x in range(self.width):
            self.walls[x][0][2] = True
            self.walls[x][self.height-1][0] = True
        for y in range(self.height):
            self.walls[0][y][3] = True
            self.walls[self.width-1][y][1] = True

        with open(filename, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 6:
                    x, y = int(parts[0]), int(parts[1])
                    if 0 <= x < self.width and 0 <= y < self.height:
                        self.walls[x][y][0] = bool(int(parts[2]))
                        self.walls[x][y][1] = bool(int(parts[3]))
                        self.walls[x][y][2] = bool(int(parts[4]))
                        self.walls[x][y][3] = bool(int(parts[5]))
        return True

    def has_wall(self, x, y, d):
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.walls[x][y][d]
        return True

    def set_wall(self, x, y, d, val=True):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.walls[x][y][d] = val
            dx, dy = DIR_DELTAS[d]
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                self.walls[nx][ny][(d + 2) % 4] = val

    def compute_floodfill_distances(self, goals):
        for x in range(self.width):
            for y in range(self.height):
                self.distances[x][y] = -1
        queue = []
        for gx, gy in goals:
            self.distances[gx][gy] = 0
            queue.append((gx, gy))
        while queue:
            cx, cy = queue.pop(0)
            d_cur = self.distances[cx][cy]
            for d in range(4):
                if not self.has_wall(cx, cy, d):
                    dx, dy = DIR_DELTAS[d]
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        if self.distances[nx][ny] == -1:
                            self.distances[nx][ny] = d_cur + 1
                            queue.append((nx, ny))

def dijkstra_fastest_path(maze, start_x, start_y, start_h, goals):
    width, height = maze.width, maze.height
    pq = [(0.0, start_x, start_y, start_h)]
    best_cost = {}
    parent = {}
    best_goal_state = None

    while pq:
        cost, x, y, h = heapq.heappop(pq)
        state = (x, y, h)
        if state in best_cost and best_cost[state] <= cost:
            continue
        best_cost[state] = cost

        if (x, y) in goals:
            best_goal_state = state
            break

        for td in [1, 3]:
            nh = (h + td) % 4
            tc = cost + 1.2
            nstate = (x, y, nh)
            if nstate not in best_cost or best_cost[nstate] > tc:
                heapq.heappush(pq, (tc, x, y, nh))
                if nstate not in parent or tc < best_cost.get(nstate, float('inf')):
                    parent[nstate] = ((x, y, h), [])

        dx, dy = DIR_DELTAS[h]
        for L in range(1, 16):
            px = x + dx * (L - 1)
            py = y + dy * (L - 1)
            if maze.has_wall(px, py, h): break
            nx = x + dx * L
            ny = y + dy * L
            if not (0 <= nx < width and 0 <= ny < height): break
            if not maze.visited[nx][ny]: break

            sprint_cost = cost + 1.0 + 0.35 * (L - 1)
            nstate = (nx, ny, h)
            if nstate not in best_cost or best_cost[nstate] > sprint_cost:
                heapq.heappush(pq, (sprint_cost, nx, ny, h))
                seg = [(x + dx * s, y + dy * s) for s in range(1, L + 1)]
                parent[nstate] = ((x, y, h), seg)

    if best_goal_state is None:
        return []

    full_path = []
    curr = best_goal_state
    while curr in parent:
        prev_state, seg = parent[curr]
        full_path = seg + full_path
        curr = prev_state
    return [(start_x, start_y)] + full_path

def generate_floodfill_exploration_path(maze, goals):
    """Simulates micromouse exploration from start to center goal."""
    explored_walls = [[[False]*4 for _ in range(16)] for _ in range(16)]
    for x in range(16):
        explored_walls[x][0][2] = True
        explored_walls[x][15][0] = True
    for y in range(16):
        explored_walls[0][y][3] = True
        explored_walls[15][y][1] = True
    explored_walls[0][0][1] = True
    explored_walls[1][0][3] = True

    visited = [[False]*16 for _ in range(16)]
    cur_x, cur_y = 0, 0
    cur_h = DIR_NORTH
    path = [(0, 0)]
    steps = 0

    while (cur_x, cur_y) not in goals and steps < 400:
        for d in range(4):
            if maze.has_wall(cur_x, cur_y, d):
                explored_walls[cur_x][cur_y][d] = True
                dx, dy = DIR_DELTAS[d]
                nx, ny = cur_x + dx, cur_y + dy
                if 0 <= nx < 16 and 0 <= ny < 16:
                    explored_walls[nx][ny][(d+2)%4] = True
        visited[cur_x][cur_y] = True

        dist = [[-1]*16 for _ in range(16)]
        q = list(goals)
        for gx, gy in q: dist[gx][gy] = 0
        while q:
            cx, cy = q.pop(0)
            cd = dist[cx][cy]
            for d in range(4):
                if not explored_walls[cx][cy][d]:
                    dx, dy = DIR_DELTAS[d]
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < 16 and 0 <= ny < 16 and dist[nx][ny] == -1:
                        dist[nx][ny] = cd + 1
                        q.append((nx, ny))

        best_d = 999
        best_n = None
        for d in [cur_h, (cur_h+3)%4, (cur_h+1)%4, (cur_h+2)%4]:
            if not explored_walls[cur_x][cur_y][d]:
                dx, dy = DIR_DELTAS[d]
                nx, ny = cur_x + dx, cur_y + dy
                if 0 <= nx < 16 and 0 <= ny < 16 and dist[nx][ny] != -1:
                    if dist[nx][ny] < best_d:
                        best_d = dist[nx][ny]
                        best_n = (nx, ny, d)
        if best_n is None: break
        cur_x, cur_y, cur_h = best_n
        path.append((cur_x, cur_y))
        steps += 1

    return path

# ==============================================================================
# 2. CONTINUOUS TANGENT-ARC TRAJECTORY GENERATOR WITH CELL-CENTER PIVOTS
# ==============================================================================
class TrajectoryPoint:
    __slots__ = ['x', 'y', 'theta', 'v', 'omega', 'g_force', 'desc', 'is_curve', 'radius']
    def __init__(self, x, y, theta, v, omega, g_force=0.0, desc="", is_curve=False, radius=0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.omega = omega
        self.g_force = g_force
        self.desc = desc
        self.is_curve = is_curve
        self.radius = radius

def extract_safe_waypoints(path, maze, allow_diagonals=True):
    if len(path) < 2:
        return [((path[0][0] + 0.5) * 180.0, (path[0][1] + 0.5) * 180.0)], [False]

    dirs = []
    for i in range(len(path) - 1):
        dx = path[i+1][0] - path[i][0]
        dy = path[i+1][1] - path[i][1]
        dirs.append(DIR_DELTAS.index((dx, dy)))

    n = len(dirs)
    waypoints = [((path[0][0] + 0.5) * 180.0, (path[0][1] + 0.5) * 180.0)]
    wp_is_diag = [False]
    i = 0

    while i < n:
        diag_k = 0
        p_start, p_end = None, None
        if allow_diagonals and i + 1 < n:
            d_a = dirs[i]
            d_b = dirs[i+1]
            diff = (d_b - d_a) % 4
            if diff in [1, 3]:
                k = 2
                while i + k < n:
                    exp = d_a if k % 2 == 0 else d_b
                    if dirs[i+k] == exp: k += 1
                    else: break
                while k >= 3:
                    c0 = path[i]
                    dxA, dyA = DIR_DELTAS[d_a]
                    pst = ((c0[0] + 0.5 + 0.5 * dxA) * 180.0, (c0[1] + 0.5 + 0.5 * dyA) * 180.0)
                    
                    d_last = dirs[i + k - 1]
                    dxLast, dyLast = DIR_DELTAS[d_last]
                    c_lp = path[i + k - 1]
                    ped = ((c_lp[0] + 0.5 + 0.5 * dxLast) * 180.0, (c_lp[1] + 0.5 + 0.5 * dyLast) * 180.0)

                    # Post clearance test (robot half width 34mm + post 6mm = 40mm, safety margin 46mm)
                    safe = True
                    for px in range(17):
                        for py in range(17):
                            if dist_pt_to_seg((px * 180.0, py * 180.0), pst, ped) < 46.0:
                                safe = False; break
                        if not safe: break

                    # Wall collision & clearance test
                    if safe:
                        for x in range(16):
                            for y in range(16):
                                x0, x1 = x * 180.0, (x + 1) * 180.0
                                y0, y1 = y * 180.0, (y + 1) * 180.0
                                wls = []
                                if maze.has_wall(x, y, 0): wls.append(((x0, y1), (x1, y1)))
                                if maze.has_wall(x, y, 1): wls.append(((x1, y0), (x1, y1)))
                                if maze.has_wall(x, y, 2): wls.append(((x0, y0), (x1, y0)))
                                if maze.has_wall(x, y, 3): wls.append(((x0, y0), (x0, y1)))
                                for w1, w2 in wls:
                                    if segments_intersect(pst, ped, w1, w2):
                                        safe = False; break
                                    if (dist_pt_to_seg(w1, pst, ped) < 46.0 or
                                        dist_pt_to_seg(w2, pst, ped) < 46.0 or
                                        dist_pt_to_seg(pst, w1, w2) < 46.0 or
                                        dist_pt_to_seg(ped, w1, w2) < 46.0):
                                        safe = False; break
                                if not safe: break

                    if safe:
                        diag_k = k
                        p_start, p_end = pst, ped
                        break
                    k -= 1

        if diag_k >= 3:
            c_entry = path[i]
            entry_pt = ((c_entry[0] + 0.5) * 180.0, (c_entry[1] + 0.5) * 180.0)
            if math.hypot(entry_pt[0] - waypoints[-1][0], entry_pt[1] - waypoints[-1][1]) > 10.0:
                waypoints.append(entry_pt)
                wp_is_diag.append(False)
            
            waypoints.append(p_start)
            wp_is_diag.append(True)
            waypoints.append(p_end)
            wp_is_diag.append(False)
            
            c_exit = path[i + diag_k]
            exit_pt = ((c_exit[0] + 0.5) * 180.0, (c_exit[1] + 0.5) * 180.0)
            waypoints.append(exit_pt)
            wp_is_diag.append(False)
            
            i += diag_k
        else:
            d = dirs[i]
            cnt = 1
            while i + cnt < n and dirs[i + cnt] == d:
                cnt += 1
            target_cell = path[i + cnt]
            waypoints.append(((target_cell[0] + 0.5) * 180.0, (target_cell[1] + 0.5) * 180.0))
            wp_is_diag.append(False)
            i += cnt

    return waypoints, wp_is_diag

def calculate_trajectory_metrics(traj_points):
    if not traj_points:
        return {'time': 999.0, 'distance': 0.0, 'avg_speed': 0.0, 'curves': 0, 'diagonals': 0}
    total_dist = len(traj_points) * 2.0
    total_time = sum(2.0 / max(50.0, pt.v) for pt in traj_points)
    num_curves = sum(1 for pt in traj_points if pt.is_curve)
    num_diags = sum(1 for pt in traj_points if "Diagonal" in pt.desc)
    avg_speed = total_dist / max(0.01, total_time)
    return {
        'time': total_time,
        'distance': total_dist,
        'avg_speed': avg_speed,
        'curves': num_curves,
        'diagonals': num_diags
    }

def generate_curve_trajectory(waypoints, wp_is_diag, nominal_R=55.0, ds=2.0,
                              v_straight=500.0, v_diag=550.0, v_curve_max=350.0,
                              acceleration=2600.0, deceleration=2600.0):
    N = len(waypoints)
    if N < 2: return []

    v_list = []
    L_list = []
    u_list = []
    for i in range(N - 1):
        vx = waypoints[i+1][0] - waypoints[i][0]
        vy = waypoints[i+1][1] - waypoints[i][1]
        L = math.hypot(vx, vy)
        v_list.append((vx, vy))
        L_list.append(L)
        u_list.append((vx / L, vy / L) if L > 1e-6 else (1.0, 0.0))

    corners = []
    for i in range(1, N - 1):
        u_in = u_list[i - 1]
        u_out = u_list[i]
        ang_in = math.atan2(u_in[1], u_in[0])
        ang_out = math.atan2(u_out[1], u_out[0])
        d_theta = normalize_angle(ang_out - ang_in)

        if abs(d_theta) < 1e-3:
            corners.append(None)
            continue

        turn_left = d_theta > 0
        ang_deg = abs(int(round(math.degrees(d_theta))))

        # Sharp turn or 180° turnaround (e.g. dead end in corridor):
        # Rotate in place at exact cell center with 37.5mm clearance on all sides!
        if abs(d_theta) > math.radians(105):
            Wi = waypoints[i]
            corners.append({
                'type': 'pivot',
                'Tin': Wi, 'Tout': Wi, 'center': Wi,
                'ang_in': ang_in, 'ang_out': ang_out,
                'sweep': d_theta, 'turn_left': turn_left,
                'ang_deg': ang_deg
            })
        else:
            # Continuous circular arc
            capped_turn = abs(d_theta)
            tan_half = math.tan(capped_turn / 2.0)
            d_nom = nominal_R * tan_half
            d_max = 0.40 * min(L_list[i - 1], L_list[i])
            d = min(d_nom, d_max)
            R_eff = d / tan_half

            Wi = waypoints[i]
            Tin = (Wi[0] - d * u_in[0], Wi[1] - d * u_in[1])
            Tout = (Wi[0] + d * u_out[0], Wi[1] + d * u_out[1])

            nin = (-u_in[1], u_in[0]) if turn_left else (u_in[1], -u_in[0])
            center = (Tin[0] + R_eff * nin[0], Tin[1] + R_eff * nin[1])
            start_phi = math.atan2(Tin[1] - center[1], Tin[0] - center[0])

            corners.append({
                'type': 'arc',
                'Tin': Tin, 'Tout': Tout, 'center': center,
                'R': R_eff, 'start_phi': start_phi, 'sweep': d_theta,
                'turn_left': turn_left, 'ang_deg': ang_deg
            })

    points = []
    cur_pos = waypoints[0]

    for i in range(len(corners)):
        c = corners[i]
        is_diag = wp_is_diag[i]
        cruise_speed = v_diag if is_diag else v_straight
        desc_line = "Diagonal Glide" if is_diag else "Straight Corridor"

        if c is None:
            next_pt = waypoints[i + 1]
            dist = math.hypot(next_pt[0] - cur_pos[0], next_pt[1] - cur_pos[1])
            steps = max(1, int(dist / ds))
            u_line = u_list[i]
            theta_line = math.atan2(u_line[1], u_line[0])
            for s in range(1, steps + 1):
                f = s / steps
                px = cur_pos[0] + f * (next_pt[0] - cur_pos[0])
                py = cur_pos[1] + f * (next_pt[1] - cur_pos[1])
                points.append(TrajectoryPoint(px, py, theta_line, cruise_speed, 0.0, 0.0, desc_line, False, 0.0))
            cur_pos = next_pt
            continue

        Tin = c['Tin']
        dist = math.hypot(Tin[0] - cur_pos[0], Tin[1] - cur_pos[1])
        steps = max(1, int(dist / ds))
        u_line = u_list[i]
        theta_line = math.atan2(u_line[1], u_line[0])
        for s in range(1, steps + 1):
            f = s / steps
            px = cur_pos[0] + f * (Tin[0] - cur_pos[0])
            py = cur_pos[1] + f * (Tin[1] - cur_pos[1])
            points.append(TrajectoryPoint(px, py, theta_line, cruise_speed, 0.0, 0.0, desc_line, False, 0.0))

        if c['type'] == 'pivot':
            pivot_steps = max(20, int(abs(c['sweep']) / 0.03))
            for s in range(1, pivot_steps + 1):
                f = s / pivot_steps
                th = normalize_angle(c['ang_in'] + f * c['sweep'])
                points.append(TrajectoryPoint(c['center'][0], c['center'][1], th, 0.0, 5.0 if c['turn_left'] else -5.0, 0.0, f"In-Place {c['ang_deg']}° Pivot", False, 0.0))
        else:
            arc_len = abs(c['sweep'] * c['R'])
            arc_steps = max(3, int(arc_len / ds))
            v_grip = math.sqrt(0.55 * 9810.0 * c['R'])
            v_alpha = c['R'] * math.sqrt(0.35 * (2.0 * N20_SPEEDRUN_ACCEL_MM_S2 / 72.0) * abs(c['sweep']))
            v_curve = min(v_curve_max, v_grip, v_alpha)
            g_force = (v_curve * v_curve) / (c['R'] * 9810.0)
            desc_curve = f"Smooth {c['ang_deg']}° {'Left' if c['turn_left'] else 'Right'} Arc"
            direction_sign = 1.0 if c['turn_left'] else -1.0
            omega_nominal = (v_curve / c['R']) * direction_sign

            for s in range(1, arc_steps + 1):
                f = s / arc_steps
                phi = c['start_phi'] + f * c['sweep']
                px = c['center'][0] + c['R'] * math.cos(phi)
                py = c['center'][1] + c['R'] * math.sin(phi)
                theta_arc = normalize_angle(phi + (math.pi / 2.0 if c['turn_left'] else -math.pi / 2.0))

                # Smooth trapezoidal yaw rate ramp (0 -> 1 over first 25%, 1 -> 0 over last 25%)
                if f < 0.25:
                    ramp = f / 0.25
                elif f > 0.75:
                    ramp = (1.0 - f) / 0.25
                else:
                    ramp = 1.0
                pt_omega = omega_nominal * ramp

                points.append(TrajectoryPoint(px, py, theta_arc, v_curve, pt_omega, g_force, desc_curve, True, c['R']))


        cur_pos = c['Tout']

    last_pt = waypoints[-1]
    is_last_diag = wp_is_diag[-1]
    cruise_speed = v_diag if is_last_diag else v_straight
    desc_last = "Center Arrival" if is_last_diag else "Straight Center Sprint"
    dist = math.hypot(last_pt[0] - cur_pos[0], last_pt[1] - cur_pos[1])
    steps = max(1, int(dist / ds))
    u_final = u_list[-1]
    theta_final = math.atan2(u_final[1], u_final[0])
    for s in range(1, steps + 1):
        f = s / steps
        px = cur_pos[0] + f * (last_pt[0] - cur_pos[0])
        py = cur_pos[1] + f * (last_pt[1] - cur_pos[1])
        points.append(TrajectoryPoint(px, py, theta_final, cruise_speed, 0.0, 0.0, desc_last, False, 0.0))

    M = len(points)
    if M == 0:
        return []

    # ==========================================================================
    # TWO-PASS TRAPEZOIDAL VELOCITY PLANNING (ACTUATOR BRAKING COMPLIANCE)
    # ==========================================================================
    a_accel_plan = min(acceleration, N20_SPEEDRUN_ACCEL_MM_S2)
    a_decel_plan = min(deceleration, N20_SPEEDRUN_ACCEL_MM_S2)

    # 1. Boundary conditions: start and end from rest
    points[0].v = 0.0
    points[-1].v = 0.0

    # 2. Backward pass: Deceleration ramps before curves, pivots, and goal
    for j in range(M - 2, -1, -1):
        pj = points[j]
        pj1 = points[j + 1]
        step_dist = math.hypot(pj1.x - pj.x, pj1.y - pj.y)
        if step_dist > 1e-4:
            v_max_allowable = math.sqrt(pj1.v * pj1.v + 2.0 * a_decel_plan * step_dist)
            if pj.v > v_max_allowable:
                pj.v = v_max_allowable
        else:
            pj.v = min(pj.v, pj1.v)

    # 3. Forward pass: Acceleration ramps out of curves, pivots, and start
    for j in range(1, M):
        pj_prev = points[j - 1]
        pj = points[j]
        step_dist = math.hypot(pj.x - pj_prev.x, pj.y - pj_prev.y)
        if step_dist > 1e-4:
            v_max_allowable = math.sqrt(pj_prev.v * pj_prev.v + 2.0 * a_accel_plan * step_dist)
            if pj.v > v_max_allowable:
                pj.v = v_max_allowable
        else:
            pj.v = min(pj.v, pj_prev.v)

    return points

# ==============================================================================
# 3. Approximate N20 Robot Model (requires hardware calibration)
# ==============================================================================
class SensorHit:
    """
    Simulates real-world 12-bit ADC optical reflection signal with:
    - Lambertian incident angle cosine attenuation
    - Non-linear distance inverse-square response
    - Ambient optical shot/thermal noise & ADC quantization
    - Corner post edge hit detection (pillar glitch)
    - Intermittent optical glitches/dropouts (dust/specular flare)
    """
    def __init__(self, dist, hit_pt, adc, raw_dist, cos_alpha=1.0, is_post=False, glitch=False):
        self.dist = dist           # Sensed distance in mm (with non-linearity & noise)
        self.hit_pt = hit_pt       # World coords (wx, wy)
        self.adc = adc             # 12-bit ADC reading (0 - 4095)
        self.raw_dist = raw_dist   # True geometric distance in mm
        self.cos_alpha = cos_alpha # Lambertian incidence angle cosine
        self.is_post = is_post     # True if ray hit a corner post
        self.glitch = glitch       # True if optical drop/spike active

    def __getitem__(self, idx):
        if idx == 0: return self.dist
        elif idx == 1: return self.hit_pt
        elif idx == 2: return self.adc
        elif idx == 3: return self.raw_dist
        raise IndexError("SensorHit index out of range")

class PhysicalMicromouse:
    def __init__(self, x=90.0, y=90.0, theta=math.pi/2, seed=0,
                 motor_scale=1.0, friction_scale=1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.W = 72.0
        self.half_w = ROBOT_HALF_WIDTH_MM
        self.half_l = ROBOT_HALF_LENGTH_MM

        self.v_left = 0.0
        self.v_right = 0.0
        self.wall_centering_enabled = True
        self.drift_bias = 0.0
        self.i_cross = 0.0   # Integral cross-track bias accumulator

        self.rng = random.Random(seed)
        self.motor_scale = max(0.5, min(1.0, motor_scale))
        self.friction_scale = max(0.35, min(1.0, friction_scale))
        self.max_wheel_speed = N20_NO_LOAD_SPEED_MM_S * self.motor_scale
        self.a_accel = N20_SPEEDRUN_ACCEL_MM_S2 * self.motor_scale
        self.a_brake = N20_SPEEDRUN_ACCEL_MM_S2 * self.motor_scale
        self.tau_m = 0.025
        self.mu = 0.55 * self.friction_scale
        self.motor_left_gain = self.rng.uniform(0.975, 1.0)
        self.motor_right_gain = self.rng.uniform(0.975, 1.0)

        # Realistic hardware & signal flags
        self.optical_noise_enabled = True
        self.motor_imbalance_enabled = True
        self.glitches_enabled = True
        self.motor_imbalance_pct = 0.012 # 1.2% realistic motor/gear friction variance

        # Dynamic state & telemetry metrics
        self.centering_mode = "DUAL-WALL"
        self.slip_ratio = 0.0
        self.gyro_drift = 0.0
        self.gyro_drift_rate = math.radians(0.08) # 0.08 deg/s zero-rate bias
        self.last_sync_cell = (0, 0)
        self.sync_active = False
        self.active_glitches = 0

        # 6 Sensor channels: L90, L45, FL, FR, R45, R90 (matching schematic rev 1.0 & config.h)
        dummy_hit = SensorHit(90.0, None, 450, 90.0, 1.0, False, False)
        self.sensor_l90 = dummy_hit
        self.sensor_l45 = dummy_hit
        self.sensor_fl  = dummy_hit
        self.sensor_fr  = dummy_hit
        self.sensor_r45 = dummy_hit
        self.sensor_r90 = dummy_hit

        # Backward compatibility aliases
        self.sensor_left  = self.sensor_l90
        self.sensor_right = self.sensor_r90

        # History for post falling-edge detection
        self.prev_l90_adc = 450
        self.prev_r90_adc = 450
        self.wall_left = False
        self.wall_front = False
        self.wall_right = False
        self.opening_left = False
        self.opening_right = False
        self.centering_error = 0.0

    def read_sensors(self, wall_segs, posts, spatial_index=None):
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)

        # Exact physical locations of the 6 optical channels on robot chassis:
        # L90: left flank (x=0, y=+34) -> ray pointing at theta + 90 deg
        pos_L90 = (self.x - self.half_w * sin_t, self.y + self.half_w * cos_t)
        dir_L90 = (-sin_t, cos_t)

        # L45: front-left diagonal (x=+36, y=+25) -> ray pointing at theta + 45 deg
        pos_L45 = (self.x + 36.0 * cos_t - 25.0 * sin_t, self.y + 36.0 * sin_t + 25.0 * cos_t)
        th_l45 = self.theta + math.pi / 4.0
        dir_L45 = (math.cos(th_l45), math.sin(th_l45))

        # FL: front-left center 0° (x=+40, y=+12) -> ray pointing at theta (0 deg)
        pos_FL = (self.x + self.half_l * cos_t - 12.0 * sin_t, self.y + self.half_l * sin_t + 12.0 * cos_t)
        dir_FL = (cos_t, sin_t)

        # FR: front-right center 0° (x=+40, y=-12) -> ray pointing at theta (0 deg)
        pos_FR = (self.x + self.half_l * cos_t + 12.0 * sin_t, self.y + self.half_l * sin_t - 12.0 * cos_t)
        dir_FR = (cos_t, sin_t)

        # R45: front-right diagonal (x=+36, y=-25) -> ray pointing at theta - 45 deg
        pos_R45 = (self.x + 36.0 * cos_t + 25.0 * sin_t, self.y + 36.0 * sin_t - 25.0 * cos_t)
        th_r45 = self.theta - math.pi / 4.0
        dir_R45 = (math.cos(th_r45), math.sin(th_r45))

        # R90: right flank (x=0, y=-34) -> ray pointing at theta - 90 deg
        pos_R90 = (self.x + self.half_w * sin_t, self.y - self.half_w * cos_t)
        dir_R90 = (sin_t, -cos_t)

        def cast_ray(orig, rdir, max_range=240.0):
            closest_dist = max_range
            closest_pt = None
            closest_cos = 1.0
            is_post_hit = False

            # 1. Test against walls
            candidate_walls = (
                spatial_index.wall_candidates_for_ray(orig, max_range)
                if spatial_index is not None else wall_segs
            )
            for w1, w2, _ in candidate_walls:
                min_x = min(w1[0], w2[0]) - 5.0
                max_x = max(w1[0], w2[0]) + 5.0
                min_y = min(w1[1], w2[1]) - 5.0
                max_y = max(w1[1], w2[1]) + 5.0
                if (orig[0] < min_x - max_range or orig[0] > max_x + max_range or
                    orig[1] < min_y - max_range or orig[1] > max_y + max_range):
                    continue

                res = ray_segment_intersect(orig, rdir, w1, w2, max_range)
                if res and res[0] < closest_dist:
                    closest_dist = res[0]
                    closest_pt = res[1]
                    closest_cos = res[2]
                    is_post_hit = False

            # 2. Test against posts (12mm diameter posts at cell corners)
            candidate_posts = (
                spatial_index.post_candidates_for_ray(orig, max_range)
                if spatial_index is not None else posts
            )
            for px, py in candidate_posts:
                if abs(orig[0] - px) > max_range + 8.0 or abs(orig[1] - py) > max_range + 8.0:
                    continue
                pres = ray_post_intersect(orig, rdir, px, py, r_post=6.0, max_dist=max_range)
                if pres and pres[0] < closest_dist:
                    closest_dist = pres[0]
                    closest_pt = pres[1]
                    closest_cos = pres[2]
                    is_post_hit = True

            # 3. Realistic Phototransistor & ADC Modeling
            I0 = 820.0
            if closest_dist < max_range:
                # Lambertian reflection with distance inverse-square
                attenuated_cos = max(0.12, closest_cos)
                ideal_adc = 40.0 + min(4055.0, (I0 * attenuated_cos) / ((closest_dist / 55.0 + 0.1) ** 2))
            else:
                ideal_adc = 40.0 # Ambient dark noise floor

            # Optical noise & jitter
            sim_adc = ideal_adc
            glitch_active = False
            if self.optical_noise_enabled:
                sigma = 12.0 + 0.015 * ideal_adc
                sim_adc += self.rng.gauss(0.0, sigma)

            # Signal dropouts / glitches (dust, specular reflection flare off tape/screws)
            if self.glitches_enabled and self.rng.random() < 0.003:
                glitch_active = True
                sim_adc = max(30.0, sim_adc * self.rng.uniform(0.5, 0.75))

            sim_adc = max(0, min(4095, int(round(sim_adc))))

            # Invert sensor reading to perceived distance (matching firmware calibrated curve)
            if sim_adc >= 115:
                perceived_dist = 55.0 * (math.sqrt(I0 / max(1.0, float(sim_adc - 35))) - 0.1)
                perceived_dist = max(5.0, min(max_range, perceived_dist))
            else:
                perceived_dist = max_range

            return SensorHit(perceived_dist, closest_pt, sim_adc, closest_dist, closest_cos, is_post_hit, glitch_active)

        self.sensor_l90 = cast_ray(pos_L90, dir_L90)
        self.sensor_l45 = cast_ray(pos_L45, dir_L45)
        self.sensor_fl  = cast_ray(pos_FL,  dir_FL)
        self.sensor_fr  = cast_ray(pos_FR,  dir_FR)
        self.sensor_r45 = cast_ray(pos_R45, dir_R45)
        self.sensor_r90 = cast_ray(pos_R90, dir_R90)

        # Backward compatibility references
        self.sensor_left  = self.sensor_l90
        self.sensor_right = self.sensor_r90

        # Count active optical glitches
        self.active_glitches = sum(1 for s in [self.sensor_l90, self.sensor_l45, self.sensor_fl, self.sensor_fr, self.sensor_r45, self.sensor_r90] if s.glitch)

        # Pillar post edge detection (falling edge when passing a wall opening)
        post_edge_left = (self.prev_l90_adc > 350 and self.sensor_l90.adc < 280)
        post_edge_right = (self.prev_r90_adc > 350 and self.sensor_r90.adc < 280)
        self.sync_active = False
        if post_edge_left or post_edge_right:
            cx = int(self.x // CELL_SIZE_MM)
            cy = int(self.y // CELL_SIZE_MM)
            self.last_sync_cell = (cx, cy)
            self.sync_active = True

        self.prev_l90_adc = self.sensor_l90.adc
        self.prev_r90_adc = self.sensor_r90.adc
        self.wall_left = self.sensor_l90.adc > 300
        self.wall_right = self.sensor_r90.adc > 300
        self.wall_front = max(self.sensor_fl.adc, self.sensor_fr.adc) > 400
        self.opening_left = self.wall_left and self.sensor_l45.adc < 0.65 * 850
        self.opening_right = self.wall_right and self.sensor_r45.adc < 0.65 * 850

        valid_left = self.wall_left and not self.opening_left and self.sensor_l45.adc > 350
        valid_right = self.wall_right and not self.opening_right and self.sensor_r45.adc > 350
        both_funnel = self.sensor_l45.adc > 350 and self.sensor_r45.adc > 350
        if self.wall_front:
            fl, fr = self.sensor_fl.adc, self.sensor_fr.adc
            average = (fl + fr) * 0.5
            self.centering_error = 1.5 * (fl - fr) / average if average > 100 else 0.0
        elif valid_left and valid_right:
            self.centering_error = ((self.sensor_l45.adc - 850) -
                                    (self.sensor_r45.adc - 850)) / 850.0
        elif both_funnel and not self.wall_left and not self.wall_right:
            self.centering_error = ((self.sensor_l45.adc - 850) -
                                    (self.sensor_r45.adc - 850)) / 850.0
        elif valid_left:
            self.centering_error = 2.0 * (self.sensor_l45.adc - 850) / 850.0
        elif valid_right:
            self.centering_error = -2.0 * (self.sensor_r45.adc - 850) / 850.0
        else:
            self.centering_error = 0.0
        self.centering_error = max(-1.5, min(1.5, self.centering_error))

    def step_physics(self, dt, target_pt, final_point=None):
        """
        Approximate Micromouse dynamics, not a firmware-in-the-loop guarantee:
        - Stanley path tracking + adaptive multi-mode straightaway wall centering
        - Single-wall vs Dual-wall vs Heading-lock vs Front-wall suppression
        - N20 wheel speed, acceleration, and motor response limits
        - Mechanical motor & wheel imbalance with natural straightaway drift
        - Dynamic wheel micro-slip & traction model
        - Gyro zero-rate bias drift
        """
        # 1. Front Wall Proximity Check
        d_front = min(self.sensor_fl.dist, self.sensor_fr.dist)
        is_front_close = (d_front < 110.0 or self.wall_front)

        # 2. Straightaway Wall Centering Controller (Multi-Mode matching C++ firmware)
        sensor_trim = 0.0
        is_curving = abs(target_pt.omega) > 0.22
        is_orthogonal = (abs(math.cos(target_pt.theta)) > 0.92 or abs(math.sin(target_pt.theta)) > 0.92)

        if self.wall_centering_enabled and not is_curving and is_orthogonal:
            if is_front_close:
                # Suppress side trim when approaching front wall (avoids false turning jerks)
                self.centering_mode = "FRONT-SUPPRESS"
                sensor_trim = 0.0
            else:
                has_left = self.wall_left and not self.opening_left and self.sensor_l45.adc > 350
                has_right = self.wall_right and not self.opening_right and self.sensor_r45.adc > 350

                dL = self.sensor_l90.dist if self.sensor_l90.dist < 115.0 else (self.sensor_l45.dist * 0.707)
                dR = self.sensor_r90.dist if self.sensor_r90.dist < 115.0 else (self.sensor_r45.dist * 0.707)
                d_nominal = 50.0 # Nominal centered clearance mm (168mm corridor - 68mm robot)/2

                if has_left and has_right:
                    self.centering_mode = "DUAL-WALL"
                    sensor_trim = 5.0 * math.radians(max(-25.0, min(25.0, -35.0 * self.centering_error)))
                elif has_left:
                    self.centering_mode = "SINGLE-WALL-L"
                    sensor_trim = 5.0 * math.radians(max(-25.0, min(25.0, -35.0 * self.centering_error)))
                elif has_right:
                    self.centering_mode = "SINGLE-WALL-R"
                    sensor_trim = 5.0 * math.radians(max(-25.0, min(25.0, -35.0 * self.centering_error)))
                else:
                    self.centering_mode = "IMU-HEADING-LOCK"
                    sensor_trim = 0.0

                # Anti-windup clamp on steering trim
                sensor_trim = max(-0.35, min(0.35, sensor_trim))
        else:
            if not is_orthogonal:
                self.centering_mode = "DIAGONAL-IMU-LOCK"
            elif is_curving:
                self.centering_mode = "CURVING"
            else:
                self.centering_mode = "DISABLED"
            sensor_trim = 0.0

        # 3. Path Tracking Controller
        is_pivot = (target_pt.v < 1.0 and abs(target_pt.omega) > 0.5)

        if is_pivot:
            # During in-place pivots at cell centers, purely track heading setpoint
            self.centering_mode = "PIVOT"
            sensor_trim = 0.0
            self.i_cross = 0.0
            e_theta = normalize_angle(target_pt.theta - self.theta)
            v_target = 0.0
            omega_target = target_pt.omega + 8.0 * e_theta
        else:
            # Stanley Path Tracking (Cross-Track & Heading Error)
            dx = target_pt.x - self.x
            dy = target_pt.y - self.y
            e_along = dx * math.cos(target_pt.theta) + dy * math.sin(target_pt.theta)
            e_cross = -dx * math.sin(target_pt.theta) + dy * math.cos(target_pt.theta)

            # Cross-track integral term: only accumulate on straights, decay quickly in curves
            if not is_curving:
                self.i_cross = max(-0.30, min(0.30, self.i_cross + e_cross * 0.10 * dt))
            else:
                self.i_cross *= 0.95

            v_clamp = max(80.0, abs(self.v))
            k_stanley = 2.8
            theta_correction = math.atan2(k_stanley * e_cross, v_clamp)

            theta_desired = normalize_angle(target_pt.theta + theta_correction)
            e_theta = normalize_angle(theta_desired - self.theta)

            # 4. Target Velocity & Yaw Rate Setpoints
            v_target = max(0.0, target_pt.v)
            omega_target = target_pt.omega + 14.0 * e_theta + sensor_trim + self.drift_bias + self.i_cross


        if final_point is not None:
            remaining = math.hypot(final_point.x - self.x, final_point.y - self.y)
            braking_limit = math.sqrt(2.0 * self.a_brake * remaining)
            if remaining > 3.0 and target_pt.v < 1.0:
                v_target = min(max(v_target, 25.0), braking_limit)
            else:
                v_target = min(v_target, braking_limit)
            if remaining <= 4.0:
                v_target = 0.0
                heading_error = normalize_angle(final_point.theta - self.theta)
                omega_target = max(-5.0, min(5.0, 8.0 * heading_error))

        # Model each N20 motor independently with finite speed, torque, response,
        # mismatch, battery variation, and a traction-limited differential drive.
        v_left_target = v_target - (self.W / 2.0) * omega_target
        v_right_target = v_target + (self.W / 2.0) * omega_target
        v_left_target = max(-self.max_wheel_speed, min(self.max_wheel_speed, v_left_target))
        v_right_target = max(-self.max_wheel_speed, min(self.max_wheel_speed, v_right_target))
        if self.motor_imbalance_enabled:
            v_left_target *= self.motor_left_gain
            v_right_target *= self.motor_right_gain

        old_left, old_right = self.v_left, self.v_right
        for side, target in (("left", v_left_target), ("right", v_right_target)):
            current = self.v_left if side == "left" else self.v_right
            rate = (target - current) / self.tau_m
            accel_limit = self.a_accel if abs(target) > abs(current) else self.a_brake
            updated = current + max(-accel_limit * dt, min(accel_limit * dt, rate * dt))
            if side == "left":
                self.v_left = updated
            else:
                self.v_right = updated

        v_effective = (self.v_left + self.v_right) / 2.0
        omega_effective = (self.v_right - self.v_left) / self.W
        longitudinal_accel = math.hypot(self.v_left - old_left, self.v_right - old_right) / (math.sqrt(2.0) * max(dt, 1e-6))
        lateral_accel = abs(v_effective * omega_effective)
        friction_limit = self.mu * 9810.0
        demand = math.hypot(longitudinal_accel, lateral_accel)
        self.slip_ratio = max(0.0, min(0.25, 1.0 - friction_limit / demand)) if demand > friction_limit else 0.0
        if lateral_accel > friction_limit and abs(v_effective) > 1.0:
            omega_effective *= friction_limit / lateral_accel
        v_effective *= 1.0 - self.slip_ratio
        self.v = v_effective
        self.omega = omega_effective

        # 8. Physical Integration (Ground Truth State)
        self.x += v_effective * math.cos(self.theta) * dt
        self.y += v_effective * math.sin(self.theta) * dt
        self.theta = normalize_angle(self.theta + omega_effective * dt)

        # 9. IMU Gyro Drift Accumulation
        self.gyro_drift += self.gyro_drift_rate * dt

# ==============================================================================
# 4. PHYSICAL POLYGON-TO-SEGMENT COLLISION ENGINE
# ==============================================================================
class MMSCollisionEngine:
    def __init__(self, maze):
        self.maze = maze
        self.rebuild_obstacles()

    def rebuild_obstacles(self):
        self.wall_segs = []
        for x in range(16):
            for y in range(16):
                x0, x1 = x * 180.0, (x + 1) * 180.0
                y0, y1 = y * 180.0, (y + 1) * 180.0
                if self.maze.has_wall(x, y, 0): self.wall_segs.append(((x0, y1), (x1, y1), f"North Wall ({x},{y})"))
                if self.maze.has_wall(x, y, 1): self.wall_segs.append(((x1, y0), (x1, y1), f"East Wall ({x},{y})"))
                if self.maze.has_wall(x, y, 2): self.wall_segs.append(((x0, y0), (x1, y0), f"South Wall ({x},{y})"))
                if self.maze.has_wall(x, y, 3): self.wall_segs.append(((x0, y0), (x0, y1), f"West Wall ({x},{y})"))

        self.posts = [(x * 180.0, y * 180.0) for x in range(17) for y in range(17)]
        self.wall_grid = {}
        for index, (w1, w2, _) in enumerate(self.wall_segs):
            min_x = min(w1[0], w2[0]) - 6.0
            max_x = max(w1[0], w2[0]) + 6.0
            min_y = min(w1[1], w2[1]) - 6.0
            max_y = max(w1[1], w2[1]) + 6.0
            for cell_x in self._cell_range(min_x, max_x, 15):
                for cell_y in self._cell_range(min_y, max_y, 15):
                    self.wall_grid.setdefault((cell_x, cell_y), []).append(index)

    @staticmethod
    def _cell_range(min_value, max_value, max_cell):
        first = max(0, int(min_value // CELL_SIZE_MM))
        last = min(max_cell, int(max_value // CELL_SIZE_MM))
        return range(first, last + 1)

    def wall_candidates_in_bounds(self, min_x, max_x, min_y, max_y):
        indices = set()
        for cell_x in self._cell_range(min_x, max_x, 15):
            for cell_y in self._cell_range(min_y, max_y, 15):
                indices.update(self.wall_grid.get((cell_x, cell_y), ()))
        return [self.wall_segs[index] for index in indices]

    def wall_candidates_for_ray(self, origin, max_range):
        return self.wall_candidates_in_bounds(
            origin[0] - max_range, origin[0] + max_range,
            origin[1] - max_range, origin[1] + max_range,
        )

    def post_candidates_in_bounds(self, min_x, max_x, min_y, max_y):
        return [
            (cell_x * CELL_SIZE_MM, cell_y * CELL_SIZE_MM)
            for cell_x in self._cell_range(min_x, max_x, 16)
            for cell_y in self._cell_range(min_y, max_y, 16)
        ]

    def post_candidates_for_ray(self, origin, max_range):
        radius = max_range + 8.0
        return self.post_candidates_in_bounds(
            origin[0] - radius, origin[0] + radius,
            origin[1] - radius, origin[1] + radius,
        )

    def check_collision(self, bx, by, btheta, half_w=34.0, half_l=40.0):
        cos_t = math.cos(btheta)
        sin_t = math.sin(btheta)
        corners = [
            (bx + half_l * cos_t - half_w * sin_t, by + half_l * sin_t + half_w * cos_t),
            (bx + half_l * cos_t + half_w * sin_t, by + half_l * sin_t - half_w * cos_t),
            (bx - half_l * cos_t + half_w * sin_t, by - half_l * sin_t - half_w * cos_t),
            (bx - half_l * cos_t - half_w * sin_t, by - half_l * sin_t + half_w * cos_t)
        ]
        edges = [
            (corners[0], corners[1]), (corners[1], corners[2]),
            (corners[2], corners[3]), (corners[3], corners[0])
        ]
        for corner in corners:
            if corner[0] < 0.0 or corner[0] > 2880.0 or corner[1] < 0.0 or corner[1] > 2880.0:
                return True, "Arena Perimeter Wall", corner

        broadphase_radius = half_l + half_w
        nearby_walls = self.wall_candidates_in_bounds(
            bx - broadphase_radius, bx + broadphase_radius,
            by - broadphase_radius, by + broadphase_radius,
        )
        for w1, w2, name in nearby_walls:
            min_wx = min(w1[0], w2[0]) - 6.0
            max_wx = max(w1[0], w2[0]) + 6.0
            min_wy = min(w1[1], w2[1]) - 6.0
            max_wy = max(w1[1], w2[1]) + 6.0
            if bx < min_wx - half_l - half_w or bx > max_wx + half_l + half_w or by < min_wy - half_l - half_w or by > max_wy + half_l + half_w:
                continue
            wall_radius = WALL_HALF_THICKNESS_MM + COLLISION_CLEARANCE_MM
            wall_inside = False
            for wx, wy in (w1, w2):
                lx = (wx - bx) * cos_t + (wy - by) * sin_t
                ly = -(wx - bx) * sin_t + (wy - by) * cos_t
                dx = max(abs(lx) - half_l, 0.0)
                dy = max(abs(ly) - half_w, 0.0)
                if dx * dx + dy * dy <= wall_radius * wall_radius:
                    wall_inside = True
                    break
            if wall_inside or any(segments_intersect(edge[0], edge[1], w1, w2) for edge in edges):
                return True, name, ((w1[0] + w2[0]) * 0.5, (w1[1] + w2[1]) * 0.5)
            if any(dist_pt_to_seg(corner, w1, w2) <= wall_radius for corner in corners):
                return True, name, (w1[0], w1[1])

        post_radius = half_l + POST_RADIUS_MM + COLLISION_CLEARANCE_MM
        nearby_posts = self.post_candidates_in_bounds(
            bx - post_radius, bx + post_radius,
            by - post_radius, by + post_radius,
        )
        for px, py in nearby_posts:
            if abs(bx - px) > half_l + POST_RADIUS_MM + COLLISION_CLEARANCE_MM or abs(by - py) > half_l + POST_RADIUS_MM + COLLISION_CLEARANCE_MM:
                continue
            lx = (px - bx) * cos_t + (py - by) * sin_t
            ly = -(px - bx) * sin_t + (py - by) * cos_t
            dx = max(abs(lx) - half_l, 0.0)
            dy = max(abs(ly) - half_w, 0.0)
            if dx * dx + dy * dy <= (POST_RADIUS_MM + COLLISION_CLEARANCE_MM) ** 2:
                return True, f"Corner Post ({int(px/180)},{int(py/180)})", (px, py)

        return False, None, None

    def check_swept_collision(self, start, end, half_w=ROBOT_HALF_WIDTH_MM,
                              half_l=ROBOT_HALF_LENGTH_MM):
        sx, sy, stheta = start
        ex, ey, etheta = end
        delta_theta = normalize_angle(etheta - stheta)
        distance = math.hypot(ex - sx, ey - sy)
        steps = max(1, math.ceil(distance / 1.0), math.ceil(abs(delta_theta) / math.radians(1.0)))
        for step in range(1, steps + 1):
            fraction = step / steps
            x = sx + (ex - sx) * fraction
            y = sy + (ey - sy) * fraction
            theta = normalize_angle(stheta + delta_theta * fraction)
            collision = self.check_collision(x, y, theta, half_w, half_l)
            if collision[0]:
                return collision
        return False, None, None


def simulation_tick(bot, collider, trajectory, trajectory_index, dt):
    if not trajectory:
        return trajectory_index, (True, "No trajectory", (bot.x, bot.y)), False

    trajectory_index = min(max(0, trajectory_index), len(trajectory) - 1)
    search_end = min(len(trajectory), trajectory_index + 80)
    best_index = trajectory_index
    best_distance = float("inf")
    for index in range(trajectory_index, search_end):
        point = trajectory[index]
        dx, dy = point.x - bot.x, point.y - bot.y
        dtheta = normalize_angle(point.theta - bot.theta)
        distance = dx * dx + dy * dy + (35.0 * dtheta) ** 2
        if distance < best_distance:
            best_distance, best_index = distance, index
    trajectory_index = max(trajectory_index, best_index)

    target_index = min(len(trajectory) - 1, trajectory_index + 6)
    target = trajectory[target_index]
    start_pose = (bot.x, bot.y, bot.theta)
    bot.read_sensors(collider.wall_segs, collider.posts, collider)
    bot.step_physics(dt, target, trajectory[-1])
    end_pose = (bot.x, bot.y, bot.theta)
    collision = collider.check_swept_collision(start_pose, end_pose, bot.half_w, bot.half_l)

    final = trajectory[-1]
    final_distance = math.hypot(final.x - bot.x, final.y - bot.y)
    heading_error = abs(normalize_angle(final.theta - bot.theta))
    finished = (
        final_distance <= 4.0
        and heading_error <= math.radians(3.0)
        and abs(bot.v) <= 12.0
    )
    return trajectory_index, collision, finished


# ==============================================================================
# 5. HIGH-DPI CRISP RESIZABLE MMS SIMULATOR (WITH DUAL VIEW SPLIT SCREEN)
# ==============================================================================
class MMSAdvancedSimulator:
    def __init__(self, seed=1, motor_scale=1.0, friction_scale=1.0):
        pygame.init()
        pygame.display.set_caption("MMS - Micromouse Simulator [Continuous Curve Edition]")
        self.screen_w = 1560
        self.screen_h = 890
        # WINDOWS SNAP COMPATIBILITY: pygame.RESIZABLE flag enables Win+Left / Win+Right and Snap Layouts!
        self.screen = pygame.display.set_mode((self.screen_w, self.screen_h), pygame.RESIZABLE)
        self.clock = pygame.time.Clock()

        # HIGH-RESOLUTION ANTI-ALIASED CRISP FONTS
        self.font_toolbar = pygame.font.SysFont("Segoe UI", 13, bold=True)
        self.font_btn = pygame.font.SysFont("Segoe UI", 12, bold=True)
        self.font_stage = pygame.font.SysFont("Segoe UI", 12, bold=True)
        self.font_cell = pygame.font.SysFont("Consolas", 11, bold=True)
        self.font_hud = pygame.font.SysFont("Segoe UI", 13)
        self.font_hud_bold = pygame.font.SysFont("Segoe UI", 13, bold=True)
        self.font_sensor = pygame.font.SysFont("Consolas", 11, bold=True)
        self.font_log = pygame.font.SysFont("Consolas", 12)

        self.toolbar_h = 46
        self.banner_h = 28
        self.split_screen = True

        # Mazes Library (10 Competition Mazes Testing Every Physical Dynamic)
        self.maze_list = [
            "example5.num",
            "classic.num",
            "speedway.num",
            "japan_finals.num",
            "diagonal_paradise.num",
            "curve_showcase.num",
            "dragstrip_sprint.num",
            "deadend_labyrinth.num",
            "japan_championship_2018.num",
            "apic_world_cup.num"
        ]
        self.current_maze_idx = 0

        # COMPETITION STATE MACHINE: "AUTO_TOURNAMENT", "EXPLORATION", "RETURN", "SPEEDRUN"
        self.competition_mode = "AUTO_TOURNAMENT"
        self.current_phase = "EXPLORATION" # Active phase in the tournament
        self.strategy_mode = "AUTO"        # "AUTO", "CURVES", "DIAGONALS"
        self.best_strategy = "CURVES"
        self.time_savings = 0.0
        self.metrics_curves = {}
        self.metrics_diags = {}

        self.running = False
        self.crashed = False
        self.crash_info = ""
        self.crash_pos = None
        self.finished = False
        self.lap_time = 0.0
        self.phase_time = 0.0
        self.transition_timer = 0.0 # Countdown delay between phases
        self.speed_multiplier = 2.0
        self.trail = []
        self.max_trail = 400

        # MMS Live Console Logs
        self.logs = [
            "[SIM] N20 differential-drive model ready (hardware correlation required).",
            "[HARDWARE] 6-Channel Optical IR Arrays (L90, L45, FL, FR, R45, R90) active.",
            "[PHYSICS] N20 speed/acceleration bounds, wheel lag, traction and uncertainty online.",
            "[STRAIGHTS] Adaptive Multi-Mode Centering (Dual/Single/IMU-Lock) & Post Sync active.",
            f"[MAZE] Active Maze: {self.maze_list[self.current_maze_idx]}."
        ]

        # Load Maze
        self.maze = Maze(16, 16)
        self.load_active_maze()

        # Physical Robot & Collider
        start_pt = self.explore_traj[0]
        self.bot = PhysicalMicromouse(
            start_pt.x, start_pt.y, start_pt.theta, seed=seed,
            motor_scale=motor_scale, friction_scale=friction_scale)
        self.sim_accumulator = 0.0
        self.collider = MMSCollisionEngine(self.maze)

        self.traj_points = self.explore_traj
        self.traj_idx = 0

        # Colors
        self.CLR_BG = (235, 238, 242)
        self.CLR_TOOLBAR = (246, 247, 250)
        self.CLR_BANNER = (228, 232, 240)
        self.CLR_MAZE_BG = (0, 0, 0)
        self.CLR_WALL = (235, 38, 38)
        self.CLR_POST = (235, 38, 38)
        self.CLR_GRID = (28, 30, 34)
        self.CLR_VISITED = (0, 28, 36)
        self.CLR_GOAL = (240, 180, 0)
        self.CLR_BOT = (210, 218, 228)

        self.update_layout()

    def load_active_maze(self):
        maze_filename = self.maze_list[self.current_maze_idx]
        maze_path = os.path.join(os.path.dirname(__file__), "mazes", maze_filename)
        if not os.path.exists(maze_path):
            maze_path = os.path.join(os.path.dirname(__file__), maze_filename)
        self.maze.load_num(maze_path)

        for x in range(16):
            for y in range(16):
                self.maze.visited[x][y] = True

        goals = {(7, 7), (7, 8), (8, 7), (8, 8)}
        self.maze.compute_floodfill_distances(goals)

        # 1. Precompute Exploration Trajectory (Orthogonal mapping search)
        exp_path = generate_floodfill_exploration_path(self.maze, goals)
        exp_wp, exp_diag = extract_safe_waypoints(exp_path, self.maze, allow_diagonals=False)
        self.explore_traj = generate_curve_trajectory(
            exp_wp, exp_diag, nominal_R=55.0, v_straight=240.0, v_diag=240.0,
            v_curve_max=200.0, acceleration=1500.0, deceleration=1500.0)

        # 2. Precompute Return Trajectory (Center -> (0,0) with high-speed diagonal glide)
        last_cell = exp_path[-1]
        ret_path = dijkstra_fastest_path(self.maze, last_cell[0], last_cell[1], DIR_NORTH, {(0, 0)})
        ret_wp, ret_diag = extract_safe_waypoints(ret_path, self.maze, allow_diagonals=True)
        self.return_traj = generate_curve_trajectory(
            ret_wp, ret_diag, nominal_R=60.0, v_straight=380.0, v_diag=380.0,
            v_curve_max=300.0, acceleration=2000.0, deceleration=2000.0)

        # 3. Precompute Championship Speedrun Options (Curves vs Diagonals)
        fwd_path = dijkstra_fastest_path(self.maze, 0, 0, DIR_NORTH, goals)
        
        # Option A: Pure Continuous Curves
        fwd_wp_c, fwd_diag_c = extract_safe_waypoints(fwd_path, self.maze, allow_diagonals=False)
        self.speedrun_traj_curves = generate_curve_trajectory(
            fwd_wp_c, fwd_diag_c, nominal_R=60.0, v_straight=500.0,
            v_curve_max=350.0, acceleration=2600.0, deceleration=2600.0)
        self.metrics_curves = calculate_trajectory_metrics(self.speedrun_traj_curves)

        # Option B: With Diagonals
        fwd_wp_d, fwd_diag_d = extract_safe_waypoints(fwd_path, self.maze, allow_diagonals=True)
        self.speedrun_traj_diags = generate_curve_trajectory(
            fwd_wp_d, fwd_diag_d, nominal_R=60.0, v_straight=500.0,
            v_diag=550.0, v_curve_max=350.0, acceleration=2600.0, deceleration=2600.0)
        self.metrics_diags = calculate_trajectory_metrics(self.speedrun_traj_diags)


        # Dynamic Strategy Optimizer
        t_c = self.metrics_curves['time']
        t_d = self.metrics_diags['time']
        if t_d < t_c:
            self.best_strategy = "DIAGONALS"
            self.time_savings = t_c - t_d
        else:
            self.best_strategy = "CURVES"
            self.time_savings = t_d - t_c

        self.apply_strategy(getattr(self, 'strategy_mode', 'AUTO'), log_decision=True)

        if hasattr(self, 'collider'):
            self.collider.rebuild_obstacles()

    def apply_strategy(self, mode, log_decision=False):
        self.strategy_mode = mode
        effective = self.best_strategy if mode == "AUTO" else mode

        if effective == "DIAGONALS":
            self.speedrun_traj = self.speedrun_traj_diags
        else:
            self.speedrun_traj = self.speedrun_traj_curves

        if self.current_phase == "SPEEDRUN":
            self.traj_points = self.speedrun_traj
            self.traj_idx = 0

        if log_decision and hasattr(self, 'metrics_curves') and self.metrics_curves:
            t_c = self.metrics_curves.get('time', 0.0)
            t_d = self.metrics_diags.get('time', 0.0)
            m_name = self.maze_list[self.current_maze_idx].replace('.num', '')
            self.log(f"[OPTIMIZER] ⏱ Strategy Benchmark on {m_name}:")
            self.log(f"  • Continuous Curves: {t_c:.2f}s ({self.metrics_curves.get('distance',0):.0f}mm @ {self.metrics_curves.get('avg_speed',0):.0f}mm/s)")
            self.log(f"  • Diagonal Sprints:  {t_d:.2f}s ({self.metrics_diags.get('distance',0):.0f}mm @ {self.metrics_diags.get('avg_speed',0):.0f}mm/s)")
            self.log(f"  ⚡ AUTO-SELECTED: {self.best_strategy} ({self.time_savings:.2f}s FASTER!)")

    def cycle_strategy(self):
        order = ["AUTO", "CURVES", "DIAGONALS"]
        idx = order.index(self.strategy_mode)
        next_mode = order[(idx + 1) % len(order)]
        self.apply_strategy(next_mode, log_decision=False)
        self.log(f"[STRATEGY] Speedrun Strategy set to: {self.strategy_mode}!")

    def update_layout(self):
        """Dynamically updates geometry when window is resized or snapped."""
        is_compact = self.screen_w < 1100
        top_offset = self.toolbar_h + self.banner_h

        if is_compact:
            self.split_screen = False
            self.global_w = self.screen_w - 40
            self.global_h = self.screen_h - top_offset - 20
            self.global_cell_px = min(self.global_w, self.global_h) / 16.0
            self.global_scale = self.global_cell_px / CELL_SIZE_MM
            self.global_offset_x = 20
            self.global_offset_y = top_offset + 10
        else:
            self.global_w = int(self.screen_w * 0.48)
            self.global_h = self.screen_h - top_offset - 20
            self.global_cell_px = min(self.global_w - 20, self.global_h - 20) / 16.0
            self.global_scale = self.global_cell_px / CELL_SIZE_MM
            self.global_offset_x = 20
            self.global_offset_y = top_offset + 10

            self.chase_x = self.global_offset_x + int(16 * self.global_cell_px) + 24
            self.chase_y = top_offset + 10
            self.chase_w = self.screen_w - self.chase_x - 20
            self.chase_h = int((self.screen_h - top_offset) * 0.47)

            self.hud_x = self.chase_x
            self.hud_y = self.chase_y + self.chase_h + 10
            self.hud_w = self.chase_w
            self.hud_h = self.screen_h - self.hud_y - 12

        # Toolbar Buttons (responsive layout)
        x = 10
        self.btn_run_rect = pygame.Rect(x, 8, 52, 30)
        x += 56 # 66
        self.btn_pause_rect = pygame.Rect(x, 8, 55, 30)
        x += 59 # 125
        self.btn_reset_rect = pygame.Rect(x, 8, 55, 30)
        x += 59 # 184
        self.btn_maze_rect = pygame.Rect(x, 8, 115, 30)
        x += 119 # 303
        self.btn_mode_rect = pygame.Rect(x, 8, 125, 30)
        x += 129 # 432
        self.btn_strat_rect = pygame.Rect(x, 8, 125, 30)
        x += 129 # 561
        self.btn_split_rect = pygame.Rect(x, 8, 80, 30)
        x += 84 # 645
        self.btn_center_rect = pygame.Rect(x, 8, 95, 30)
        x += 99 # 744
        self.btn_noise_rect = pygame.Rect(x, 8, 95, 30)
        x += 99 # 843
        self.btn_imbal_rect = pygame.Rect(x, 8, 100, 30)
        x += 104 # 947
        self.btn_drift_rect = pygame.Rect(x, 8, 65, 30)
        x += 69 # 1016
        self.btn_crash_rect = pygame.Rect(x, 8, 65, 30)
        x += 69 # 1085
        self.slider_track_rect = pygame.Rect(x, 19, 85, 8)

    def log(self, msg):
        self.logs.append(msg)
        if len(self.logs) > 30: self.logs.pop(0)

    def world_to_global_screen(self, x_mm, y_mm):
        px = self.global_offset_x + x_mm * self.global_scale
        py = self.global_offset_y + (16 * CELL_SIZE_MM - y_mm) * self.global_scale
        return px, py

    def cycle_maze(self):
        self.current_maze_idx = (self.current_maze_idx + 1) % len(self.maze_list)
        self.load_active_maze()
        self.reset_sim()
        self.log(f"[MAZE] Loaded {self.maze_list[self.current_maze_idx]}. Trajectories recomputed!")

    def set_phase(self, new_phase, keep_running=False):
        self.current_phase = new_phase
        if new_phase == "EXPLORATION":
            self.traj_points = self.explore_traj
        elif new_phase == "RETURN":
            self.traj_points = self.return_traj
        elif new_phase == "SPEEDRUN":
            self.traj_points = self.speedrun_traj

        self.traj_idx = 0
        if self.traj_points:
            start_pt = self.traj_points[0]
            self.bot.x = start_pt.x
            self.bot.y = start_pt.y
            self.bot.theta = start_pt.theta
        self.bot.v = 0.0
        self.bot.omega = 0.0
        self.bot.v_left = 0.0
        self.bot.v_right = 0.0
        self.bot.i_cross = 0.0
        self.bot.gyro_drift = 0.0
        self.bot.drift_bias = 0.0
        self.phase_time = 0.0
        self.finished = False
        self.crashed = False
        self.crash_info = ""
        self.crash_pos = None
        self.trail.clear()
        self.sim_accumulator = 0.0
        self.running = keep_running

    def cycle_mode(self):
        modes = ["AUTO_TOURNAMENT", "EXPLORATION", "RETURN", "SPEEDRUN"]
        idx = modes.index(self.competition_mode)
        self.competition_mode = modes[(idx + 1) % len(modes)]
        if self.competition_mode == "AUTO_TOURNAMENT":
            self.set_phase("EXPLORATION")
        else:
            self.set_phase(self.competition_mode)
        self.reset_sim()
        self.log(f"[MODE] Mode set to {self.competition_mode}")

    def reset_sim(self):
        if self.competition_mode == "AUTO_TOURNAMENT":
            self.set_phase("EXPLORATION", keep_running=False)
        else:
            self.set_phase(self.competition_mode, keep_running=False)
        self.running = False
        self.crashed = False
        self.finished = False
        self.lap_time = 0.0
        self.phase_time = 0.0
        self.transition_timer = 0.0
        self.trail.clear()
        self.collider.rebuild_obstacles()
        self.log(f"[SIM] Simulator reset. Ready at Phase: {self.current_phase}")

    def toggle_wall_centering(self):
        self.bot.wall_centering_enabled = not self.bot.wall_centering_enabled
        status_str = "ENABLED" if self.bot.wall_centering_enabled else "DISABLED"
        self.log(f"[CONTROL] Optical IR Wall Centering {status_str}!")

    def toggle_noise(self):
        self.bot.optical_noise_enabled = not self.bot.optical_noise_enabled
        self.bot.glitches_enabled = self.bot.optical_noise_enabled
        st = "ENABLED (Noise ±15 ADC + Glitches)" if self.bot.optical_noise_enabled else "DISABLED (Ideal Beam)"
        self.log(f"[SENSORS] Optical Sensor Noise & Glitches {st}!")

    def toggle_imbalance(self):
        self.bot.motor_imbalance_enabled = not self.bot.motor_imbalance_enabled
        st = "ENABLED (1.2% Drift)" if self.bot.motor_imbalance_enabled else "DISABLED (0.0% Symmetric)"
        self.log(f"[HARDWARE] Motor & Wheel Mechanical Asymmetry {st}!")

    def inject_drift(self):
        self.bot.drift_bias += math.radians(3.5)
        self.log(f"[DISTURBANCE] Injected +3.5° Steering Error! Watch mouse veer...")

    def trigger_test_crash(self):
        cur_cx = int(self.bot.x // CELL_SIZE_MM)
        cur_cy = int(self.bot.y // CELL_SIZE_MM)
        self.maze.set_wall(cur_cx, cur_cy, DIR_NORTH, True)
        self.maze.set_wall(cur_cx, cur_cy, DIR_EAST, True)
        self.collider.rebuild_obstacles()
        self.running = True
        self.log(f"[SIM] Wall barrier placed at ({cur_cx},{cur_cy})! Watch crash...")

    def run(self):
        app_running = True
        dragging_slider = False

        while app_running:
            dt = self.clock.tick(60) / 1000.0

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    app_running = False
                elif event.type == pygame.VIDEORESIZE:
                    self.screen_w = max(800, event.w)
                    self.screen_h = max(550, event.h)
                    self.screen = pygame.display.set_mode((self.screen_w, self.screen_h), pygame.RESIZABLE)
                    self.update_layout()
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_SPACE:
                        if not self.crashed:
                            self.running = not self.running
                    elif event.key == pygame.K_r:
                        self.reset_sim()
                    elif event.key == pygame.K_m:
                        self.cycle_maze()
                    elif event.key == pygame.K_a:
                        self.competition_mode = "AUTO_TOURNAMENT"
                        self.set_phase("EXPLORATION")
                        self.running = True
                    elif event.key == pygame.K_1:
                        self.competition_mode = "EXPLORATION"
                        self.set_phase("EXPLORATION")
                    elif event.key == pygame.K_2:
                        self.competition_mode = "RETURN"
                        self.set_phase("RETURN")
                    elif event.key == pygame.K_3:
                        self.competition_mode = "SPEEDRUN"
                        self.set_phase("SPEEDRUN")
                    elif event.key == pygame.K_s:
                        self.split_screen = not self.split_screen
                    elif event.key == pygame.K_o:
                        self.cycle_strategy()
                    elif event.key == pygame.K_w:
                        self.toggle_wall_centering()
                    elif event.key == pygame.K_g:
                        self.toggle_noise()
                    elif event.key == pygame.K_i:
                        self.toggle_imbalance()
                    elif event.key == pygame.K_d:
                        self.inject_drift()
                    elif event.key == pygame.K_c:
                        self.trigger_test_crash()
                    elif event.key == pygame.K_UP:
                        self.speed_multiplier = min(10.0, self.speed_multiplier + 0.5)
                    elif event.key == pygame.K_DOWN:
                        self.speed_multiplier = max(0.5, self.speed_multiplier - 0.5)
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    mx, my = event.pos
                    if self.btn_run_rect.collidepoint(mx, my):
                        if not self.crashed: self.running = True
                    elif self.btn_pause_rect.collidepoint(mx, my):
                        self.running = False
                    elif self.btn_reset_rect.collidepoint(mx, my):
                        self.reset_sim()
                    elif self.btn_maze_rect.collidepoint(mx, my):
                        self.cycle_maze()
                    elif self.btn_mode_rect.collidepoint(mx, my):
                        self.cycle_mode()
                    elif self.btn_strat_rect.collidepoint(mx, my):
                        self.cycle_strategy()
                    elif self.btn_split_rect.collidepoint(mx, my):
                        self.split_screen = not self.split_screen
                    elif self.btn_center_rect.collidepoint(mx, my):
                        self.toggle_wall_centering()
                    elif self.btn_noise_rect.collidepoint(mx, my):
                        self.toggle_noise()
                    elif self.btn_imbal_rect.collidepoint(mx, my):
                        self.toggle_imbalance()
                    elif self.btn_drift_rect.collidepoint(mx, my):
                        self.inject_drift()
                    elif self.btn_crash_rect.collidepoint(mx, my):
                        self.trigger_test_crash()
                    elif self.slider_track_rect.inflate(10, 14).collidepoint(mx, my):
                        dragging_slider = True
                elif event.type == pygame.MOUSEBUTTONUP:
                    dragging_slider = False
                elif event.type == pygame.MOUSEMOTION and dragging_slider:
                    mx, _ = event.pos
                    rel = max(0.0, min(1.0, (mx - self.slider_track_rect.x) / self.slider_track_rect.w))
                    self.speed_multiplier = 0.5 + rel * 9.5

            # ------------------------------------------------------------------
            # AUTOMATIC MULTI-PHASE TOURNAMENT TRANSITION LOGIC
            # ------------------------------------------------------------------
            if self.transition_timer > 0.0:
                self.transition_timer -= dt
                if self.transition_timer <= 0.0:
                    self.transition_timer = 0.0
                    if self.competition_mode == "AUTO_TOURNAMENT":
                        if self.current_phase == "EXPLORATION":
                            self.log("[TOURNAMENT] 🔄 Transitioning to Phase 2: RETURN TO START (0,0)...")
                            self.set_phase("RETURN", keep_running=True)
                        elif self.current_phase == "RETURN":
                            self.log("[TOURNAMENT] 🚀 Transitioning to Phase 3: CHAMPIONSHIP SPEEDRUN...")
                            self.set_phase("SPEEDRUN", keep_running=True)

            # ------------------------------------------------------------------
            # PHYSICAL SIMULATION SUBSTEPPING LOOP (ZERO EULER LAG)
            # ------------------------------------------------------------------
            if self.running and not self.crashed and not self.finished and self.traj_points and self.transition_timer == 0.0:
                self.sim_accumulator += dt * self.speed_multiplier
                n_sub = int(self.sim_accumulator / CONTROL_DT_S)
                self.sim_accumulator -= n_sub * CONTROL_DT_S
                self.lap_time += dt * self.speed_multiplier
                self.phase_time += dt * self.speed_multiplier

                for _ in range(n_sub):
                    self.traj_idx, collision, reached_goal = simulation_tick(
                        self.bot, self.collider, self.traj_points,
                        self.traj_idx, CONTROL_DT_S)
                    if collision[0]:
                        self.crashed = True
                        self.running = False
                        self.crash_info = collision[1]
                        self.crash_pos = collision[2]
                        self.log(f"[COLLISION] Hit {collision[1]} at X={collision[2][0]:.1f}, Y={collision[2][1]:.1f} mm.")
                        break

                    if reached_goal:
                        if self.competition_mode == "AUTO_TOURNAMENT":
                            if self.current_phase == "EXPLORATION":
                                self.log(f"[MOUSE] 🏆 CENTER GOAL REACHED! Exploration Time: {self.phase_time:.2f}s.")
                                self.log("[TOURNAMENT] Auto-returning to Start in 0.8s...")
                                self.transition_timer = 0.8
                            elif self.current_phase == "RETURN":
                                self.log(f"[MOUSE] 🏁 RETURNED TO START (0,0)! Return Time: {self.phase_time:.2f}s.")
                                self.log("[TOURNAMENT] Launching Championship Speedrun in 1.0s...")
                                self.transition_timer = 1.0
                            elif self.current_phase == "SPEEDRUN":
                                self.finished = True
                                self.running = False
                                self.log(f"[MOUSE] 🥇 CHAMPIONSHIP SPEEDRUN WON! Lap: {self.phase_time:.3f}s! Total Tournament: {self.lap_time:.2f}s.")
                        else:
                            self.finished = True
                            self.running = False
                            self.log(f"[MOUSE] 🏆 Finished {self.current_phase}! Time: {self.phase_time:.3f}s.")
                        break

                # Update trajectory trail
                cur_pt = self.traj_points[self.traj_idx]
                gx, gy = self.world_to_global_screen(self.bot.x, self.bot.y)
                self.trail.append((gx, gy, cur_pt.g_force))
                if len(self.trail) > self.max_trail:
                    self.trail.pop(0)

            self.draw()
            pygame.display.flip()

        pygame.quit()

    def draw(self):
        self.screen.fill(self.CLR_BG)

        # ----------------------------------------------------------------------
        # 1. MMS TOP TOOLBAR
        # ----------------------------------------------------------------------
        pygame.draw.rect(self.screen, self.CLR_TOOLBAR, (0, 0, self.screen_w, self.toolbar_h))
        pygame.draw.line(self.screen, (210, 215, 222), (0, self.toolbar_h), (self.screen_w, self.toolbar_h), 1)

        def draw_btn(rect, text, is_active=False, bg_clr=(230, 234, 240), text_clr=(20, 25, 35)):
            pygame.draw.rect(self.screen, bg_clr, rect, border_radius=4)
            pygame.draw.rect(self.screen, (165, 172, 184), rect, width=1, border_radius=4)
            lbl = self.font_btn.render(text, True, text_clr)
            self.screen.blit(lbl, (rect.x + (rect.w - lbl.get_width()) // 2, rect.y + 7))

        draw_btn(self.btn_run_rect, "▶  Run", self.running, (40, 167, 69) if self.running else (230, 234, 240), (255, 255, 255) if self.running else (20, 20, 20))
        draw_btn(self.btn_pause_rect, "❚❚ Pause", not self.running, (230, 234, 240), (20, 20, 20))
        draw_btn(self.btn_reset_rect, "↺  Reset", False, (230, 234, 240), (20, 20, 20))
        mz_raw = self.maze_list[self.current_maze_idx].replace('.num', '').replace('_', ' ')
        if len(mz_raw) > 14: mz_raw = mz_raw[:13] + ".."
        draw_btn(self.btn_maze_rect, f"Maze: {mz_raw}", False, (240, 245, 255), (0, 90, 180))
        
        mode_label = "AUTO-TOURNAMENT" if self.competition_mode == "AUTO_TOURNAMENT" else f"Mode: {self.competition_mode[:7]}"
        draw_btn(self.btn_mode_rect, mode_label, False, (235, 243, 255), (0, 100, 220))

        # Dynamic Strategy Button (Auto Fastest / Curves / Diagonals)
        if self.strategy_mode == "AUTO":
            strat_txt = f"⚡ AUTO ({self.best_strategy[:5]})"
            strat_bg = (230, 250, 240)
            strat_clr = (0, 140, 70)
        elif self.strategy_mode == "DIAGONALS":
            strat_txt = "📐 DIAGONALS"
            strat_bg = (255, 245, 230)
            strat_clr = (200, 100, 0)
        else:
            strat_txt = "🏎 CURVES"
            strat_bg = (240, 245, 255)
            strat_clr = (0, 100, 220)
        draw_btn(self.btn_strat_rect, strat_txt, False, strat_bg, strat_clr)

        draw_btn(self.btn_split_rect, f"📺 Split: {'ON' if self.split_screen else 'OFF'}", self.split_screen, (230, 240, 255), (0, 80, 180))
        draw_btn(self.btn_center_rect, f"🎯 Centering", self.bot.wall_centering_enabled, (230, 255, 240) if self.bot.wall_centering_enabled else (255, 230, 230), (0, 140, 60) if self.bot.wall_centering_enabled else (200, 30, 30))
        draw_btn(self.btn_noise_rect, f"⚡ Noise: {'ON' if self.bot.optical_noise_enabled else 'OFF'}", self.bot.optical_noise_enabled, (230, 255, 240) if self.bot.optical_noise_enabled else (255, 235, 235), (0, 140, 60) if self.bot.optical_noise_enabled else (180, 50, 50))
        draw_btn(self.btn_imbal_rect, f"⚙ Imbal: {'ON' if self.bot.motor_imbalance_enabled else 'OFF'}", self.bot.motor_imbalance_enabled, (255, 245, 230) if self.bot.motor_imbalance_enabled else (240, 245, 255), (180, 90, 0) if self.bot.motor_imbalance_enabled else (0, 90, 180))
        draw_btn(self.btn_drift_rect, "⚠️ Drift", False, (255, 245, 230), (180, 90, 0))
        draw_btn(self.btn_crash_rect, "💥 Crash", False, (255, 235, 235), (200, 30, 30))

        # Speed Slider
        lbl_turtle = self.font_toolbar.render("🐢", True, (80, 85, 95))
        lbl_rabbit = self.font_toolbar.render("🐇", True, (80, 85, 95))
        self.screen.blit(lbl_turtle, (self.slider_track_rect.x - 20, 13))
        pygame.draw.rect(self.screen, (190, 195, 205), self.slider_track_rect, border_radius=4)
        handle_rel = (self.speed_multiplier - 0.5) / 9.5
        handle_x = self.slider_track_rect.x + int(handle_rel * self.slider_track_rect.w)
        pygame.draw.circle(self.screen, (0, 120, 215), (handle_x, self.slider_track_rect.centery), 7)
        self.screen.blit(lbl_rabbit, (self.slider_track_rect.right + 6, 13))
        lbl_speed = self.font_btn.render(f"{self.speed_multiplier:.1f}x", True, (50, 55, 65))
        self.screen.blit(lbl_speed, (self.slider_track_rect.right + 30, 15))

        # ----------------------------------------------------------------------
        # 1B. COMPETITION STAGE BANNER
        # ----------------------------------------------------------------------
        banner_rect = pygame.Rect(0, self.toolbar_h, self.screen_w, self.banner_h)
        pygame.draw.rect(self.screen, self.CLR_BANNER, banner_rect)
        pygame.draw.line(self.screen, (200, 205, 215), (0, self.toolbar_h + self.banner_h), (self.screen_w, self.toolbar_h + self.banner_h), 1)

        speedrun_banner = f"PHASE 3: SPEEDRUN [⚡ {self.strategy_mode}: {self.best_strategy} (-{self.time_savings:.1f}s)]"
        stages = [
            ("PHASE 1: EXPLORATION (0,0 ➔ Center)", "EXPLORATION"),
            ("PHASE 2: RETURN TO START (Center ➔ 0,0)", "RETURN"),
            (speedrun_banner, "SPEEDRUN")
        ]
        seg_w = self.screen_w // 3
        for idx, (title, ph_key) in enumerate(stages):
            sx = idx * seg_w
            is_active = (self.current_phase == ph_key)
            if is_active:
                pygame.draw.rect(self.screen, (0, 120, 215), (sx + 4, self.toolbar_h + 3, seg_w - 8, self.banner_h - 6), border_radius=4)
                lbl_st = self.font_stage.render(f"● {title}", True, (255, 255, 255))
            else:
                lbl_st = self.font_stage.render(title, True, (110, 120, 135))
            self.screen.blit(lbl_st, (sx + (seg_w - lbl_st.get_width()) // 2, self.toolbar_h + 6))

        # ----------------------------------------------------------------------
        # 2. LEFT VIEWPORT: 16x16 MICROMOUSE MAZE FIELD
        # ----------------------------------------------------------------------
        maze_rect = pygame.Rect(self.global_offset_x, self.global_offset_y, int(16 * self.global_cell_px), int(16 * self.global_cell_px))
        pygame.draw.rect(self.screen, self.CLR_MAZE_BG, maze_rect)
        pygame.draw.rect(self.screen, (60, 65, 75), maze_rect, width=2)

        # Draw Cells & Goal Area
        for x in range(16):
            for y in range(16):
                px = self.global_offset_x + x * self.global_cell_px
                py = self.global_offset_y + (15 - y) * self.global_cell_px
                if (x, y) in {(7, 7), (7, 8), (8, 7), (8, 8)}:
                    pygame.draw.rect(self.screen, (45, 36, 12), (px + 1, py + 1, self.global_cell_px - 1, self.global_cell_px - 1))
                elif (x, y) == (0, 0):
                    pygame.draw.rect(self.screen, (10, 35, 55), (px + 1, py + 1, self.global_cell_px - 1, self.global_cell_px - 1))
                pygame.draw.rect(self.screen, self.CLR_GRID, (px, py, self.global_cell_px, self.global_cell_px), width=1)

        # Draw Walls
        wall_thick = max(2, int(6.0 * self.global_scale))
        for w1, w2, _ in self.collider.wall_segs:
            p1 = self.world_to_global_screen(w1[0], w1[1])
            p2 = self.world_to_global_screen(w2[0], w2[1])
            pygame.draw.line(self.screen, self.CLR_WALL, p1, p2, wall_thick)

        # Draw Corner Posts
        post_rad = max(2, int(6.0 * self.global_scale))
        for px, py in self.collider.posts:
            pp = self.world_to_global_screen(px, py)
            pygame.draw.circle(self.screen, self.CLR_POST, (int(pp[0]), int(pp[1])), post_rad)

        # Draw Global Trajectory Planned Route
        if self.traj_points:
            traj_screen_pts = [self.world_to_global_screen(p.x, p.y) for p in self.traj_points]
            if len(traj_screen_pts) > 1:
                pygame.draw.lines(self.screen, (50, 160, 255), False, traj_screen_pts, 2)

        # Draw Trajectory Heatmap Trail
        if len(self.trail) > 1:
            for t_idx in range(len(self.trail) - 1):
                p1 = (int(self.trail[t_idx][0]), int(self.trail[t_idx][1]))
                p2 = (int(self.trail[t_idx+1][0]), int(self.trail[t_idx+1][1]))
                gf = self.trail[t_idx][2]
                t_clr = (255, 60, 60) if gf > 0.65 else ((255, 180, 0) if gf > 0.35 else (0, 230, 160))
                pygame.draw.line(self.screen, t_clr, p1, p2, 3)

        # Draw Global Robot Polygon
        bx, by = self.world_to_global_screen(self.bot.x, self.bot.y)
        theta = -self.bot.theta + math.pi / 2.0
        bot_w = 68.0 * self.global_scale
        bot_l = 80.0 * self.global_scale
        half_w, half_l = bot_w / 2.0, bot_l / 2.0
        corners = [
            (half_l, -half_w), (half_l + 6, 0), (half_l, half_w),
            (-half_l, half_w), (-half_l, -half_w)
        ]
        rot_corners = [(bx + cx * math.cos(theta) - cy * math.sin(theta), by + cx * math.sin(theta) + cy * math.cos(theta)) for cx, cy in corners]
        bot_clr = (255, 40, 40) if self.crashed else self.CLR_BOT
        pygame.draw.polygon(self.screen, bot_clr, rot_corners)
        pygame.draw.polygon(self.screen, (40, 45, 55), rot_corners, 2)

        if self.crashed and self.crash_pos:
            cx, cy = self.world_to_global_screen(self.crash_pos[0], self.crash_pos[1])
            pygame.draw.circle(self.screen, (255, 230, 0), (int(cx), int(cy)), 12, 3)
            pygame.draw.circle(self.screen, (255, 40, 0), (int(cx), int(cy)), 6)

        # ----------------------------------------------------------------------
        # 3. RIGHT VIEWPORT: SPLIT SCREEN CHASE CAMERA & COCKPIT
        # ----------------------------------------------------------------------
        if self.split_screen and hasattr(self, 'chase_x'):
            chase_rect = pygame.Rect(self.chase_x, self.chase_y, self.chase_w, self.chase_h)
            pygame.draw.rect(self.screen, (10, 13, 18), chase_rect)
            pygame.draw.rect(self.screen, (180, 190, 205), chase_rect, width=1)

            self.screen.set_clip(chase_rect)

            zoom = self.global_scale * 3.6
            cam_cx, cam_cy = chase_rect.centerx, chase_rect.centery

            def world_to_cam(wx, wy):
                cpx = cam_cx + (wx - self.bot.x) * zoom
                cpy = cam_cy - (wy - self.bot.y) * zoom
                return cpx, cpy

            wall_thick_cam = 7
            for w1, w2, _ in self.collider.wall_segs:
                p1 = world_to_cam(w1[0], w1[1])
                p2 = world_to_cam(w2[0], w2[1])
                if -100 <= p1[0] <= self.screen_w + 100 or -100 <= p2[0] <= self.screen_w + 100:
                    pygame.draw.line(self.screen, self.CLR_WALL, p1, p2, wall_thick_cam)

            post_sz_cam = 14
            for px, py in self.collider.posts:
                cpx, cpy = world_to_cam(px, py)
                if chase_rect.collidepoint(cpx, cpy):
                    r = pygame.Rect(cpx - post_sz_cam // 2, cpy - post_sz_cam // 2, post_sz_cam, post_sz_cam)
                    pygame.draw.rect(self.screen, self.CLR_POST, r)
                    pygame.draw.rect(self.screen, (0, 0, 0), r.inflate(-6, -6))

            def draw_sensor_ray(sensor_data, sensor_orig, beam_color=(0, 240, 120), label_name=""):
                dist, hit_pt, adc_val, is_post, glitch = sensor_data.dist, sensor_data.hit_pt, sensor_data.adc, sensor_data.is_post, sensor_data.glitch
                o_px, o_py = world_to_cam(sensor_orig[0], sensor_orig[1])
                if hit_pt:
                    h_px, h_py = world_to_cam(hit_pt[0], hit_pt[1])
                    beam_clr = (255, 60, 60) if glitch else beam_color
                    pygame.draw.line(self.screen, beam_clr, (o_px, o_py), (h_px, h_py), 2)
                    pygame.draw.circle(self.screen, (255, 255, 255), (int(h_px), int(h_py)), 4)
                    if is_post:
                        pygame.draw.circle(self.screen, (255, 160, 0), (int(h_px), int(h_py)), 7, 2)
                    tag_str = f"{label_name} {dist:.0f}mm ({adc_val} ADC)"
                    tag_clr = (255, 80, 80) if glitch else (255, 255, 120)
                    tag_lbl = self.font_sensor.render(tag_str, True, tag_clr)
                    self.screen.blit(tag_lbl, (h_px + 6, h_py - 6))

            cos_b = math.cos(self.bot.theta)
            sin_b = math.sin(self.bot.theta)

            pos_L90 = (self.bot.x - self.bot.half_w * sin_b, self.bot.y + self.bot.half_w * cos_b)
            pos_L45 = (self.bot.x + 36.0 * cos_b - 25.0 * sin_b, self.bot.y + 36.0 * sin_b + 25.0 * cos_b)
            pos_FL  = (self.bot.x + self.bot.half_l * cos_b - 12.0 * sin_b, self.bot.y + self.bot.half_l * sin_b + 12.0 * cos_b)
            pos_FR  = (self.bot.x + self.bot.half_l * cos_b + 12.0 * sin_b, self.bot.y + self.bot.half_l * sin_b - 12.0 * cos_b)
            pos_R45 = (self.bot.x + 36.0 * cos_b + 25.0 * sin_b, self.bot.y + 36.0 * sin_b - 25.0 * cos_b)
            pos_R90 = (self.bot.x + self.bot.half_w * sin_b, self.bot.y - self.bot.half_w * cos_b)

            # Draw all 6 optical sensor rays with authentic hardware geometry
            draw_sensor_ray(self.bot.sensor_l90, pos_L90, (0, 230, 255), "L90")
            draw_sensor_ray(self.bot.sensor_l45, pos_L45, (180, 255, 50), "L45")
            draw_sensor_ray(self.bot.sensor_fl,  pos_FL,  (0, 255, 140), "FL")
            draw_sensor_ray(self.bot.sensor_fr,  pos_FR,  (0, 255, 140), "FR")
            draw_sensor_ray(self.bot.sensor_r45, pos_R45, (180, 255, 50), "R45")
            draw_sensor_ray(self.bot.sensor_r90, pos_R90, (0, 230, 255), "R90")

            z_w = 70.0 * zoom
            z_l = 80.0 * zoom
            z_half_w = z_w / 2.0
            z_half_l = z_l / 2.0
            z_corners = [
                (z_half_l, -z_half_w), (z_half_l + 12, 0), (z_half_l, z_half_w),
                (-z_half_l, z_half_w), (-z_half_l, -z_half_w)
            ]
            z_rot = [(cam_cx + cx * math.cos(theta) - cy * math.sin(theta), cam_cy + cx * math.sin(theta) + cy * math.cos(theta)) for cx, cy in z_corners]
            pygame.draw.polygon(self.screen, bot_clr, z_rot)
            pygame.draw.polygon(self.screen, (30, 35, 45), z_rot, 3)

            for side in [-1, 1]:
                wh_x = cam_cx + (z_half_w + 3) * side * math.sin(theta)
                wh_y = cam_cy + (z_half_w + 3) * side * math.cos(theta)
                pygame.draw.circle(self.screen, (20, 20, 20), (int(wh_x), int(wh_y)), 7)
                pygame.draw.circle(self.screen, (100, 110, 125), (int(wh_x), int(wh_y)), 4)

            nose_px = cam_cx + (z_half_l + 25) * math.cos(theta)
            nose_py = cam_cy + (z_half_l + 25) * math.sin(theta)
            pygame.draw.line(self.screen, (255, 50, 50) if self.crashed else (0, 255, 100), (cam_cx, cam_cy), (nose_px, nose_py), 3)

            self.screen.set_clip(None)

            lbl_cam = self.font_hud_bold.render("DYNAMIC ONBOARD CHASE CAMERA (3.6x) + 6-CH OPTICAL IR ARRAYS", True, (0, 140, 220))
            self.screen.blit(lbl_cam, (self.chase_x + 12, self.chase_y + 8))

        # ----------------------------------------------------------------------
        # 4. RIGHT VIEWPORT: TELEMETRY & LIVE MONOSPACE CONSOLE
        # ----------------------------------------------------------------------
        if hasattr(self, 'hud_x'):
            hud_panel_rect = pygame.Rect(self.hud_x, self.hud_y, self.hud_w, self.hud_h)
            pygame.draw.rect(self.screen, (255, 255, 255), hud_panel_rect, border_radius=6)
            pygame.draw.rect(self.screen, (205, 212, 222), hud_panel_rect, width=1, border_radius=6)

            lbl_hud_title = self.font_hud_bold.render("APPROXIMATE N20 MODEL & DIFFERENTIAL KINEMATICS", True, (25, 30, 40))
            self.screen.blit(lbl_hud_title, (self.hud_x + 16, self.hud_y + 10))
            pygame.draw.line(self.screen, (230, 234, 242), (self.hud_x + 16, self.hud_y + 30), (self.hud_x + self.hud_w - 16, self.hud_y + 30), 1)

            y_c = self.hud_y + 34
            cur_pt = self.traj_points[self.traj_idx] if self.traj_points else None

            def draw_hud_col(col_x, label, val_str, val_clr=(20, 25, 35)):
                lbl = self.font_hud.render(label, True, (105, 115, 130))
                val = self.font_hud_bold.render(val_str, True, val_clr)
                self.screen.blit(lbl, (col_x, y_c))
                self.screen.blit(val, (col_x, y_c + 16))

            col1_x = self.hud_x + 16
            col2_x = self.hud_x + self.hud_w // 2 + 10

            status_txt = "💥 CRASHED" if self.crashed else ("🏆 TOURNAMENT WON" if self.finished else ("🔄 TRANSITIONING" if self.transition_timer > 0 else ("🚀 RUNNING" if self.running else "⏸ PAUSED")))
            status_clr = (220, 30, 30) if self.crashed else ((40, 167, 69) if self.finished else ((230, 120, 0) if self.transition_timer > 0 else ((0, 120, 215) if self.running else (130, 135, 145))))
            draw_hud_col(col1_x, "Active State:", status_txt, status_clr)
            draw_hud_col(col2_x, "Phase Time / Total:", f"{self.phase_time:.2f}s / {self.lap_time:.2f}s", (0, 110, 200))

            y_c += 36
            cur_v = self.bot.v
            draw_hud_col(col1_x, "Linear Velocity (v):", f"{cur_v:6.1f} mm/s", (0, 140, 210))
            cur_g = cur_pt.g_force if cur_pt else 0.0
            g_clr = (220, 30, 30) if cur_g > 0.7 else ((220, 140, 0) if cur_g > 0.35 else (40, 160, 80))
            draw_hud_col(col2_x, "Lateral G-Load (v²/R):", f"{cur_g:4.2f} G", g_clr)

            y_c += 36
            draw_hud_col(col1_x, "Wheel Speeds (vL / vR):", f"{self.bot.v_left:5.0f} / {self.bot.v_right:5.0f} mm/s")
            omega_deg = math.degrees(self.bot.omega)
            draw_hud_col(col2_x, "Gyro Yaw Rate (ω):", f"{omega_deg:6.1f} °/s")

            y_c += 36
            mode_str = f"[{self.bot.centering_mode}]"
            mode_clr = (40, 160, 80) if "DUAL" in mode_str else ((0, 120, 220) if "SINGLE" in mode_str else ((200, 120, 0) if "HEADING" in mode_str else (180, 50, 50)))
            dL = self.bot.sensor_l90.dist
            dR = self.bot.sensor_r90.dist
            draw_hud_col(col1_x, "Straight Wall Centering:", f"{mode_str} L:{dL:.0f}|R:{dR:.0f}mm", mode_clr)
            cur_cx = int(self.bot.x // CELL_SIZE_MM)
            cur_cy = int(self.bot.y // CELL_SIZE_MM)
            draw_hud_col(col2_x, "Coordinates & Heading:", f"({cur_cx},{cur_cy}) | {math.degrees(self.bot.theta):4.0f}°")

            y_c += 36
            noise_str = "Jitter ±16 ADC | SNR: 32dB" if self.bot.optical_noise_enabled else "OFF (Ideal Raycast)"
            if self.bot.active_glitches > 0:
                noise_str += " ⚠️ GLITCH"
            noise_clr = (220, 60, 60) if self.bot.active_glitches > 0 else ((40, 160, 80) if self.bot.optical_noise_enabled else (120, 125, 135))
            draw_hud_col(col1_x, "Optical Signal & Jitter:", noise_str, noise_clr)
            drift_deg = math.degrees(self.bot.gyro_drift)
            imbal_str = "1.2% Drift" if self.bot.motor_imbalance_enabled else "0% (Ideal)"
            draw_hud_col(col2_x, "IMU Drift & Imbalance:", f"Drift: {drift_deg:+.2f}° | {imbal_str}", (200, 100, 0) if abs(drift_deg) > 1.0 else (0, 120, 200))

            y_c += 36
            sync_str = f"Synced at ({self.bot.last_sync_cell[0]},{self.bot.last_sync_cell[1]})" if self.bot.last_sync_cell != (0, 0) else "Active"
            slip_pct = self.bot.slip_ratio * 100.0
            draw_hud_col(col1_x, "Traction & Post Sync:", f"Slip: {slip_pct:.1f}% | {sync_str}", (40, 160, 80) if not self.bot.sync_active else (255, 160, 0))
            draw_hud_col(col2_x, "Strategy Optimizer:", f"{self.strategy_mode} ({self.best_strategy})", (0, 140, 70) if "AUTO" in self.strategy_mode else (0, 100, 220))

            log_box_y = y_c + 40
            log_box_h = max(35, self.hud_h - (log_box_y - self.hud_y) - 8)
            log_rect = pygame.Rect(self.hud_x + 14, log_box_y, self.hud_w - 28, log_box_h)
            pygame.draw.rect(self.screen, (12, 15, 20), log_rect, border_radius=4)
            pygame.draw.rect(self.screen, (60, 68, 80), log_rect, width=1, border_radius=4)

            y_log = log_box_y + 8
            for msg in self.logs[-5:]:
                clr = (255, 80, 80) if "[COLLISION]" in msg else ((0, 230, 140) if "[MOUSE]" in msg or "[HARDWARE]" in msg else ((255, 190, 40) if "[TOURNAMENT]" in msg or "[DISTURBANCE]" in msg or "[STRAIGHTS]" in msg else (180, 200, 220)))
                lbl = self.font_log.render(msg, True, clr)
                self.screen.blit(lbl, (self.hud_x + 22, y_log))
                y_log += 20

if __name__ == "__main__":
    sim = MMSAdvancedSimulator()
    sim.run()
