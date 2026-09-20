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
        return t, (ox + t * dx, oy + t * dy)
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

                    # Post clearance test (robot half width 34mm + post 6mm = 40mm)
                    safe = True
                    for px in range(17):
                        for py in range(17):
                            if dist_pt_to_seg((px * 180.0, py * 180.0), pst, ped) < 42.0:
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
                                    if dist_pt_to_seg(pst, w1, w2) < 32.0 or dist_pt_to_seg(ped, w1, w2) < 32.0:
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
            wp_is_diag.append(False)
            waypoints.append(p_end)
            wp_is_diag.append(True)
            
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
                              v_straight=1150.0, v_diag=1350.0, v_curve_max=820.0):
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
            v_curve = min(v_curve_max, math.sqrt(1.2 * 9810.0 * c['R']))
            g_force = (v_curve * v_curve) / (c['R'] * 9810.0)
            omega = (v_curve / c['R']) * (1.0 if c['turn_left'] else -1.0)
            desc_curve = f"Smooth {c['ang_deg']}° {'Left' if c['turn_left'] else 'Right'} Arc"

            for s in range(1, arc_steps + 1):
                f = s / arc_steps
                phi = c['start_phi'] + f * c['sweep']
                px = c['center'][0] + c['R'] * math.cos(phi)
                py = c['center'][1] + c['R'] * math.sin(phi)
                theta_arc = normalize_angle(phi + (math.pi / 2.0 if c['turn_left'] else -math.pi / 2.0))
                points.append(TrajectoryPoint(px, py, theta_arc, v_curve, omega, g_force, desc_curve, True, c['R']))

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
        dec_v = cruise_speed * (1.0 - 0.7 * (f ** 2)) if s > steps - 20 else cruise_speed
        points.append(TrajectoryPoint(px, py, theta_final, dec_v, 0.0, 0.0, desc_last, False, 0.0))

    return points

# ==============================================================================
# 3. PHYSICAL ROBOT SENSORS & DYNAMICS (3 DEGREES OF FREEDOM)
# ==============================================================================
class PhysicalMicromouse:
    def __init__(self, x=90.0, y=90.0, theta=math.pi/2):
        self.x = x
        self.y = y
        self.theta = theta
        self.v = 0.0
        self.omega = 0.0
        self.W = 72.0        # Wheel track width mm
        self.half_w = 34.0   # Half width mm
        self.half_l = 40.0   # Half length mm

        self.v_left = 0.0
        self.v_right = 0.0
        self.wall_centering_enabled = True
        self.drift_bias = 0.0

        self.sensor_left = (90.0, None)
        self.sensor_right = (90.0, None)
        self.sensor_fl = (150.0, None)
        self.sensor_fr = (150.0, None)

    def read_sensors(self, wall_segs, posts):
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)

        pos_L = (self.x - self.half_w * sin_t, self.y + self.half_w * cos_t)
        pos_R = (self.x + self.half_w * sin_t, self.y - self.half_w * cos_t)
        pos_nose = (self.x + self.half_l * cos_t, self.y + self.half_l * sin_t)

        dir_L  = (-sin_t, cos_t)
        dir_R  = (sin_t, -cos_t)
        dir_FL = (math.cos(self.theta + 0.38), math.sin(self.theta + 0.38))
        dir_FR = (math.cos(self.theta - 0.38), math.sin(self.theta - 0.38))

        def cast_ray(orig, rdir, max_range=240.0):
            closest_dist = max_range
            closest_pt = None
            for w1, w2, _ in wall_segs:
                hit = ray_segment_intersect(orig, rdir, w1, w2, max_range)
                if hit and hit[0] < closest_dist:
                    closest_dist = hit[0]
                    closest_pt = hit[1]
            return closest_dist, closest_pt

        self.sensor_left  = cast_ray(pos_L, dir_L)
        self.sensor_right = cast_ray(pos_R, dir_R)
        self.sensor_fl    = cast_ray(pos_nose, dir_FL)
        self.sensor_fr    = cast_ray(pos_nose, dir_FR)

    def step_physics(self, dt, target_pt):
        """Stanley path tracking controller with differential wheel kinematics."""
        dL = self.sensor_left[0]
        dR = self.sensor_right[0]

        dx = target_pt.x - self.x
        dy = target_pt.y - self.y
        e_along = dx * math.cos(target_pt.theta) + dy * math.sin(target_pt.theta)
        e_cross = -dx * math.sin(target_pt.theta) + dy * math.cos(target_pt.theta)

        v_clamp = max(100.0, abs(self.v))
        k_stanley = 4.5
        theta_correction = math.atan2(k_stanley * e_cross, v_clamp)

        sensor_trim = 0.0
        if self.wall_centering_enabled and dL < 125.0 and dR < 125.0 and abs(target_pt.omega) < 0.2:
            sensor_trim = -(dL - dR) * 0.02

        theta_desired = normalize_angle(target_pt.theta + theta_correction)
        e_theta = normalize_angle(theta_desired - self.theta)

        self.omega = target_pt.omega + 16.0 * e_theta + sensor_trim + self.drift_bias
        self.v = target_pt.v + 3.0 * e_along

        self.v_left = self.v - (self.W / 2.0) * self.omega
        self.v_right = self.v + (self.W / 2.0) * self.omega

        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta = normalize_angle(self.theta + self.omega * dt)

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

    def check_collision(self, bx, by, btheta, half_w=34.0, half_l=40.0):
        if bx - half_w < 0 or bx + half_w > 2880.0 or by - half_l < 0 or by + half_l > 2880.0:
            return True, "Arena Perimeter Wall", (bx, by)

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

        for w1, w2, name in self.wall_segs:
            min_wx = min(w1[0], w2[0]) - 6.0
            max_wx = max(w1[0], w2[0]) + 6.0
            min_wy = min(w1[1], w2[1]) - 6.0
            max_wy = max(w1[1], w2[1]) + 6.0
            if bx < min_wx - 45 or bx > max_wx + 45 or by < min_wy - 45 or by > max_wy + 45:
                continue
            for e1, e2 in edges:
                if segments_intersect(e1, e2, w1, w2):
                    return True, name, ((e1[0] + e2[0]) * 0.5, (e1[1] + e2[1]) * 0.5)

        for px, py in self.posts:
            if abs(bx - px) > 48 or abs(by - py) > 48:
                continue
            for e1, e2 in edges:
                if dist_pt_to_seg((px, py), e1, e2) < 6.0:
                    return True, f"Corner Post ({int(px/180)},{int(py/180)})", (px, py)

        return False, None, None

# ==============================================================================
# 5. HIGH-DPI CRISP RESIZABLE MMS SIMULATOR (WITH DUAL VIEW SPLIT SCREEN)
# ==============================================================================
class MMSAdvancedSimulator:
    def __init__(self):
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
            "[SIM] Antigravitieee High-DPI Resizable Simulator ready.",
            "[SIM] Tournament Cycle: Exploration ➔ Return ➔ Speedrun fully automated!",
            "[OPTIMIZER] Intelligent Path Selector: Evaluates Curves vs Diagonals!",
            "[SENSORS] 4 Optical IR Raycast Lasers Active (Left, Right, FL, FR).",
            f"[MAZE] Active Maze: {self.maze_list[self.current_maze_idx]}."
        ]

        # Load Maze
        self.maze = Maze(16, 16)
        self.load_active_maze()

        # Physical Robot & Collider
        start_pt = self.explore_traj[0]
        self.bot = PhysicalMicromouse(start_pt.x, start_pt.y, start_pt.theta)
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

        # 1. Precompute Exploration Trajectory (with Optical Lookahead Diagonals)
        # In real-world robotics, 45° optical IR lasers have direct line-of-sight down open staircases,
        # verifying clearance before entry so the mouse can glide diagonally on the very first run!
        exp_path = generate_floodfill_exploration_path(self.maze, goals)
        exp_wp, exp_diag = extract_safe_waypoints(exp_path, self.maze, allow_diagonals=True)
        self.explore_traj = generate_curve_trajectory(exp_wp, exp_diag, nominal_R=50.0, v_straight=850.0, v_diag=1100.0, v_curve_max=650.0)

        # 2. Precompute Return Trajectory (Center -> (0,0) with high-speed diagonal glide)
        last_cell = exp_path[-1]
        ret_path = dijkstra_fastest_path(self.maze, last_cell[0], last_cell[1], DIR_NORTH, {(0, 0)})
        ret_wp, ret_diag = extract_safe_waypoints(ret_path, self.maze, allow_diagonals=True)
        self.return_traj = generate_curve_trajectory(ret_wp, ret_diag, nominal_R=55.0, v_straight=950.0, v_diag=1200.0, v_curve_max=700.0)

        # 3. Precompute Championship Speedrun Options (Curves vs Diagonals)
        fwd_path = dijkstra_fastest_path(self.maze, 0, 0, DIR_NORTH, goals)
        
        # Option A: Pure Continuous Curves
        fwd_wp_c, fwd_diag_c = extract_safe_waypoints(fwd_path, self.maze, allow_diagonals=False)
        self.speedrun_traj_curves = generate_curve_trajectory(fwd_wp_c, fwd_diag_c, nominal_R=55.0, v_straight=1150.0, v_curve_max=820.0)
        self.metrics_curves = calculate_trajectory_metrics(self.speedrun_traj_curves)

        # Option B: With Diagonals
        fwd_wp_d, fwd_diag_d = extract_safe_waypoints(fwd_path, self.maze, allow_diagonals=True)
        self.speedrun_traj_diags = generate_curve_trajectory(fwd_wp_d, fwd_diag_d, nominal_R=55.0, v_straight=1150.0, v_diag=1350.0, v_curve_max=820.0)
        self.metrics_diags = calculate_trajectory_metrics(self.speedrun_traj_diags)

        # Dynamic Strategy Optimizer: Compare physical times and choose the fastest
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

        # Toolbar Buttons
        self.btn_run_rect = pygame.Rect(16, 8, 65, 30)
        self.btn_pause_rect = pygame.Rect(86, 8, 65, 30)
        self.btn_reset_rect = pygame.Rect(156, 8, 65, 30)
        self.btn_maze_rect = pygame.Rect(226, 8, 125, 30)
        self.btn_mode_rect = pygame.Rect(356, 8, 145, 30)
        self.btn_strat_rect = pygame.Rect(506, 8, 160, 30)
        self.btn_split_rect = pygame.Rect(671, 8, 110, 30)
        self.btn_center_rect = pygame.Rect(786, 8, 105, 30)
        self.btn_drift_rect = pygame.Rect(896, 8, 90, 30)
        self.btn_crash_rect = pygame.Rect(991, 8, 90, 30)
        self.slider_track_rect = pygame.Rect(1096, 19, 95, 8)

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
        self.phase_time = 0.0
        self.finished = False
        self.crashed = False
        self.crash_info = ""
        self.crash_pos = None
        self.trail.clear()
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
                n_sub = 4
                sub_dt = (dt * self.speed_multiplier) / n_sub
                self.lap_time += dt * self.speed_multiplier
                self.phase_time += dt * self.speed_multiplier

                for _ in range(n_sub):
                    # Check if reached final trajectory point
                    if self.traj_idx >= len(self.traj_points) - 15:
                        self.traj_idx = len(self.traj_points) - 1
                        final_pt = self.traj_points[-1]
                        self.bot.x = final_pt.x
                        self.bot.y = final_pt.y
                        self.bot.theta = final_pt.theta
                        self.bot.v = 0.0
                        self.bot.omega = 0.0
                        self.bot.v_left = 0.0
                        self.bot.v_right = 0.0

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

                    # Window search for closest point along trajectory (position + heading metric)
                    best_d = float('inf')
                    best_i = self.traj_idx
                    search_end = min(len(self.traj_points), self.traj_idx + 80)
                    for i in range(self.traj_idx, search_end):
                        dx = self.traj_points[i].x - self.bot.x
                        dy = self.traj_points[i].y - self.bot.y
                        dth = normalize_angle(self.traj_points[i].theta - self.bot.theta)
                        d = dx * dx + dy * dy + (35.0 * dth) ** 2
                        if d < best_d:
                            best_d = d
                            best_i = i
                    self.traj_idx = max(self.traj_idx, best_i)

                    # Lookahead target point (~24mm ahead, 12 samples of 2mm)
                    lookahead_idx = min(len(self.traj_points) - 1, self.traj_idx + 12)
                    target_pt = self.traj_points[lookahead_idx]

                    self.bot.read_sensors(self.collider.wall_segs, self.collider.posts)
                    self.bot.step_physics(sub_dt, target_pt)

                    # Collision Check
                    has_col, hit_name, hit_pt = self.collider.check_collision(self.bot.x, self.bot.y, self.bot.theta)
                    if has_col:
                        self.crashed = True
                        self.running = False
                        self.crash_info = hit_name
                        self.crash_pos = hit_pt
                        self.log(f"[COLLISION] 💥 CRASH! Hit {hit_name} at X={hit_pt[0]:.1f}, Y={hit_pt[1]:.1f} mm!")
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
                lbl_st = self.font_stage.render(f"○ {title}", True, (100, 110, 125))
            self.screen.blit(lbl_st, (sx + (seg_w - lbl_st.get_width()) // 2, self.toolbar_h + 6))

        # ----------------------------------------------------------------------
        # 2. LEFT VIEWPORT: GLOBAL 16x16 OVERHEAD MAZE CANVAS
        # ----------------------------------------------------------------------
        maze_sz = int(16 * self.global_cell_px)
        global_maze_rect = pygame.Rect(self.global_offset_x, self.global_offset_y, maze_sz, maze_sz)
        pygame.draw.rect(self.screen, self.CLR_MAZE_BG, global_maze_rect)

        # Faint cell grid lines & floodfill distance numbers
        for x in range(16):
            for y in range(16):
                px, py = self.world_to_global_screen(x * CELL_SIZE_MM, (y + 1) * CELL_SIZE_MM)
                cell_rect = pygame.Rect(px, py, self.global_cell_px, self.global_cell_px)
                pygame.draw.rect(self.screen, self.CLR_GRID, cell_rect, 1)

                # Center Goal Highlight
                if x in [7, 8] and y in [7, 8]:
                    goal_surf = pygame.Surface((self.global_cell_px, self.global_cell_px), pygame.SRCALPHA)
                    goal_surf.fill((255, 215, 0, 45))
                    self.screen.blit(goal_surf, (px, py))

                # Distance Text
                dist_val = self.maze.distances[x][y]
                if dist_val >= 0:
                    dist_lbl = self.font_cell.render(str(dist_val), True, (80, 90, 105))
                    self.screen.blit(dist_lbl, (px + (self.global_cell_px - dist_lbl.get_width()) // 2, py + (self.global_cell_px - dist_lbl.get_height()) // 2))

        # Walls
        wall_thick = 4
        for x in range(16):
            for y in range(16):
                x0, x1 = x * CELL_SIZE_MM, (x + 1) * CELL_SIZE_MM
                y0, y1 = y * CELL_SIZE_MM, (y + 1) * CELL_SIZE_MM
                if self.maze.has_wall(x, y, DIR_NORTH):
                    pygame.draw.line(self.screen, self.CLR_WALL, self.world_to_global_screen(x0, y1), self.world_to_global_screen(x1, y1), wall_thick)
                if self.maze.has_wall(x, y, DIR_EAST):
                    pygame.draw.line(self.screen, self.CLR_WALL, self.world_to_global_screen(x1, y1), self.world_to_global_screen(x1, y0), wall_thick)
                if self.maze.has_wall(x, y, DIR_SOUTH):
                    pygame.draw.line(self.screen, self.CLR_WALL, self.world_to_global_screen(x0, y0), self.world_to_global_screen(x1, y0), wall_thick)
                if self.maze.has_wall(x, y, DIR_WEST):
                    pygame.draw.line(self.screen, self.CLR_WALL, self.world_to_global_screen(x0, y1), self.world_to_global_screen(x0, y0), wall_thick)

        # Red Square Posts
        post_sz = max(4, int(6 * (self.global_cell_px / 45.0)))
        for x in range(17):
            for y in range(17):
                px, py = self.world_to_global_screen(x * CELL_SIZE_MM, y * CELL_SIZE_MM)
                r = pygame.Rect(px - post_sz // 2, py - post_sz // 2, post_sz, post_sz)
                pygame.draw.rect(self.screen, self.CLR_POST, r)
                self.screen.set_at((int(px), int(py)), (0, 0, 0))

        # Trajectory Trail
        if len(self.trail) > 1:
            for i in range(len(self.trail) - 1):
                p1, p2 = self.trail[i], self.trail[i+1]
                gf = p2[2]
                clr = (255, 60, 50) if gf > 0.7 else ((255, 195, 0) if gf > 0.35 else (0, 220, 255))
                pygame.draw.line(self.screen, clr, (p1[0], p1[1]), (p2[0], p2[1]), 2)

        # Overview Robot on Global Canvas
        bx, by = self.world_to_global_screen(self.bot.x, self.bot.y)
        theta = -self.bot.theta
        bot_w = 70.0 * self.global_scale
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

            def draw_sensor_ray(sensor_data, sensor_orig, beam_color=(0, 240, 120)):
                dist, hit_pt = sensor_data
                o_px, o_py = world_to_cam(sensor_orig[0], sensor_orig[1])
                if hit_pt:
                    h_px, h_py = world_to_cam(hit_pt[0], hit_pt[1])
                    pygame.draw.line(self.screen, beam_color, (o_px, o_py), (h_px, h_py), 2)
                    pygame.draw.circle(self.screen, (255, 255, 255), (int(h_px), int(h_py)), 4)
                    tag_lbl = self.font_sensor.render(f"{dist:.0f}mm", True, (255, 255, 100))
                    self.screen.blit(tag_lbl, (h_px + 6, h_py - 6))

            cos_b = math.cos(self.bot.theta)
            sin_b = math.sin(self.bot.theta)
            pos_L = (self.bot.x - self.bot.half_w * sin_b, self.bot.y + self.bot.half_w * cos_b)
            pos_R = (self.bot.x + self.bot.half_w * sin_b, self.bot.y - self.bot.half_w * cos_b)
            pos_nose = (self.bot.x + self.bot.half_l * cos_b, self.bot.y + self.bot.half_l * sin_b)

            draw_sensor_ray(self.bot.sensor_left, pos_L, (0, 230, 255))
            draw_sensor_ray(self.bot.sensor_right, pos_R, (0, 230, 255))
            draw_sensor_ray(self.bot.sensor_fl, pos_nose, (0, 255, 140))
            draw_sensor_ray(self.bot.sensor_fr, pos_nose, (0, 255, 140))

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

            lbl_cam = self.font_hud_bold.render("DYNAMIC ONBOARD CHASE CAMERA (ZOOM 3.6x) + 4x IR RAYCASTS", True, (0, 140, 220))
            self.screen.blit(lbl_cam, (self.chase_x + 12, self.chase_y + 8))

        # ----------------------------------------------------------------------
        # 4. RIGHT VIEWPORT: TELEMETRY & LIVE MONOSPACE CONSOLE
        # ----------------------------------------------------------------------
        if hasattr(self, 'hud_x'):
            hud_panel_rect = pygame.Rect(self.hud_x, self.hud_y, self.hud_w, self.hud_h)
            pygame.draw.rect(self.screen, (255, 255, 255), hud_panel_rect, border_radius=6)
            pygame.draw.rect(self.screen, (205, 212, 222), hud_panel_rect, width=1, border_radius=6)

            lbl_hud_title = self.font_hud_bold.render("PHYSICAL SENSORS & DIFFERENTIAL KINEMATICS", True, (25, 30, 40))
            self.screen.blit(lbl_hud_title, (self.hud_x + 16, self.hud_y + 10))
            pygame.draw.line(self.screen, (230, 234, 242), (self.hud_x + 16, self.hud_y + 30), (self.hud_x + self.hud_w - 16, self.hud_y + 30), 1)

            y_c = self.hud_y + 36
            cur_pt = self.traj_points[self.traj_idx] if self.traj_points else None

            def draw_hud_col(col_x, label, val_str, val_clr=(20, 25, 35)):
                lbl = self.font_hud.render(label, True, (105, 115, 130))
                val = self.font_hud_bold.render(val_str, True, val_clr)
                self.screen.blit(lbl, (col_x, y_c))
                self.screen.blit(val, (col_x, y_c + 18))

            col1_x = self.hud_x + 16
            col2_x = self.hud_x + self.hud_w // 2 + 10

            status_txt = "💥 CRASHED" if self.crashed else ("🏆 TOURNAMENT WON" if self.finished else ("🔄 TRANSITIONING" if self.transition_timer > 0 else ("🚀 RUNNING" if self.running else "⏸ PAUSED")))
            status_clr = (220, 30, 30) if self.crashed else ((40, 167, 69) if self.finished else ((230, 120, 0) if self.transition_timer > 0 else ((0, 120, 215) if self.running else (130, 135, 145))))
            draw_hud_col(col1_x, "Active State:", status_txt, status_clr)
            draw_hud_col(col2_x, "Phase Time / Total:", f"{self.phase_time:.2f}s / {self.lap_time:.2f}s", (0, 110, 200))

            y_c += 42
            cur_v = self.bot.v
            draw_hud_col(col1_x, "Linear Velocity (v):", f"{cur_v:6.1f} mm/s", (0, 140, 210))
            cur_g = cur_pt.g_force if cur_pt else 0.0
            g_clr = (220, 30, 30) if cur_g > 0.7 else ((220, 140, 0) if cur_g > 0.35 else (40, 160, 80))
            draw_hud_col(col2_x, "Lateral G-Load (v²/R):", f"{cur_g:4.2f} G", g_clr)

            y_c += 42
            draw_hud_col(col1_x, "Wheel Speeds (vL / vR):", f"{self.bot.v_left:5.0f} / {self.bot.v_right:5.0f} mm/s")
            omega_deg = math.degrees(self.bot.omega)
            draw_hud_col(col2_x, "Gyro Yaw Rate (ω):", f"{omega_deg:6.1f} °/s")

            y_c += 42
            dL = self.bot.sensor_left[0]
            dR = self.bot.sensor_right[0]
            center_status = f"L:{dL:4.0f} | R:{dR:4.0f} mm"
            draw_hud_col(col1_x, "IR Wall Clearance:", center_status, (40, 160, 80) if abs(dL - dR) < 20 else (200, 100, 0))
            cur_cx = int(self.bot.x // CELL_SIZE_MM)
            cur_cy = int(self.bot.y // CELL_SIZE_MM)
            draw_hud_col(col2_x, "Coordinates & Heading:", f"({cur_cx},{cur_cy}) | {math.degrees(self.bot.theta):4.0f}°")

            y_c += 42
            t_c = self.metrics_curves.get('time', 0.0)
            t_d = self.metrics_diags.get('time', 0.0)
            strat_val = f"{self.strategy_mode} ({self.best_strategy})"
            strat_clr = (0, 140, 70) if "AUTO" in self.strategy_mode else ((200, 100, 0) if "DIAG" in self.strategy_mode else (0, 100, 220))
            draw_hud_col(col1_x, "Strategy Optimizer:", strat_val, strat_clr)
            draw_hud_col(col2_x, "Curves vs Diagonals:", f"{t_c:.2f}s vs {t_d:.2f}s (Δ {self.time_savings:.2f}s)", (200, 100, 0) if self.best_strategy == "DIAGONALS" else (0, 110, 200))

            log_box_y = y_c + 44
            log_box_h = max(35, self.hud_h - (log_box_y - self.hud_y) - 8)
            log_rect = pygame.Rect(self.hud_x + 14, log_box_y, self.hud_w - 28, log_box_h)
            pygame.draw.rect(self.screen, (12, 15, 20), log_rect, border_radius=4)
            pygame.draw.rect(self.screen, (60, 68, 80), log_rect, width=1, border_radius=4)

            y_log = log_box_y + 8
            for msg in self.logs[-5:]:
                clr = (255, 80, 80) if "[COLLISION]" in msg else ((0, 230, 140) if "[MOUSE]" in msg else ((255, 190, 40) if "[TOURNAMENT]" in msg or "[DISTURBANCE]" in msg else (180, 200, 220)))
                lbl = self.font_log.render(msg, True, clr)
                self.screen.blit(lbl, (self.hud_x + 22, y_log))
                y_log += 20

if __name__ == "__main__":
    sim = MMSAdvancedSimulator()
    sim.run()
