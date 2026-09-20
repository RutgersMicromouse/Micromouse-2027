import sys, os, math, time
import pygame

# ==============================================================================
# 1. MAZE PARSER & DATA STRUCTURES
# ==============================================================================
class Maze:
    def __init__(self, width=16, height=16):
        self.width = width
        self.height = height
        # walls[x][y] = [N, E, S, W]
        self.walls = [[[False]*4 for _ in range(height)] for _ in range(width)]
        self.visited = [[False]*height for _ in range(width)]
        # Add outer boundary walls
        for x in range(width):
            self.walls[x][0][2] = True
            self.walls[x][height-1][0] = True
        for y in range(height):
            self.walls[0][y][3] = True
            self.walls[width-1][y][1] = True

    def load_num(self, filename):
        if not os.path.exists(filename):
            return False
        with open(filename, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 6:
                    x, y = int(parts[0]), int(parts[1])
                    if 0 <= x < self.width and 0 <= y < self.height:
                        self.walls[x][y][0] = bool(int(parts[2])) # N
                        self.walls[x][y][1] = bool(int(parts[3])) # E
                        self.walls[x][y][2] = bool(int(parts[4])) # S
                        self.walls[x][y][3] = bool(int(parts[5])) # W
        return True

    def has_wall(self, x, y, d):
        if 0 <= x < self.width and 0 <= y < self.height:
            return self.walls[x][y][d]
        return True

    def add_wall(self, x, y, d):
        if 0 <= x < self.width and 0 <= y < self.height:
            self.walls[x][y][d] = True
            dx, dy = [ (0, 1), (1, 0), (0, -1), (-1, 0) ][d]
            nx, ny = x + dx, y + dy
            if 0 <= nx < self.width and 0 <= ny < self.height:
                self.walls[nx][ny][(d + 2) % 4] = True

# ==============================================================================
# 2. CONTINUOUS KINEMATIC TRAJECTORY GENERATOR (SMOOTH CURVES & ARCS)
# ==============================================================================
# Robot state: x (mm), y (mm), theta (rad), v (mm/s), omega (rad/s)
CELL_SIZE_MM = 180.0
HALF_CELL_MM = 90.0

class TrajectoryPoint:
    def __init__(self, x_mm, y_mm, theta_rad, v_mm_s, omega_rad_s, g_force=0.0):
        self.x = x_mm
        self.y = y_mm
        self.theta = theta_rad
        self.v = v_mm_s
        self.omega = omega_rad_s
        self.g_force = g_force

class CurveTrajectoryBuilder:
    """
    Builds continuous smooth curves (Clothoid/Arc 90°, Arc 45°, Slaloms, Straight Sprints)
    Transitions seamlessly without stopping at cell centers.
    """
    def __init__(self):
        self.points = []

    def add_straight(self, start_x, start_y, heading_rad, dist_mm, v_cruise, dt=0.005):
        cos_h = math.cos(heading_rad)
        sin_h = math.sin(heading_rad)
        num_steps = max(1, int(dist_mm / (v_cruise * dt)))
        step_dist = dist_mm / num_steps

        cx, cy = start_x, start_y
        for _ in range(num_steps):
            cx += step_dist * cos_h
            cy += step_dist * sin_h
            self.points.append(TrajectoryPoint(cx, cy, heading_rad, v_cruise, 0.0, 0.0))
        return cx, cy, heading_rad

    def add_arc_turn(self, start_x, start_y, start_theta, sweep_angle_rad, radius_mm, v_cruise, dt=0.005):
        """
        Generates a smooth circular arc turn of radius R sweeping sweep_angle_rad (positive = CCW / Left, negative = CW / Right).
        """
        arc_length = abs(sweep_angle_rad * radius_mm)
        num_steps = max(2, int(arc_length / (v_cruise * dt)))
        d_theta = sweep_angle_rad / num_steps
        omega = sweep_angle_rad / (arc_length / v_cruise)
        g_force = (v_cruise * v_cruise / (radius_mm * 9810.0)) # Centrifugal Gs

        # Determine center of rotation
        # Normal vector to heading: left normal is (-sin, cos)
        turn_dir = 1.0 if sweep_angle_rad > 0 else -1.0
        center_x = start_x - turn_dir * radius_mm * math.sin(start_theta)
        center_y = start_y + turn_dir * radius_mm * math.cos(start_theta)

        # Initial radial angle from center to start point
        init_angle = math.atan2(start_y - center_y, start_x - center_x)

        cx, cy = start_x, start_y
        curr_theta = start_theta
        for s in range(1, num_steps + 1):
            ang = init_angle + d_theta * s
            cx = center_x + radius_mm * math.cos(ang)
            cy = center_y + radius_mm * math.sin(ang)
            curr_theta = start_theta + d_theta * s
            self.points.append(TrajectoryPoint(cx, cy, curr_theta, v_cruise, omega, g_force))

        return cx, cy, curr_theta

print("Trajectory generator initialized successfully.")
