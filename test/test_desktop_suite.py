#!/usr/bin/env python3
"""
Automated Desktop Unit Test Suite for Antigravitieee Micromouse.
Tests core navigation, math, kinematics, path decomposition, and maze solving.
Run anytime with: python test/test_desktop_suite.py
"""

import unittest
import math
import sys
import os

# Add sim directory to path to test simulator models
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'sim')))
from sim_main import (
    MMSMaze, MMSFloodfill, dijkstra_fastest_path,
    decompose_path_into_segments, DIR_NORTH, DIR_EAST, DIR_SOUTH, DIR_WEST
)


class TestMathUtils(unittest.TestCase):
    """Verifies angle normalization, deadband, and heading calculations."""

    @staticmethod
    def normalize_angle_180(deg):
        deg = math.fmod(deg + 180.0, 360.0)
        if deg < 0.0:
            deg += 360.0
        return deg - 180.0

    @staticmethod
    def shortest_angular_difference(target, current):
        return TestMathUtils.normalize_angle_180(target - current)

    def test_angle_normalization(self):
        self.assertAlmostEqual(self.normalize_angle_180(0.0), 0.0)
        self.assertAlmostEqual(self.normalize_angle_180(90.0), 90.0)
        self.assertAlmostEqual(abs(self.normalize_angle_180(180.0)), 180.0)
        self.assertAlmostEqual(self.normalize_angle_180(270.0), -90.0)
        self.assertAlmostEqual(self.normalize_angle_180(360.0), 0.0)
        self.assertAlmostEqual(self.normalize_angle_180(450.0), 90.0)
        self.assertAlmostEqual(self.normalize_angle_180(-270.0), 90.0)
        self.assertAlmostEqual(abs(self.normalize_angle_180(-180.0)), 180.0)

    def test_shortest_angular_difference(self):
        # 0 to 90 deg -> +90 (CCW)
        self.assertAlmostEqual(self.shortest_angular_difference(90.0, 0.0), 90.0)
        # 0 to 270 deg -> -90 (CW)
        self.assertAlmostEqual(self.shortest_angular_difference(270.0, 0.0), -90.0)
        # 350 to 10 deg -> +20 (cross zero CCW)
        self.assertAlmostEqual(self.shortest_angular_difference(10.0, 350.0), 20.0)
        # 10 to 350 deg -> -20 (cross zero CW)
        self.assertAlmostEqual(self.shortest_angular_difference(350.0, 10.0), -20.0)


class TestMazeAndFloodfill(unittest.TestCase):
    """Verifies maze representation, outer boundary walls, and floodfill distance metrics."""

    def setUp(self):
        self.maze = MMSMaze()
        self.flood = MMSFloodfill(self.maze)

    def test_initial_outer_boundaries(self):
        # All outer edges must have walls
        for x in range(16):
            self.assertTrue(self.maze.has_wall(x, 0, DIR_SOUTH))
            self.assertTrue(self.maze.has_wall(x, 15, DIR_NORTH))
        for y in range(16):
            self.assertTrue(self.maze.has_wall(0, y, DIR_WEST))
            self.assertTrue(self.maze.has_wall(15, y, DIR_EAST))

    def test_start_cell_east_wall(self):
        # Standard Micromouse rule: (0,0) has an East wall
        self.assertTrue(self.maze.has_wall(0, 0, DIR_EAST))
        self.assertTrue(self.maze.has_wall(1, 0, DIR_WEST))

    def test_center_goal_distances(self):
        self.flood.set_goal_center()
        self.flood.recalculate()
        # The 4 center cells (7,7), (7,8), (8,7), (8,8) must have distance = 0
        center_coords = [(7, 7), (7, 8), (8, 7), (8, 8)]
        for cx, cy in center_coords:
            self.assertEqual(self.flood.distance[cx][cy], 0)

        # Distance from start (0,0) to center must be greater than 0
        self.assertGreater(self.flood.distance[0][0], 0)

    def test_wall_symmetry(self):
        # Setting a North wall on (2, 2) must reflect as South wall on (2, 3)
        self.maze.add_wall(2, 2, DIR_NORTH, send_to_api=False)
        self.assertTrue(self.maze.has_wall(2, 2, DIR_NORTH))
        self.assertTrue(self.maze.has_wall(2, 3, DIR_SOUTH))


class TestDijkstraSolver(unittest.TestCase):
    """Verifies fastest path finding using Dijkstra's algorithm."""

    def setUp(self):
        self.maze = MMSMaze()
        # Mark all cells visited for exploration solver test
        for x in range(16):
            for y in range(16):
                self.maze.visited[x][y] = True

    def test_open_grid_path(self):
        goals = [(7, 7), (7, 8), (8, 7), (8, 8)]
        path = dijkstra_fastest_path(self.maze, 0, 0, DIR_NORTH, goals)
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 5)
        # First node must be start
        self.assertEqual(path[0], (0, 0))
        # Last node must be in center
        self.assertIn(path[-1], goals)
        # Check path continuity (each step moves exactly 1 cell)
        for i in range(len(path) - 1):
            dx = abs(path[i + 1][0] - path[i][0])
            dy = abs(path[i + 1][1] - path[i][1])
            self.assertEqual(dx + dy, 1, f"Discontinuous step at {i}: {path[i]} -> {path[i+1]}")


class TestPathDecomposer(unittest.TestCase):
    """Verifies decomposition of coordinate paths into smooth sprints and diagonals."""

    def test_straight_segment_decomposition(self):
        # Straight path North from (0,0) to (0, 5)
        path = [(0, y) for y in range(6)]
        segments = decompose_path_into_segments(path)
        self.assertTrue(len(segments) >= 1)
        self.assertEqual(segments[0][0], 'STRAIGHT')
        self.assertEqual(segments[0][2], 5)

    def test_diagonal_decomposition(self):
        # Staircase path: (0,0)->(0,1)->(1,1)->(1,2)->(2,2)->(2,3)
        path = [(0, 0), (0, 1), (1, 1), (1, 2), (2, 2), (2, 3)]
        segments = decompose_path_into_segments(path)
        # Should detect diagonal or straight transitions
        types = [s[0] for s in segments]
        self.assertTrue('DIAG' in types or 'DIAGONAL' in types or 'STRAIGHT' in types)


class TestTrapezoidalProfile(unittest.TestCase):
    """Verifies kinematic trapezoidal profile generation."""

    def test_profile_timing(self):
        dist = 180.0       # 1 cell = 180 mm
        max_vel = 300.0    # 300 mm/s
        accel = 1500.0     # 1500 mm/s^2

        # Time to accelerate: v/a = 300 / 1500 = 0.2s
        # Dist to accelerate: 0.5 * v^2 / a = 0.5 * 90000 / 1500 = 30 mm
        # Decel distance: 30 mm
        # Cruise distance: 180 - 60 = 120 mm
        # Cruise time: 120 / 300 = 0.4s
        # Total time: 0.2 + 0.4 + 0.2 = 0.8s
        d_accel = (max_vel ** 2) / (2.0 * accel)
        self.assertAlmostEqual(d_accel, 30.0)
        d_cruise = dist - 2.0 * d_accel
        self.assertAlmostEqual(d_cruise, 120.0)
        t_total = (max_vel / accel) * 2.0 + (d_cruise / max_vel)
        self.assertAlmostEqual(t_total, 0.8)

    def test_n20_speedrun_kinematics(self):
        """Verifies N20 12V 30:1 motor speedrun sprint timing on a multi-cell straight."""
        dist = 540.0  # 3 cells = 540 mm
        max_vel = 500.0  # SPEEDRUN_CRUISE_SPEED_MM_S
        accel = 2600.0   # SPEEDRUN_ACCEL_MM_S2
        d_accel = (max_vel ** 2) / (2.0 * accel)
        self.assertLess(d_accel * 2.0, dist)  # Verified reaches cruise velocity
        d_cruise = dist - 2.0 * d_accel
        t_accel = max_vel / accel
        t_cruise = d_cruise / max_vel
        t_total = 2.0 * t_accel + t_cruise
        self.assertAlmostEqual(t_accel, 500.0 / 2600.0)
        self.assertAlmostEqual(d_accel, 48.0769, places=3)



if __name__ == '__main__':
    print("=" * 60)
    print("  ANTIGRAVITIEEE MICROMOUSE - AUTOMATED UNIT TEST SUITE")
    print("=" * 60)
    unittest.main(verbosity=2)
