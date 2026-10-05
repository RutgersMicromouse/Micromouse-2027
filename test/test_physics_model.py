import math
import os
import sys
import unittest

os.environ["SDL_VIDEODRIVER"] = "dummy"
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "sim")))

import curve_sim


class TestPhysicalModel(unittest.TestCase):
    def test_swept_collision_detects_wall_crossed_between_ticks(self):
        maze = curve_sim.Maze()
        maze.set_wall(1, 2, curve_sim.DIR_EAST)
        collider = curve_sim.MMSCollisionEngine(maze)

        collision = collider.check_swept_collision(
            (300.0, 450.0, 0.0), (420.0, 450.0, 0.0))

        self.assertTrue(collision[0])
        self.assertIn("East Wall", collision[1])

    def test_spatial_index_preserves_wall_candidates_at_cell_boundaries(self):
        collider = curve_sim.MMSCollisionEngine(curve_sim.Maze())
        for x, y in ((179.0, 359.0), (180.0, 360.0), (359.0, 541.0), (2879.0, 2879.0)):
            radius = curve_sim.ROBOT_HALF_LENGTH_MM + curve_sim.ROBOT_HALF_WIDTH_MM
            expected = []
            for wall in collider.wall_segs:
                (x1, y1), (x2, y2), _ = wall
                min_x, max_x = min(x1, x2) - 6.0, max(x1, x2) + 6.0
                min_y, max_y = min(y1, y2) - 6.0, max(y1, y2) + 6.0
                if not (x < min_x - radius or x > max_x + radius or
                        y < min_y - radius or y > max_y + radius):
                    expected.append(wall)

            actual = collider.wall_candidates_in_bounds(
                x - radius, x + radius, y - radius, y + radius)
            self.assertTrue(set(expected).issubset(set(actual)))

    def test_indexed_sensor_rays_match_exhaustive_sensor_rays(self):
        maze = curve_sim.Maze()
        maze.set_wall(1, 2, curve_sim.DIR_EAST)
        collider = curve_sim.MMSCollisionEngine(maze)
        exhaustive = curve_sim.PhysicalMicromouse(300.0, 450.0, 0.0, seed=19)
        indexed = curve_sim.PhysicalMicromouse(300.0, 450.0, 0.0, seed=19)

        exhaustive.read_sensors(collider.wall_segs, collider.posts)
        indexed.read_sensors(collider.wall_segs, collider.posts, collider)

        for sensor_name in (
            "sensor_l90", "sensor_l45", "sensor_fl",
            "sensor_fr", "sensor_r45", "sensor_r90",
        ):
            self.assertEqual(
                getattr(indexed, sensor_name).adc,
                getattr(exhaustive, sensor_name).adc,
            )

    def test_motion_reaches_endpoint_without_pose_snapping(self):
        maze = curve_sim.Maze()
        collider = curve_sim.MMSCollisionEngine(maze)
        bot = curve_sim.PhysicalMicromouse(90.0, 450.0, 0.0, seed=7)
        trajectory = [
            curve_sim.TrajectoryPoint(x, 450.0, 0.0, 0.0 if i == 0 else 150.0, 0.0)
            for i, x in enumerate(range(90, 291, 2))
        ]
        trajectory[-1].v = 0.0
        reached_goal = False
        index = 0

        for _ in range(5000):
            index, collision, reached_goal = curve_sim.simulation_tick(
                bot, collider, trajectory, index, curve_sim.CONTROL_DT_S)
            self.assertFalse(collision[0], collision[1])
            if reached_goal:
                break

        self.assertTrue(reached_goal)
        self.assertLessEqual(math.hypot(bot.x - 290.0, bot.y - 450.0), 4.0)
        self.assertLessEqual(abs(bot.v), 12.0)

    def test_terminal_alignment_corrects_heading_without_pose_snapping(self):
        maze = curve_sim.Maze()
        collider = curve_sim.MMSCollisionEngine(maze)
        bot = curve_sim.PhysicalMicromouse(
            1352.34, 1348.97, math.pi / 2.0 + math.radians(6.0), seed=7)
        trajectory = [curve_sim.TrajectoryPoint(
            1350.0, 1350.0, math.pi / 2.0, 0.0, 0.0)]
        reached_goal = False
        index = 0

        for _ in range(5000):
            index, collision, reached_goal = curve_sim.simulation_tick(
                bot, collider, trajectory, index, curve_sim.CONTROL_DT_S)
            self.assertFalse(collision[0], collision[1])
            if reached_goal:
                break

        self.assertTrue(reached_goal)
        self.assertLessEqual(abs(curve_sim.normalize_angle(bot.theta - math.pi / 2.0)),
                             math.radians(3.0))

    def test_motor_velocity_respects_n20_no_load_bound(self):
        bot = curve_sim.PhysicalMicromouse(seed=3, motor_scale=0.9)
        target = curve_sim.TrajectoryPoint(100000.0, 100000.0, 0.0, 900.0, 0.0)

        for _ in range(1000):
            bot.step_physics(curve_sim.CONTROL_DT_S, target)

        self.assertLessEqual(abs(bot.v_left), bot.max_wheel_speed)
        self.assertLessEqual(abs(bot.v_right), bot.max_wheel_speed)

    def test_sensor_noise_is_reproducible_for_a_fixed_seed(self):
        maze = curve_sim.Maze()
        maze.set_wall(0, 0, curve_sim.DIR_NORTH)
        collider = curve_sim.MMSCollisionEngine(maze)
        readings = []
        for _ in range(2):
            bot = curve_sim.PhysicalMicromouse(90.0, 90.0, math.pi / 2.0, seed=19)
            bot.read_sensors(collider.wall_segs, collider.posts)
            readings.append(tuple(
                sensor.adc for sensor in (
                    bot.sensor_l90, bot.sensor_l45, bot.sensor_fl,
                    bot.sensor_fr, bot.sensor_r45, bot.sensor_r90)
            ))

        self.assertEqual(readings[0], readings[1])
        self.assertEqual(len(readings[0]), 6)


if __name__ == "__main__":
    unittest.main(verbosity=2)
