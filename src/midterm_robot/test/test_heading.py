"""Test heading feedback with simulated orientation samples, without ROS."""

import math
import unittest

from midterm_robot.heading import angle_difference, HeadingController, quaternion_yaw


class HeadingTests(unittest.TestCase):
    def command(self, controller, degrees, lidar=2.0, sonar=2.0, ages=(0, 0, 0)):
        yaw = None if degrees is None else math.radians(degrees)
        return controller.command(lidar, sonar, yaw, *ages)

    def test_quaternion_yaw_and_normalization(self):
        self.assertAlmostEqual(quaternion_yaw(0, 0, 0, 1), 0)
        for degrees in (-135, -45, 45, 135):
            half = math.radians(degrees) / 2
            actual = quaternion_yaw(0, 0, 2 * math.sin(half), 2 * math.cos(half))
            self.assertAlmostEqual(math.degrees(actual), degrees)

    def test_invalid_orientation_is_rejected(self):
        self.assertIsNone(quaternion_yaw(0, 0, 0, 0))
        self.assertIsNone(quaternion_yaw(0, 0, math.nan, 1))
        self.assertIsNone(quaternion_yaw(0, 0, 0, math.inf))

    def test_deviation_wraps_across_180(self):
        self.assertAlmostEqual(math.degrees(angle_difference(
            math.radians(-179), math.radians(179))), 2.0)
        self.assertAlmostEqual(math.degrees(angle_difference(
            math.radians(179), math.radians(-179))), -2.0)

    def test_clear_front_drives_without_turning(self):
        self.assertEqual(self.command(HeadingController(), 30), (0.3, 0.0))

    def test_target_is_relative_and_persists_after_front_clears(self):
        c = HeadingController()
        self.assertEqual(self.command(c, 20, lidar=0.3), (0.0, 0.5))
        self.assertAlmostEqual(math.degrees(c.target_yaw), 65)
        linear, angular = self.command(c, 40)
        self.assertEqual(linear, 0)
        self.assertGreater(angular, 0)
        self.assertAlmostEqual(math.degrees(c.target_yaw), 65)
        self.assertEqual(self.command(c, 64), (0.0, 0.0))
        self.assertIsNone(c.target_yaw)
        self.assertEqual(self.command(c, 64), (0.3, 0.0))

    def test_turn_crosses_angle_wrap(self):
        c = HeadingController()
        self.command(c, 170, sonar=0.2)
        self.assertAlmostEqual(math.degrees(c.target_yaw), -145)
        self.assertGreater(self.command(c, -170)[1], 0)
        self.assertEqual(self.command(c, -145), (0, 0))

    def test_configured_right_turn(self):
        c = HeadingController(-30)
        self.assertLess(self.command(c, 10, sonar=0.2)[1], 0)
        self.assertAlmostEqual(math.degrees(c.target_yaw), -20)
        self.assertEqual(self.command(c, -20), (0, 0))

    def test_missing_or_stale_imu_stops_and_cancels(self):
        for yaw, ages in [(None, (0, 0, 0)), (20, (0, 0, 0.6))]:
            c = HeadingController()
            self.command(c, 0, lidar=0.3)
            self.assertEqual(self.command(c, yaw, ages=ages), (0, 0))
            self.assertIsNone(c.target_yaw)

    def test_stale_or_invalid_range_stops_during_turn(self):
        for lidar, sonar, ages in [(None, 2, (0, 0, 0)), (2, None, (0, 0, 0)),
                                   (2, 2, (0.6, 0, 0)), (2, 2, (0, 0.6, 0))]:
            c = HeadingController()
            self.command(c, 0, lidar=0.3)
            self.assertEqual(self.command(c, 20, lidar, sonar, ages), (0, 0))
            self.assertIsNone(c.target_yaw)

    def test_persistent_obstacle_is_checked_after_completed_turn(self):
        c = HeadingController()
        self.command(c, 0, lidar=0.3)
        self.assertEqual(self.command(c, 45, lidar=0.3), (0, 0))
        self.assertGreater(self.command(c, 45, lidar=0.3)[1], 0)
        self.assertAlmostEqual(math.degrees(c.target_yaw), 90)

    def test_feedback_slows_and_converges_without_reverse_motion(self):
        c = HeadingController()
        yaw = 0.0
        initial_speed = c.command(0.3, 2, yaw, 0, 0, 0)[1]
        speeds = []
        for _ in range(200):
            linear, angular = c.command(2, 2, yaw, 0, 0, 0)
            self.assertEqual(linear, 0)
            speeds.append(abs(angular))
            yaw += angular * 0.05
            if c.target_yaw is None:
                break
        self.assertIsNone(c.target_yaw)
        self.assertLessEqual(abs(math.degrees(yaw) - 45), 3)
        self.assertTrue(any(0 < speed < initial_speed for speed in speeds))

    def test_bad_turn_parameters_rejected(self):
        for value in (0, 4, 180, math.nan, math.inf):
            with self.assertRaises(ValueError):
                HeadingController(value)


if __name__ == '__main__':
    unittest.main()
