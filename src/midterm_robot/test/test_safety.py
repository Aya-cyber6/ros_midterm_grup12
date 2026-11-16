"""Exercise motion decisions without requiring ROS or Gazebo."""

import math
import unittest

from midterm_robot.safety import front_distance, motion, range_distance


class SafetyTests(unittest.TestCase):
    def test_missing_or_stale_sensor_stops(self):
        for values in [(None, 1, 0, 0), (1, None, 0, 0),
                       (1, 1, 0.6, 0), (1, 1, 0, 0.6)]:
            self.assertEqual(motion(*values), (0.0, 0.0))

    def test_obstacle_from_either_sensor_turns(self):
        self.assertEqual(motion(0.3, 1, 0, 0), (0.0, 0.5))
        self.assertEqual(motion(1, 0.2, 0, 0), (0.0, 0.5))
        self.assertEqual(motion(1, 1, 0, 0), (0.3, 0.0))

    def test_nan_is_invalid_but_infinity_is_clear(self):
        self.assertIsNone(front_distance([math.nan], 0, 0.1, 0.1, 10))
        self.assertEqual(front_distance([math.inf], 0, 0.1, 0.1, 10), 10)
        self.assertIsNone(range_distance(math.nan, 0.05, 2))
        self.assertEqual(range_distance(-math.inf, 0.05, 2), 0)

    def test_scan_angles_select_front_not_array_midpoint(self):
        scan = [2.0] * 360
        scan[0] = 0.2
        scan[180] = 0.1
        self.assertEqual(front_distance(
            scan, 0, math.pi / 180, 0.1, 10), 0.2)

    def test_empty_or_out_of_range_scan_is_invalid(self):
        self.assertIsNone(front_distance([], 0, 0.1, 0.1, 10))
        self.assertIsNone(front_distance([0, 11], 0, 0.1, 0.1, 10))


if __name__ == '__main__':
    unittest.main()
