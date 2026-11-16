"""Exercise collision recovery with synthetic time, sensors and ROS callbacks."""

import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from midterm_robot.heading import HeadingController
from midterm_robot.recovery import CollisionRecovery


class RecoveryTests(unittest.TestCase):
    def command(self, c, now, yaw=0, lidar=2, sonar=2, ages=(0, 0, 0)):
        return c.command(now, lidar, sonar, yaw, *ages)

    def start_turn(self, c, yaw=0):
        c.contact(10)
        self.command(c, 10)
        self.command(c, 10.21)
        self.assertEqual(self.command(c, 11.22, yaw=yaw), (0, 0))
        self.assertEqual(c.state, 'turning')

    def test_idle_leaves_normal_obstacle_control_in_charge(self):
        self.assertIsNone(self.command(CollisionRecovery(), 10))

    def test_stop_reverse_turn_then_resume_in_order(self):
        c = CollisionRecovery()
        self.assertTrue(c.contact(10))
        for t in (10, 10.1, 10.19):
            self.assertEqual(self.command(c, t, lidar=0.2), (0, 0))
        self.assertEqual(self.command(c, 10.21, lidar=0.2), (-0.1, 0))
        self.assertEqual(self.command(c, 11.20), (-0.1, 0))
        self.assertEqual(self.command(c, 11.22, yaw=math.radians(20)), (0, 0))
        self.assertAlmostEqual(math.degrees(c.target_yaw), 65)
        self.assertGreater(self.command(c, 11.3, yaw=math.radians(20))[1], 0)
        self.assertEqual(self.command(c, 12, yaw=math.radians(64)), (0, 0))
        self.assertIsNone(self.command(c, 12.1, yaw=math.radians(64)))

    def test_repeated_contact_does_not_extend_reverse_timer(self):
        c = CollisionRecovery()
        c.contact(10)
        self.command(c, 10.21)
        for t in (10.4, 10.6, 10.8):
            self.assertFalse(c.contact(t))
            self.assertEqual(self.command(c, t), (-0.1, 0))
        self.assertEqual(self.command(c, 11.22), (0, 0))
        self.assertEqual(c.state, 'turning')

    def test_persistent_contact_stops_after_bounded_reverse(self):
        c = CollisionRecovery()
        c.contact(10)
        self.command(c, 10.21)
        c.contact(11.2)
        self.assertEqual(self.command(c, 11.22), (0, 0))
        self.assertEqual(c.state, 'blocked')
        self.assertEqual(self.command(c, 50), (0, 0))

    def test_another_collision_during_turn_stops_without_repeating_backup(self):
        c = CollisionRecovery()
        self.start_turn(c)
        self.assertTrue(c.contact(11.5))
        self.assertEqual(self.command(c, 11.6), (0, 0))
        self.assertEqual(self.command(c, 100), (0, 0))
        self.assertEqual(c.state, 'blocked')

    def test_sensor_loss_during_reverse_blocks_even_after_data_recovers(self):
        for lidar, sonar, yaw, ages in [
                (None, 2, 0, (0, 0, 0)), (2, None, 0, (0, 0, 0)),
                (2, 2, None, (0, 0, 0)), (2, 2, math.nan, (0, 0, 0)),
                (2, 2, 0, (0.6, 0, 0)), (2, 2, 0, (0, 0.6, 0)),
                (2, 2, 0, (0, 0, 0.6))]:
            with self.subTest(lidar=lidar, sonar=sonar, yaw=yaw, ages=ages):
                c = CollisionRecovery()
                c.contact(10)
                self.command(c, 10.21)
                self.assertEqual(c.command(10.3, lidar, sonar, yaw, *ages), (0, 0))
                self.assertEqual(c.state, 'blocked')
                self.assertEqual(self.command(c, 20), (0, 0))

    def test_clock_reset_aborts_recovery(self):
        c = CollisionRecovery()
        c.contact(10)
        self.command(c, 10.21)
        self.assertEqual(self.command(c, 1), (0, 0))
        self.assertEqual(c.state, 'blocked')

    def test_paused_simulation_does_not_use_up_reverse_duration(self):
        c = CollisionRecovery()
        c.contact(10)
        self.command(c, 10.21)
        for _ in range(100):
            self.assertEqual(self.command(c, 10.3), (-0.1, 0))
        self.assertEqual(c.state, 'reversing')
        # The wall-clock sensor watchdog still halts stale streams.
        self.assertEqual(self.command(c, 10.3, ages=(0.6, 0.6, 0.6)), (0, 0))

    def test_stuck_turn_times_out(self):
        c = CollisionRecovery()
        self.start_turn(c)
        self.assertEqual(self.command(c, 22), (0, 0))
        self.assertEqual(c.state, 'blocked')

    def test_turn_wraps_across_180_degrees(self):
        c = CollisionRecovery()
        self.start_turn(c, math.radians(170))
        self.assertAlmostEqual(math.degrees(c.target_yaw), -145)
        self.assertGreater(self.command(c, 11.3, yaw=math.radians(-179))[1], 0)
        self.assertEqual(self.command(c, 12, yaw=math.radians(-145)), (0, 0))
        self.assertEqual(c.state, 'idle')


class TwistStub:
    def __init__(self):
        self.linear = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = SimpleNamespace(x=0.0, y=0.0, z=0.0)


class ControllerCallbackTests(unittest.TestCase):
    """Run the actual node callbacks with test doubles, without a ROS install."""

    @classmethod
    def setUpClass(cls):
        stubs = {
            'rclpy': SimpleNamespace(),
            'rclpy.node': SimpleNamespace(Node=object),
            'rclpy.qos': SimpleNamespace(qos_profile_sensor_data=object()),
            'rclpy.clock': SimpleNamespace(Clock=object),
            'rclpy.clock_type': SimpleNamespace(ClockType=SimpleNamespace(STEADY_TIME=1)),
            'ros_gz_interfaces.msg': SimpleNamespace(Contacts=object),
            'geometry_msgs.msg': SimpleNamespace(Twist=TwistStub),
            'sensor_msgs.msg': SimpleNamespace(Imu=object, LaserScan=object, Range=object),
            'std_msgs.msg': SimpleNamespace(Float64=object),
        }
        source = Path(__file__).parents[1] / 'midterm_robot/obstacle_avoider.py'
        spec = importlib.util.spec_from_file_location('controller_under_test', source)
        module = importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules, stubs):
            spec.loader.exec_module(module)
        cls.module = module

    def setUp(self):
        self.node = object.__new__(self.module.ObstacleAvoider)
        self.node.heading = HeadingController()
        self.node.recovery = CollisionRecovery()
        self.node.publisher_ = Mock()
        self.node.get_logger = Mock(return_value=Mock())
        self.sim_time = 10.0
        clock = SimpleNamespace(now=lambda: SimpleNamespace(
            nanoseconds=int(self.sim_time * 1e9)))
        self.node.get_clock = lambda: clock
        self.node.lidar, self.node.sonar, self.node.yaw = 2, 2, 0
        self.node.lidar_time = self.node.sonar_time = self.node.imu_time = 100.0

    def last_command(self):
        msg = self.node.publisher_.publish.call_args.args[0]
        return msg.linear.x, msg.angular.z

    def control(self, sim_time):
        self.sim_time = sim_time
        with patch.object(self.module.time, 'monotonic', return_value=100.0):
            self.node.control()
        return self.last_command()

    def test_contact_immediately_stops_and_cancels_normal_turn(self):
        self.node.heading.command(0.2, 2, 0, 0, 0, 0)
        self.node.bumper_callback(SimpleNamespace(contacts=[object()]))
        self.assertEqual(self.last_command(), (0, 0))
        self.assertIsNone(self.node.heading.target_yaw)
        self.assertEqual(self.control(10.1), (0, 0))
        self.assertEqual(self.control(10.21), (-0.1, 0))

    def test_empty_contact_array_does_not_start_recovery(self):
        self.node.bumper_callback(SimpleNamespace(contacts=[]))
        self.node.publisher_.publish.assert_not_called()
        self.assertEqual(self.control(10), (0.3, 0))

    def test_front_is_rechecked_after_recovery_turn(self):
        self.node.bumper_callback(SimpleNamespace(contacts=[object()]))
        self.control(10.21)
        self.control(11.22)
        self.node.yaw = math.radians(45)
        self.assertEqual(self.control(12), (0, 0))
        self.node.lidar = 0.2
        linear, angular = self.control(12.1)
        self.assertEqual(linear, 0)
        self.assertGreater(angular, 0)

    def test_contact_during_recovery_turn_immediately_stops(self):
        self.node.bumper_callback(SimpleNamespace(contacts=[object()]))
        self.control(10.21)
        self.control(11.22)
        self.node.bumper_callback(SimpleNamespace(contacts=[object()]))
        self.assertEqual(self.last_command(), (0, 0))
        self.assertEqual(self.control(12), (0, 0))
        self.assertEqual(self.node.recovery.state, 'blocked')


if __name__ == '__main__':
    unittest.main()
