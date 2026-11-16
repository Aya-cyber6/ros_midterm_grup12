"""Single velocity controller with LiDAR, sonar and IMU heading feedback."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan, Range
from std_msgs.msg import Float64

from midterm_robot.heading import angle_difference, HeadingController, quaternion_yaw
from midterm_robot.safety import front_distance, range_distance


class ObstacleAvoider(Node):
    """Turn by an IMU-measured angle and stop when any required sensor is lost."""

    def __init__(self):
        super().__init__('obstacle_avoider')
        angle = self.declare_parameter('turn_angle_deg', 45.0).value
        self.heading = HeadingController(float(angle))
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.deviation_pub = self.create_publisher(
            Float64, '/heading_deviation_deg', 10)
        self.lidar = None
        self.sonar = None
        self.yaw = None
        self.reference_yaw = None
        self.lidar_time = float('-inf')
        self.sonar_time = float('-inf')
        self.imu_time = float('-inf')
        self.last_heading_log = float('-inf')
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.sonar_sub = self.create_subscription(
            Range, '/sonar/range', self.sonar_callback, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, qos_profile_sensor_data)
        # Wall-clock timer can stop commands even if simulation time pauses.
        self.timer = self.create_timer(0.05, self.control)
        self.get_logger().info(
            f'Waiting for LiDAR, sonar and IMU; turn angle: {angle:.1f} deg.')

    def scan_callback(self, msg):
        self.lidar = front_distance(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max)
        self.lidar_time = time.monotonic()

    def sonar_callback(self, msg):
        self.sonar = range_distance(msg.range, msg.min_range, msg.max_range)
        self.sonar_time = time.monotonic()

    def imu_callback(self, msg):
        self.imu_time = time.monotonic()
        if msg.orientation_covariance[0] == -1:
            self.yaw = None
            return
        q = msg.orientation
        self.yaw = quaternion_yaw(q.x, q.y, q.z, q.w)
        if self.yaw is None:
            return
        if self.reference_yaw is None:
            self.reference_yaw = self.yaw
        deviation = math.degrees(angle_difference(self.yaw, self.reference_yaw))
        self.deviation_pub.publish(Float64(data=deviation))
        if self.imu_time - self.last_heading_log >= 1.0:
            self.get_logger().info(f'Heading deviation: {deviation:+.1f} deg')
            self.last_heading_log = self.imu_time

    def control(self):
        now = time.monotonic()
        previous_target = self.heading.target_yaw
        linear, angular = self.heading.command(
            self.lidar, self.sonar, self.yaw,
            now - self.lidar_time, now - self.sonar_time, now - self.imu_time)
        if previous_target is None and self.heading.target_yaw is not None:
            angle = math.degrees(self.heading.turn_angle)
            self.get_logger().info(f'Obstacle: starting {angle:+.1f} deg turn.')
        command = Twist()
        command.linear.x = linear
        command.angular.z = angular
        self.publisher_.publish(command)

    def stop_robot(self):
        self.publisher_.publish(Twist())


def main(args=None):
    """Run the controller."""
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
