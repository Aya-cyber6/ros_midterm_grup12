"""Single owner of velocity commands using LiDAR and sonar."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range

from midterm_robot.safety import front_distance, motion, range_distance


class ObstacleAvoider(Node):
    """Combine sensors and stop if either data stream is lost."""

    def __init__(self):
        super().__init__('obstacle_avoider')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar = None
        self.sonar = None
        self.lidar_time = float('-inf')
        self.sonar_time = float('-inf')
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.sonar_sub = self.create_subscription(
            Range, '/sonar/range', self.sonar_callback, qos_profile_sensor_data)
        # Wall-clock timer continues to stop commands when simulation pauses.
        self.timer = self.create_timer(0.05, self.control)
        self.get_logger().info('Waiting for LiDAR and sonar data.')

    def scan_callback(self, msg):
        self.lidar = front_distance(
            msg.ranges, msg.angle_min, msg.angle_increment,
            msg.range_min, msg.range_max)
        self.lidar_time = time.monotonic()

    def sonar_callback(self, msg):
        self.sonar = range_distance(msg.range, msg.min_range, msg.max_range)
        self.sonar_time = time.monotonic()

    def control(self):
        now = time.monotonic()
        linear, angular = motion(
            self.lidar, self.sonar,
            now - self.lidar_time, now - self.sonar_time)
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
