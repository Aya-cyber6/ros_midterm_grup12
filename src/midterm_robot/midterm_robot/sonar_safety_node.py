"""Optional sonar monitor; velocity commands belong to obstacle_avoider."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range

from midterm_robot.safety import range_distance


class SonarSafetyNode(Node):
    """Report nearby obstacles without competing for cmd_vel."""

    def __init__(self):
        super().__init__('sonar_safety_node')
        self.was_close = False
        self.sonar_sub = self.create_subscription(
            Range, '/sonar/range', self.sonar_callback, qos_profile_sensor_data)

    def sonar_callback(self, msg):
        distance = range_distance(msg.range, msg.min_range, msg.max_range)
        close = distance is not None and distance < 0.4
        if close and not self.was_close:
            self.get_logger().warn('Sonar obstacle within 0.4 m.')
        self.was_close = close


def main(args=None):
    """Run the optional monitor."""
    rclpy.init(args=args)
    node = SonarSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
