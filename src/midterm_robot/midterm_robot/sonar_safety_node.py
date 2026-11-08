#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class SonarSafetyNode(Node):
    def __init__(self):
        super().__init__('sonar_safety_node')
        
        # Sonar/Range Sensör Verisi Dinleme
        self.sonar_sub = self.create_subscription(
            Range,
            '/sonar/range',
            self.sonar_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Sonar Safety Node operational.')

    def sonar_callback(self, msg):
        dist = msg.range
        twist = Twist()
        
        # 0.4 metreye kadar yaklaşan engelde acil kaçınma
        if dist < 0.4:
            self.get_logger().warn(f'SONAR WARNING! Close-range obstacle at {dist:.2f}m. Evading!')
            twist.linear.x = 0.0
            twist.angular.z = 0.6
            self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = SonarSafetyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
