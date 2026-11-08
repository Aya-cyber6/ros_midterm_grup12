#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        # LiDAR verisine abone ol
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # Hareket komutları yayınla
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safe_distance = 0.5 # 50 cm engel mesafesi

    def scan_callback(self, msg):
        twist = Twist()
        # Önümüzdeki (0 derece) lazer ışınlarının minimum mesafesini kontrol et
        front_ranges = msg.ranges[len(msg.ranges)//2 - 20 : len(msg.ranges)//2 + 20]
        min_distance = min(front_ranges) if front_ranges else float('inf')

        if min_distance < self.safe_distance:
            self.get_logger().info('Engel Tespit Edildi! Dönülüyor...')
            twist.linear.x = 0.0
            twist.angular.z = 0.5 # Sağa/Sola dön
        else:
            twist.linear.x = 0.3 # İleri git
            twist.angular.z = 0.0

        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
