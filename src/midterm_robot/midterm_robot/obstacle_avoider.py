import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd = Twist()

    def scan_callback(self, msg):
        # LiDAR'ın ön kısmındaki 40 derecelik açıyı kontrol et
        center_index = len(msg.ranges) // 2
        front_ranges = msg.ranges[center_index - 20 : center_index + 20]
        
        # Sonsuz (inf) değerleri filtrele
        valid_ranges = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]
        min_distance = min(valid_ranges) if valid_ranges else 10.0

        if min_distance < 0.5:
            self.get_logger().info('Engel Tespit Edildi! Dönülüyor...')
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.5  # Dönmeye başla
        else:
            self.cmd.linear.x = 0.5   # İleri git
            self.cmd.angular.z = 0.0  # Düzeltme: Dönüşü durdur, düz git

        self.publisher_.publish(self.cmd)

    def stop_robot(self):
        # Düğüm kapatıldığında robotu durdur
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publisher_.publish(self.cmd)
        self.get_logger().info('Robot durduruldu.')

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
