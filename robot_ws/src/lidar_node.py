#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 512000)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.ser = serial.Serial(port, baud, timeout=0.1)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.02, self.read_scan)  # 50 Hz
        self.get_logger().info('Lidar node started')

    def read_scan(self):
        # 假设雷达自带 SDK 输出标准 LaserScan 格式 JSON 一行
        line = self.ser.readline().decode().strip()
        if not line:
            return
        try:
            import json
            obj = json.loads(line)
            scan = LaserScan()
            scan.header.stamp = self.get_clock().now().to_msg()
            scan.header.frame_id = 'laser'
            scan.angle_min = obj['angle_min']
            scan.angle_max = obj['angle_max']
            scan.angle_increment = obj['angle_increment']
            scan.range_min = 0.02
            scan.range_max = 8.0
            scan.ranges = obj['ranges']
            self.pub.publish(scan)
        except Exception as e:
            self.get_logger().debug(f'skip bad line: {e}')

def main():
    rclpy.init()
    node = LidarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()