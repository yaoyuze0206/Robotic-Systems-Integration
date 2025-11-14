#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Float32
import serial
import threading
import math
import tf_transformations

class ESP32Interface(Node):
    def __init__(self):
        super().__init__('esp32_interface')
        self.declare_parameter('port', '/dev/ttyESP32')
        self.declare_parameter('baud', 921600)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.ser = serial.Serial(port, baud, timeout=0.1)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.bat_pub  = self.create_publisher(Float32, '/battery', 10)
        self.cmd_sub  = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)
        self.lift_sub = self.create_subscription(Int16, '/elevator_height', self.lift_cb, 10)

        thread = threading.Thread(target=self.read_loop, daemon=True)
        thread.start()
        self.get_logger().info('ESP32 桥接已启动')

    # ---------- 下行 ----------
    def cmd_cb(self, msg: Twist):
        line = f"$CMD,{msg.linear.x:.3f},{msg.linear.y:.3f},{msg.angular.z:.3f}\r\n"
        self.ser.write(line.encode())

    def lift_cb(self, msg: Int16):
        angle = max(0, min(90, msg.data))
        line = f"$LFT,{angle}\r\n"
        self.ser.write(line.encode())
        self.get_logger().info(f'升降 {angle}°')

    # ---------- 上行 ----------
    def read_loop(self):
        while rclpy.ok():
            line = self.ser.readline().decode().strip()
            if line.startswith('$ODO'):
                self.handle_odom(line)
            elif line.startswith('$BAT'):
                self.handle_bat(line)

    def handle_odom(self, line):
        try:
            _, x, y, th, vx, vy, wz = line.split(',')
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id  = 'base_link'
            odom.pose.pose.position.x = float(x)
            odom.pose.pose.position.y = float(y)
            q = tf_transformations.quaternion_from_euler(0, 0, float(th))
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            odom.twist.twist.linear.x  = float(vx)
            odom.twist.twist.linear.y  = float(vy)
            odom.twist.twist.angular.z = float(wz)
            self.odom_pub.publish(odom)
        except Exception as e:
            self.get_logger().error(f'odo 解析失败: {e}')

    def handle_bat(self, line):
        try:
            _, v = line.split(',')
            self.bat_pub.publish(Float32(data=float(v)))
        except Exception as e:
            self.get_logger().error(f'bat 解析失败: {e}')

def main():
    rclpy.init()
    node = ESP32Interface()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()