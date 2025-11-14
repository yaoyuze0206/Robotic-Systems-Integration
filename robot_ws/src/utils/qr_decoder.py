#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import pyzbar.pyzbar as pyzbar

class QRDecoder(Node):
    def __init__(self):
        super().__init__('qr_decoder')
        self.sub = self.create_subscription(Image, '/image_raw', self.img_cb, 10)
        self.pub = self.create_publisher(String, '/qr_result', 10)
        self.bridge = CvBridge()
        self.get_logger().info('QR decoder ready')

    def img_cb(self, img):
        frame = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        for d in pyzbar.decode(frame):
            self.pub.publish(String(data=d.data.decode()))

def main():
    rclpy.init()
    node = QRDecoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()