from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage


class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/camera/image_raw/compressed', 
            qos_profile
        )
        self.bridge = CvBridge()
        
        # 0 for default webcam
        self.cap = cv2.VideoCapture(0)
        # Fix 1: Minimal buffer
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        if not self.cap.isOpened():
            self.get_logger().error('Cannot open camera')
            rclpy.shutdown()
            
        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tobytes()
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn('Failed to capture frame')


def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    rclpy.spin(node)
    node.cap.release()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
