from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import CompressedImage


class QRScanner(Node):
    def __init__(self):
        super().__init__('qr_scanner')
        
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/camera/image_raw/compressed',
            self.listener_callback, 
            qos_profile
        )
        self.bridge = CvBridge()
        self.detector = cv2.QRCodeDetector()
    
    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(
                f'Image conversion failed: {str(e)}')
            return
        
        data, bbox, _ = self.detector.detectAndDecode(cv_image)
        
        if data:
            self.get_logger().info(f'QR Code detected: {data}')
            if bbox is not None:
                n = len(bbox)
                for i in range(n):
                    # Convert to int
                    pt1 = (int(bbox[i][0][0]), int(bbox[i][0][1]))
                    pt2 = (int(bbox[(i + 1) % n][0][0]),
                           int(bbox[(i + 1) % n][0][1]))
                    cv2.line(cv_image, pt1, pt2, (0, 255, 0), 2)
        else:
            self.get_logger().info('No QR Code detected')
        
        # Always show the window
        cv2.imshow('QR Detection', cv_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = QRScanner()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
