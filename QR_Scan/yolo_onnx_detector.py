#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import onnxruntime as ort
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import os
from ament_index_python.packages import get_package_share_directory
from .coco_classes import COCO_CLASSES


class YOLOONNXDetector(Node):
    def __init__(self):
        super().__init__('yolo_onnx_detector')
        
        # Declare parameters
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('show_window', True)
        self.declare_parameter('input_size', 640)
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter(
            'confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.show_window = self.get_parameter('show_window').value
        self.input_size = self.get_parameter('input_size').value
        
        # Set default model path if not provided
        if not model_path:
            # Try to find the model in the source directory
            try:
                # First try: source workspace location
                workspace_path = os.path.expanduser(
                    '~/cv_ws/src/cv_robotics_tutorial')
                model_path = os.path.join(
                    workspace_path, 'cv_robotics_tutorial',
                    'models', 'yolov8n.onnx')
                if not os.path.exists(model_path):
                    # Second try: installed share directory
                    package_path = get_package_share_directory(
                        'cv_robotics_tutorial')
                    model_path = os.path.join(
                        package_path, 'models', 'yolov8n.onnx')
            except Exception as e:
                self.get_logger().warn(f'Error finding package: {e}')
                # Fallback to absolute path
                model_path = os.path.expanduser(
                    '~/cv_ws/src/cv_robotics_tutorial/cv_robotics_tutorial/models/yolov8n.onnx'
                )
        
        # Check if model exists
        if not os.path.exists(model_path):
            self.get_logger().error(
                f'Model file not found: {model_path}')
            self.get_logger().error(
                'Please ensure yolov8n.onnx is in the models directory')
            raise FileNotFoundError(f'Model not found: {model_path}')
        
        # Load ONNX model
        self.get_logger().info(f'Loading ONNX model from: {model_path}')
        self.session = ort.InferenceSession(
            model_path, providers=['CPUExecutionProvider'])
        self.get_logger().info('ONNX model loaded successfully')
        
        # Get model input details
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [
            output.name for output in self.session.get_outputs()]
        
        # Class names
        self.class_names = COCO_CLASSES
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # QoS Profile
        qos_profile = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        # Subscriber to compressed camera images
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.image_callback,
            qos_profile
        )
        
        # Publisher for detection results
        self.detection_publisher = self.create_publisher(
            CompressedImage,
            '/yolo/detections/compressed',
            qos_profile
        )
        
        self.get_logger().info('YOLO ONNX Detector Node initialized')
    
    def preprocess(self, image):
        """Preprocess image for YOLO model"""
        # Resize image to input size
        input_img = cv2.resize(image, (self.input_size, self.input_size))
        # Convert BGR to RGB
        input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
        # Normalize to [0, 1] and transpose to (1, 3, H, W)
        input_img = input_img.astype(np.float32) / 255.0
        input_img = np.transpose(input_img, (2, 0, 1))
        input_img = np.expand_dims(input_img, axis=0)
        return input_img
    
    def postprocess(self, outputs, original_shape):
        # Process YOLO outputs to get bounding boxes
        predictions = outputs[0]
        # predictions shape: (1, 84, 8400) for YOLOv8
        # Transpose to (8400, 84)
        predictions = np.squeeze(predictions).T
        
        # Extract boxes and scores
        boxes = predictions[:, :4]
        scores = predictions[:, 4:]
        
        # Get class with highest score for each detection
        class_ids = np.argmax(scores, axis=1)
        confidences = np.max(scores, axis=1)
        
        # Filter by confidence threshold
        mask = confidences > self.conf_threshold
        boxes = boxes[mask]
        confidences = confidences[mask]
        class_ids = class_ids[mask]
        
        # Convert from center format to corner format
        # and scale to original image
        orig_h, orig_w = original_shape[:2]
        scale_x = orig_w / self.input_size
        scale_y = orig_h / self.input_size
        
        boxes_xyxy = []
        for box in boxes:
            x_center, y_center, width, height = box
            x1 = int((x_center - width / 2) * scale_x)
            y1 = int((y_center - height / 2) * scale_y)
            x2 = int((x_center + width / 2) * scale_x)
            y2 = int((y_center + height / 2) * scale_y)
            boxes_xyxy.append([x1, y1, x2, y2])
        
        # Apply Non-Maximum Suppression
        if len(boxes_xyxy) > 0:
            indices = cv2.dnn.NMSBoxes(
                boxes_xyxy,
                confidences.tolist(),
                self.conf_threshold,
                self.iou_threshold
            )
            if len(indices) > 0:
                indices = indices.flatten()
                boxes_xyxy = [boxes_xyxy[i] for i in indices]
                confidences = confidences[indices]
                class_ids = class_ids[indices]
            else:
                boxes_xyxy = []
                confidences = np.array([])
                class_ids = np.array([])
        
        return boxes_xyxy, confidences, class_ids
    
    def draw_detections(self, image, boxes, confidences, class_ids):
        # Draw bounding boxes and labels on image
        for box, conf, cls_id in zip(boxes, confidences, class_ids):
            x1, y1, x2, y2 = box
            # Draw rectangle
            color = (0, 255, 0)  # Green
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f'{self.class_names[cls_id]}: {conf:.2f}'
            label_size, _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
            
            # Draw label background
            cv2.rectangle(image, (x1, y1 - label_size[1] - 10),
                         (x1 + label_size[0], y1), color, -1)
            
            # Draw label text
            cv2.putText(image, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        return image
    
    def image_callback(self, msg):
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            original_shape = frame.shape
            
            # Preprocess image
            input_img = self.preprocess(frame)
            
            # Run inference
            outputs = self.session.run(
                self.output_names, {self.input_name: input_img})
            
            # Postprocess outputs
            boxes, confidences, class_ids = self.postprocess(
                outputs, original_shape)
            
            # Draw detections
            annotated_frame = self.draw_detections(
                frame.copy(), boxes, confidences, class_ids)
            
            # Log detections
            if len(boxes) > 0:
                detection_info = []
                for conf, cls_id in zip(confidences, class_ids):
                    label = self.class_names[cls_id]
                    detection_info.append(f"{label}: {conf:.2f}")
                self.get_logger().info(
                    f'Detected: {", ".join(detection_info)}')
            
            # Publish annotated image
            annotated_msg = CompressedImage()
            annotated_msg.header = msg.header
            annotated_msg.format = "jpeg"
            annotated_msg.data = np.array(
                cv2.imencode('.jpg', annotated_frame)[1]).tobytes()
            self.detection_publisher.publish(annotated_msg)
            
            # Show window if enabled
            if self.show_window:
                cv2.imshow('YOLO ONNX Detection', annotated_frame)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(
                f'Error processing image: {str(e)}')
    
    def destroy_node(self):
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YOLOONNXDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
