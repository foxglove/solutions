#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import dlib
import numpy as np
from foxglove_msgs.msg import ImageAnnotations, CircleAnnotation, Color, Point2
from imutils.face_utils import FACIAL_LANDMARKS_68_IDXS

# cv_bridge = CvBridge()  # Removed to avoid NumPy compatibility issues
face_detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")
# Colors for different facial regions
COLORS = [
    Color(r=1.0, g=0.0, b=0.0, a=1.0),  # Red
    Color(r=0.0, g=1.0, b=0.0, a=1.0),  # Green
    Color(r=0.0, g=0.0, b=1.0, a=1.0),  # Blue
    Color(r=1.0, g=1.0, b=0.0, a=1.0),  # Yellow
    Color(r=1.0, g=0.0, b=1.0, a=1.0),  # Magenta
    Color(r=0.0, g=1.0, b=1.0, a=1.0),  # Cyan
    Color(r=1.0, g=0.5, b=0.0, a=1.0),  # Orange
    Color(r=0.5, g=0.0, b=1.0, a=1.0),  # Purple
]

class ImageMarkerNode(Node):
    def __init__(self):
        super().__init__('image_marker_node')
        
        # Subscribe to camera images
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        
        # Publish face annotations using Foxglove schema
        self.annotation_publisher = self.create_publisher(
            ImageAnnotations,
            '/face_annotations',
            10
        )
        
        self.get_logger().info('Foxglove face detection node with landmarks started')

    def image_callback(self, msg: Image):
        try:
            # Convert ROS image to OpenCV directly (avoiding cv_bridge NumPy issues)
            if msg.encoding == 'bgr8':
                cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            elif msg.encoding == 'rgb8':
                cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'mono8':
                cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            elif msg.encoding == 'yuv422' or msg.encoding == 'yuv422_yuy2':
                # YUV422 format - extract Y channel
                if msg.encoding == 'yuv422_yuy2':
                    # YUY2 format: YUYVYUYV... - extract Y values (every other byte)
                    yuv_data = np.frombuffer(msg.data, dtype=np.uint8)
                    cv_img = yuv_data[::2].reshape((msg.height, msg.width))  # Extract Y channel
                else:
                    # Standard YUV422 format
                    cv_img = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 2))
                    cv_img = cv_img[:, :, 0]  # Use Y channel
            else:
                self.get_logger().warn(f'Unsupported image encoding: {msg.encoding}')
                return
            
            # Convert to grayscale for face detection
            if len(cv_img.shape) == 3:
                gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
            elif len(cv_img.shape) == 2:
                gray = cv_img
            else:
                self.get_logger().warn(f'Unsupported image shape: {cv_img.shape}')
                return
            
            # Debug: Log image properties
            self.get_logger().info(f'Image: shape={gray.shape}, dtype={gray.dtype}, '
                                 f'min={gray.min()}, max={gray.max()}, '
                                 f'encoding={msg.encoding}')
            
            # Try different detection parameters
            faces = face_detector(gray, 0)  # No upsampling
            if len(faces) == 0:
                # Try with upsampling (more sensitive)
                faces = face_detector(gray, 1)
                self.get_logger().info('Tried with upsampling')
            
            if len(faces) == 0:
                # Try with higher upsampling
                faces = face_detector(gray, 2)
                self.get_logger().info('Tried with higher upsampling')
            
            # Create Foxglove ImageAnnotations
            annotations = ImageAnnotations()
            
            # Process each detected face
            for i, face in enumerate(faces):
                # Add circle annotation for face bounding box
                circle = CircleAnnotation()
                circle.timestamp = msg.header.stamp
                circle.position = Point2(
                    x=float((face.left() + face.right()) / 2.0),
                    y=float((face.top() + face.bottom()) / 2.0)
                )
                circle.diameter = float(max(face.width(), face.height()))
                circle.thickness = 3.0
                circle.fill_color = Color(r=1.0, g=0.0, b=0.0, a=0.3)  # Semi-transparent red
                circle.outline_color = Color(r=1.0, g=0.0, b=0.0, a=1.0)  # Solid red outline
                annotations.circles.append(circle)
                
                # Get facial landmarks for this face
                landmarks = predictor(gray, face).parts()
                
                # Draw landmarks for each facial region
                for region_idx, (name, (start_idx, end_idx)) in enumerate(
                    FACIAL_LANDMARKS_68_IDXS.items()
                ):
                    # Create points for this facial region
                    region_points = [
                        Point2(x=float(p.x), y=float(p.y)) 
                        for p in landmarks[start_idx:end_idx]
                    ]
                    
                    # Add points as individual circles for now (since ImageMarker is not available)
                    for point in region_points:
                        circle = CircleAnnotation()
                        circle.timestamp = msg.header.stamp
                        circle.position = point
                        circle.diameter = 2.0  # Small circles for landmarks
                        circle.thickness = 1.0
                        circle.fill_color = COLORS[region_idx % len(COLORS)]
                        circle.outline_color = COLORS[region_idx % len(COLORS)]
                        annotations.circles.append(circle)
                
                self.get_logger().info(f'Added face {i} with {len(landmarks)} landmarks and {len(FACIAL_LANDMARKS_68_IDXS)} regions')
            
            # Publish annotations
            self.annotation_publisher.publish(annotations)
            self.get_logger().info(f'Published {len(faces)} face annotations with landmarks')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImageMarkerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()