#!/usr/bin/env python3
"""
Lane Segmentation Node for IGVC
Detects white/yellow lanes using HSV color segmentation
Publishes detected lanes as a point cloud
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2


class LaneDetector(Node):
    def __init__(self):
        super().__init__('lane_detector')
        
        # Parameters
        self.declare_parameter('debug_viz', False)
        self.declare_parameter('min_lane_area', 100)
        self.declare_parameter('max_distance', 10.0)
        
        self.debug_viz = self.get_parameter('debug_viz').value
        self.min_lane_area = self.get_parameter('min_lane_area').value
        self.max_distance = self.get_parameter('max_distance').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera intrinsics (will be updated from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_height = 1.0  # meters above ground (mounted on pole)
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/oak/rgb/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.lane_cloud_pub = self.create_publisher(
            PointCloud2,
            '/lane_cloud',
            10
        )
        
        if self.debug_viz:
            self.debug_image_pub = self.create_publisher(
                Image,
                '/lane_debug',
                10
            )
        
        self.get_logger().info('Lane Detector initialized')
        self.get_logger().info(f'Subscribed to: /oak/rgb/image_raw')
        self.get_logger().info(f'Publishing to: /lane_cloud')
        
    def camera_info_callback(self, msg):
        """Store camera intrinsics"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera intrinsics received')
    
    def detect_white_lanes(self, image):
        """Detect white lanes using HSV thresholding"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # STRONGER white detection: lower threshold for better detection
        lower_white = np.array([0, 0, 200])  # Lowered from 220 to 200
        upper_white = np.array([180, 35, 255])  # Slightly more saturation allowed
        
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        
        # CRITICAL: Exclude green colors! (for green ramp)
        # Green is H: 40-80 in HSV
        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Remove green from white detection
        white_mask = cv2.bitwise_and(white_mask, cv2.bitwise_not(green_mask))
        
        # Also detect bright areas in grayscale (lowered threshold)
        _, bright_mask = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)  # Lowered from 235
        
        # Exclude green from bright mask too
        bright_mask = cv2.bitwise_and(bright_mask, cv2.bitwise_not(green_mask))
        
        # Combine both methods
        white_mask = cv2.bitwise_or(white_mask, bright_mask)
        
        return white_mask
    
    def detect_yellow_lanes(self, image):
        """Detect yellow lanes using HSV thresholding"""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Yellow color range in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        return yellow_mask
    
    def morphological_cleanup(self, mask):
        """Clean up mask using morphological operations"""
        kernel_small = np.ones((3, 3), np.uint8)
        kernel_medium = np.ones((5, 5), np.uint8)
        kernel_large = np.ones((7, 7), np.uint8)
        
        # Remove tiny noise first
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small, iterations=2)
        # Close small gaps in lines
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_medium, iterations=3)
        # Dilate MORE to make lines thicker and more visible
        mask = cv2.dilate(mask, kernel_large, iterations=4)  # Increased from 3 to 4 with larger kernel
        
        return mask
    
    def mask_to_3d_points(self, mask, frame_id):
        """Convert 2D mask pixels to 3D points using inverse perspective mapping"""
        if self.camera_matrix is None:
            return []
        
        # Get pixel coordinates where mask is true
        y_coords, x_coords = np.where(mask > 0)
        
        if len(x_coords) == 0:
            return []
        
        # Sample MORE points for denser barriers
        step = max(1, len(x_coords) // 2000)  # Increased from 1000 to 2000
        x_coords = x_coords[::step]
        y_coords = y_coords[::step]
        
        points = []
        
        # Camera parameters
        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]
        
        # Create DENSE VERTICAL WALLS at lane boundaries in base_link frame
        # Camera mounted at (0.0, 0.0, 1.0) with 30° pitch down (flush with pole)
        import math
        pitch = 0.52  # 30 degrees
        cam_x = 0.0   # Flush with pole
        cam_z = 1.0   # Height
        
        for x, y in zip(x_coords, y_coords):
            # Pixel to normalized image coordinates  
            x_norm = (x - cx) / fx
            y_norm = (y - cy) / fy
            
            # Estimate ground intersection distance
            # Camera looks down at angle, so bottom of image sees closer ground
            # y_norm is positive for pixels below image center
            z_cam = self.camera_height / (y_norm * math.cos(pitch) + 0.001)
            
            # Limit distance
            if z_cam < 0.5 or z_cam > self.max_distance:
                continue
            
            # Lateral offset in camera frame
            x_cam = x_norm * z_cam
            
            # Transform from camera frame to base_link frame
            # Camera pitched down 30°, so rotate points
            # Create TALLER, DENSER vertical walls in BASE_LINK frame
            for z_height in [0.0, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9, 1.05, 1.2]:  # Up to 1.2m high
                # Camera frame to base_link
                x_robot = cam_x + z_cam * math.cos(pitch)  # Forward
                y_robot = -x_cam  # Lateral (camera right = robot left)
                z_robot = z_height  # True vertical height in robot frame
                
                # Add point at this location
                points.append([x_robot, y_robot, z_robot])
                # Add MORE lateral duplicates for thicker wall
                points.append([x_robot, y_robot + 0.03, z_robot])
                points.append([x_robot, y_robot - 0.03, z_robot])
                points.append([x_robot, y_robot + 0.06, z_robot])
                points.append([x_robot, y_robot - 0.06, z_robot])
        
        return points
    
    def image_callback(self, msg):
        """Process image and detect lanes"""
        # Log that we're receiving images (first time only)
        if not hasattr(self, '_first_image'):
            self._first_image = True
            self.get_logger().info(f'📸 Receiving images: {msg.width}x{msg.height}')
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Detect white and yellow lanes
            white_mask = self.detect_white_lanes(cv_image)
            yellow_mask = self.detect_yellow_lanes(cv_image)
            
            # Apply region of interest (focus on lower half where ground is)
            height, width = cv_image.shape[:2]
            roi_mask = np.zeros((height, width), dtype=np.uint8)
            roi_mask[int(height*0.4):height, :] = 255  # Only bottom 60% of image
            
            white_mask = cv2.bitwise_and(white_mask, roi_mask)
            yellow_mask = cv2.bitwise_and(yellow_mask, roi_mask)
            
            # Combine masks
            lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
            
            # Clean up mask
            lane_mask = self.morphological_cleanup(lane_mask)
            
            # Convert to 3D points
            points_3d = self.mask_to_3d_points(lane_mask, msg.header.frame_id)
            
            # Publish point cloud (already in base_link frame)
            if len(points_3d) > 0:
                header = Header()
                header.stamp = msg.header.stamp
                header.frame_id = 'base_link'  # Already transformed to robot frame
                
                cloud_msg = pc2.create_cloud_xyz32(header, points_3d)
                self.lane_cloud_pub.publish(cloud_msg)
                
                # Log detection (every 30 frames)
                if not hasattr(self, '_frame_count'):
                    self._frame_count = 0
                self._frame_count += 1
                if self._frame_count % 30 == 0:
                    self.get_logger().info(f'✓ Detected {len(points_3d)} lane wall points')
            else:
                # Warn when no lanes detected
                if not hasattr(self, '_no_detect_count'):
                    self._no_detect_count = 0
                self._no_detect_count += 1
                if self._no_detect_count % 30 == 0:
                    self.get_logger().warn('⚠ NO LANES DETECTED! Check camera view.')
            
            # Debug visualization
            if self.debug_viz:
                debug_image = cv_image.copy()
                # Use MAGENTA overlay (not green - avoids confusion with green ramp!)
                debug_image[lane_mask > 0] = [255, 0, 255]  # Magenta/Purple overlay
                
                debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
