#!/usr/bin/env python3
"""
Lane Segmentation Node for IGVC
Detects white/yellow lanes using HSV color segmentation
Publishes detected lanes as a point cloud
"""

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)

# RViz2 Image display typically requests Reliable; sensor-style Best Effort here causes
# "incompatible QoS ... RELIABILITY_QOS_POLICY" and no frames.
LANE_DEBUG_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped, Vector3Stamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point, do_transform_vector3


class LaneDetector(Node):
    """Projects lane pixels to ground in base_link using TF + pinhole ray."""

    def __init__(self):
        super().__init__('lane_detector')

        # Default true: matches perception.launch.xml; Gazebo camera must use sensor QoS on subs
        self.declare_parameter('debug_viz', True)
        self.declare_parameter('min_lane_area', 100)
        self.declare_parameter('max_distance', 10.0)
        self.declare_parameter('output_frame', 'base_link')
        # base_joint places base_link 0.24 m above base_footprint; ground ~= footprint z in that chain
        self.declare_parameter('ground_z_in_base_link', -0.24)
        self.declare_parameter('tf_lookup_timeout_sec', 0.15)
        self.declare_parameter('horizontal_fov', 1.2217)  # rad, matches oak_d_pro Gazebo camera

        self.debug_viz = self.get_parameter('debug_viz').value
        self.min_lane_area = self.get_parameter('min_lane_area').value
        self.max_distance = self.get_parameter('max_distance').value
        self.output_frame = self.get_parameter('output_frame').value
        self.ground_z_in_base_link = self.get_parameter('ground_z_in_base_link').value
        self.tf_lookup_timeout_sec = self.get_parameter('tf_lookup_timeout_sec').value
        self.horizontal_fov = float(self.get_parameter('horizontal_fov').value)
        self.add_on_set_parameters_callback(self.parameters_callback)

        self._warned_k_fallback = False
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Gazebo (and most camera drivers) publish with sensor QoS (best effort). Default
        # "10" is reliable — no QoS match → no images → /lane_debug never updates in RViz.
        self.image_sub = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/oak/rgb/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.lane_cloud_pub = self.create_publisher(
            PointCloud2,
            '/lane_cloud',
            10,
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            '/lane_debug',
            LANE_DEBUG_QOS,
        )

        self.get_logger().info('Lane Detector initialized (TF ray -> ground plane in %s)' % self.output_frame)
        self.get_logger().info('Subscribed to: /oak/rgb/image_raw')
        self.get_logger().info('Publishing to: /lane_cloud')
        if self.debug_viz:
            self.get_logger().info(
                'Debug viz ON — publishing sensor_msgs/Image on /lane_debug (add Image display in RViz)'
            )
        else:
            self.get_logger().info(
                'Debug viz OFF — no /lane_debug topic (set debug_viz:=true in perception.launch.xml)'
            )

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'debug_viz':
                self.debug_viz = bool(param.value)
                state = 'ON' if self.debug_viz else 'OFF'
                self.get_logger().info(f'Debug viz toggled {state} on /lane_debug')
            elif param.name == 'min_lane_area':
                self.min_lane_area = int(param.value)
            elif param.name == 'max_distance':
                self.max_distance = float(param.value)
            elif param.name == 'output_frame':
                self.output_frame = str(param.value)
            elif param.name == 'ground_z_in_base_link':
                self.ground_z_in_base_link = float(param.value)
            elif param.name == 'tf_lookup_timeout_sec':
                self.tf_lookup_timeout_sec = float(param.value)

        return SetParametersResult(successful=True)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        if not hasattr(self, '_logged_calib') or not self._logged_calib:
            self._logged_calib = True
            self.get_logger().info('Camera intrinsics updated from /oak/rgb/camera_info')

    def _intrinsics_from_image_size(self, width, height):
        fx = (width / 2.0) / math.tan(self.horizontal_fov / 2.0)
        fy = fx
        return np.array([[fx, 0.0, width / 2.0], [0.0, fy, height / 2.0], [0.0, 0.0, 1.0]])

    def detect_white_lanes(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 35, 255])

        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        lower_green = np.array([35, 50, 50])
        upper_green = np.array([85, 255, 255])
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

        white_mask = cv2.bitwise_and(white_mask, cv2.bitwise_not(green_mask))

        _, bright_mask = cv2.threshold(gray, 220, 255, cv2.THRESH_BINARY)
        bright_mask = cv2.bitwise_and(bright_mask, cv2.bitwise_not(green_mask))

        white_mask = cv2.bitwise_or(white_mask, bright_mask)

        return white_mask

    def detect_yellow_lanes(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        return yellow_mask

    def morphological_cleanup(self, mask):
        kernel_small = np.ones((3, 3), np.uint8)
        kernel_medium = np.ones((5, 5), np.uint8)
        kernel_large = np.ones((7, 7), np.uint8)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_small, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_medium, iterations=3)
        mask = cv2.dilate(mask, kernel_large, iterations=4)

        return mask

    def mask_to_3d_points(self, mask, image_header):
        """
        For each mask pixel, cast a ray in the camera optical frame (z forward),
        transform to output_frame, intersect z = ground_z_in_base_link, then
        extrude vertically for costmap obstacles.
        """
        if self.camera_matrix is None:
            return []

        optical_frame = image_header.frame_id
        if not optical_frame:
            self.get_logger().warn_throttle(5.0, 'Image header.frame_id empty; cannot project lanes')
            return []

        y_coords, x_coords = np.where(mask > 0)
        if len(x_coords) == 0:
            return []

        step = max(1, len(x_coords) // 2000)
        x_coords = x_coords[::step]
        y_coords = y_coords[::step]

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        try:
            t = self.tf_buffer.lookup_transform(
                self.output_frame,
                optical_frame,
                Time.from_msg(image_header.stamp),
                timeout=Duration(seconds=self.tf_lookup_timeout_sec),
            )
        except tf2_ros.TransformException as ex:
            try:
                t = self.tf_buffer.lookup_transform(
                    self.output_frame,
                    optical_frame,
                    Time(),
                    timeout=Duration(seconds=self.tf_lookup_timeout_sec),
                )
            except tf2_ros.TransformException as ex2:
                self.get_logger().warn_throttle(
                    5.0,
                    'Lane TF failed (%s); latest also failed (%s)' % (ex, ex2),
                )
                return []

        z_g = self.ground_z_in_base_link
        points = []
        wall_dzs = [0.0, 0.15, 0.3, 0.45, 0.6, 0.75, 0.9, 1.05, 1.2]
        # Thicken in base_link Y (matches previous costmap habit); small XY mix is unnecessary
        y_thick = [0.0, 0.03, -0.03, 0.06, -0.06]

        origin_opt = PointStamped()
        origin_opt.header = image_header
        origin_opt.point.x = 0.0
        origin_opt.point.y = 0.0
        origin_opt.point.z = 0.0
        o_base = do_transform_point(origin_opt, t)
        ox, oy, oz = o_base.point.x, o_base.point.y, o_base.point.z

        for u, v in zip(x_coords, y_coords):
            dx = (float(u) - cx) / fx
            dy = (float(v) - cy) / fy
            dz = 1.0
            norm = math.sqrt(dx * dx + dy * dy + dz * dz)
            dx, dy, dz = dx / norm, dy / norm, dz / norm

            vs = Vector3Stamped()
            vs.header = image_header
            vs.vector.x = dx
            vs.vector.y = dy
            vs.vector.z = dz
            d_base = do_transform_vector3(vs, t)
            ddx = d_base.vector.x
            ddy = d_base.vector.y
            ddz = d_base.vector.z

            if abs(ddz) < 1e-6:
                continue

            lam = (z_g - oz) / ddz
            if lam <= 0.0 or lam > self.max_distance * 1.5:
                continue

            px = ox + lam * ddx
            py = oy + lam * ddy
            pz = oz + lam * ddz

            horiz = math.hypot(px - ox, py - oy)
            if horiz > self.max_distance:
                continue

            for dz_wall in wall_dzs:
                zb = z_g + dz_wall
                for dy in y_thick:
                    points.append([px, py + dy, zb])

        return points

    def image_callback(self, msg):
        if not hasattr(self, '_first_image'):
            self._first_image = True
            self.get_logger().info(f'📸 Receiving images: {msg.width}x{msg.height} frame={msg.header.frame_id!r}')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            if self.camera_matrix is None:
                h, w = cv_image.shape[:2]
                self.camera_matrix = self._intrinsics_from_image_size(w, h)
                self.dist_coeffs = np.zeros(5)
                if not self._warned_k_fallback:
                    self._warned_k_fallback = True
                    self.get_logger().warn(
                        'CameraInfo not received yet — using approximate K from image size + horizontal_fov.'
                    )

            white_mask = self.detect_white_lanes(cv_image)
            yellow_mask = self.detect_yellow_lanes(cv_image)

            height, width = cv_image.shape[:2]
            roi_mask = np.zeros((height, width), dtype=np.uint8)
            roi_mask[int(height * 0.4) : height, :] = 255

            white_mask = cv2.bitwise_and(white_mask, roi_mask)
            yellow_mask = cv2.bitwise_and(yellow_mask, roi_mask)

            lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
            lane_mask = self.morphological_cleanup(lane_mask)

            if self.debug_viz:
                debug_image = cv_image.copy()
                debug_image[lane_mask > 0] = [255, 0, 255]
                # rgb8 avoids RViz2 indexed_8bit_image / sampler conflicts with a second bgr8 panel
                debug_rgb = cv2.cvtColor(debug_image, cv2.COLOR_BGR2RGB)
                debug_msg = self.bridge.cv2_to_imgmsg(debug_rgb, encoding='rgb8')
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)

            points_3d = self.mask_to_3d_points(lane_mask, msg.header)

            if len(points_3d) > 0:
                header = Header()
                header.stamp = msg.header.stamp
                header.frame_id = self.output_frame

                cloud_msg = pc2.create_cloud_xyz32(header, points_3d)
                self.lane_cloud_pub.publish(cloud_msg)

                if not hasattr(self, '_frame_count'):
                    self._frame_count = 0
                self._frame_count += 1
                if self._frame_count % 30 == 0:
                    self.get_logger().info(f'✓ Detected {len(points_3d)} lane wall points')
            else:
                if not hasattr(self, '_no_detect_count'):
                    self._no_detect_count = 0
                self._no_detect_count += 1
                if self._no_detect_count % 30 == 0:
                    self.get_logger().warn('⚠ NO LANES DETECTED! Check camera view / TF / ground_z.')

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
