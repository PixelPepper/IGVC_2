#!/usr/bin/env python3
"""
Point Cloud Fusion Node
Fuses LiDAR scan (2D), OAK-D Pro depth point cloud, and lane detection point cloud.
All data is converted to `output_frame` (default base_link) before concat so Nav2
sees a geometrically consistent /fused_cloud and /fused_scan.
"""

import math
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header
from tf2_sensor_msgs import do_transform_cloud


class PointCloudFusion(Node):
    def __init__(self):
        super().__init__('pointcloud_fusion')

        # Parameters
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('lane_weight', 10.0)  # Higher weight for lane obstacles
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('tf_lookup_timeout_sec', 0.2)

        self.output_frame = self.get_parameter('output_frame').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.lane_weight = int(self.get_parameter('lane_weight').get_parameter_value().double_value)
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        tf_sec = self.get_parameter('tf_lookup_timeout_sec').get_parameter_value().double_value
        self._tf_timeout = Duration(seconds=0, nanoseconds=int(tf_sec * 1e9))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.latest_scan = None
        self.latest_depth_cloud = None
        self.latest_lane_cloud = None

        self.scan_sub = self.create_subscription(
            LaserScan, '/hokuyo_scan', self.scan_callback, qos
        )
        self.depth_cloud_sub = self.create_subscription(
            PointCloud2, '/oak/stereo/points', self.depth_cloud_callback, qos
        )
        self.lane_cloud_sub = self.create_subscription(
            PointCloud2, '/lane_cloud', self.lane_cloud_callback, qos
        )

        self.fused_cloud_pub = self.create_publisher(PointCloud2, '/fused_cloud', 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self.fusion_callback)

        self.get_logger().info('Point Cloud Fusion initialized')
        self.get_logger().info(
            f'Fusing to frame "{self.output_frame}"; '
            'sub: /hokuyo_scan, /oak/stereo/points, /lane_cloud -> /fused_cloud'
        )

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def depth_cloud_callback(self, msg: PointCloud2):
        self.latest_depth_cloud = msg

    def lane_cloud_callback(self, msg: PointCloud2):
        self.latest_lane_cloud = msg

    def _cloud_time(self, msg):
        t = msg.header.stamp
        if t.sec == 0 and t.nanosec == 0:
            return self.get_clock().now()
        return Time.from_msg(t)

    def _transform_cloud_to_output(self, cloud: PointCloud2) -> np.ndarray:
        """Return Nx3 points in self.output_frame, or empty on failure."""
        if cloud is None or len(cloud.data) == 0:
            return np.array([]).reshape(0, 3)
        try:
            trans = self._tf_buffer.lookup_transform(
                self.output_frame,
                cloud.header.frame_id,
                self._cloud_time(cloud),
                timeout=self._tf_timeout,
            )
            out = do_transform_cloud(cloud, trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF: could not transform cloud from "{cloud.header.frame_id}" to '
                f'"{self.output_frame}": {e}'
            )
            return np.array([]).reshape(0, 3)

        return self._extract_points_from_cloud(out)

    def laserscan_to_pointcloud(self, scan_msg: LaserScan) -> np.ndarray:
        points = []
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0
                points.append([x, y, z])
            angle += scan_msg.angle_increment
        return np.array(points) if points else np.array([]).reshape(0, 3)

    def _scan_to_cloud_in_scan_frame(self, scan_msg: LaserScan, pts: np.ndarray) -> PointCloud2:
        header = Header()
        header.stamp = scan_msg.header.stamp
        header.frame_id = scan_msg.header.frame_id
        return pc2.create_cloud_xyz32(header, pts.tolist())

    def _extract_points_from_cloud(self, cloud_msg: PointCloud2) -> np.ndarray:
        points = []
        for point in pc2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = point
            distance = math.sqrt(x * x + y * y + z * z)
            if distance < self.max_range:
                points.append([x, y, z])
        return np.array(points) if points else np.array([]).reshape(0, 3)

    def fusion_callback(self):
        all_points = []

        if self.latest_scan is not None:
            try:
                raw_lidar = self.laserscan_to_pointcloud(self.latest_scan)
                if len(raw_lidar) == 0:
                    self.get_logger().debug('Skipping LiDAR: no valid ranges')
                else:
                    cloud_scan = self._scan_to_cloud_in_scan_frame(
                        self.latest_scan, raw_lidar
                    )
                    scan_pts = self._transform_cloud_to_output(cloud_scan)
                    if len(scan_pts) > 0:
                        all_points.append(scan_pts)
                        self.get_logger().debug(
                            f'Added {len(scan_pts)} LiDAR points in {self.output_frame}'
                        )
            except Exception as e:
                self.get_logger().error(f'Error processing LiDAR: {e}')

        if self.latest_lane_cloud is not None:
            try:
                lane_points = self._transform_cloud_to_output(self.latest_lane_cloud)
                if len(lane_points) > 0:
                    n_rep = max(1, self.lane_weight)
                    weighted = np.tile(lane_points, (n_rep, 1))
                    all_points.append(weighted)
                    self.get_logger().debug(
                        f'Added {len(lane_points)} lane points (x{n_rep} weight) in {self.output_frame}'
                    )
            except Exception as e:
                self.get_logger().error(f'Error processing lane cloud: {e}')

        if not all_points:
            self.get_logger().debug('No fused data this tick')
            return

        try:
            fused_points = np.vstack(all_points)
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.output_frame
            fused_cloud = pc2.create_cloud_xyz32(header, fused_points.tolist())
            self.fused_cloud_pub.publish(fused_cloud)
            self.get_logger().debug(f'Published {len(fused_points)} fused points in {self.output_frame}')
        except Exception as e:
            self.get_logger().error(f'Error creating fused cloud: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
