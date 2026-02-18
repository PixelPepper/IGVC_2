#!/usr/bin/env python3
"""
Point Cloud Fusion Node
Fuses LiDAR scan (2D), OAK-D Pro depth point cloud, and lane detection point cloud
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
from std_msgs.msg import Header
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import math


class PointCloudFusion(Node):
    def __init__(self):
        super().__init__('pointcloud_fusion')
        
        # Parameters
        self.declare_parameter('output_frame', 'base_link')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('lane_weight', 10.0)  # Higher weight for lane obstacles
        self.declare_parameter('max_range', 10.0)
        
        self.output_frame = self.get_parameter('output_frame').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.lane_weight = self.get_parameter('lane_weight').value
        self.max_range = self.get_parameter('max_range').value
        
        # QoS Profile
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Storage for latest data
        self.latest_scan = None
        self.latest_depth_cloud = None
        self.latest_lane_cloud = None
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/hokuyo_scan',
            self.scan_callback,
            qos
        )
        
        self.depth_cloud_sub = self.create_subscription(
            PointCloud2,
            '/oak/stereo/points',
            self.depth_cloud_callback,
            qos
        )
        
        self.lane_cloud_sub = self.create_subscription(
            PointCloud2,
            '/lane_cloud',
            self.lane_cloud_callback,
            qos
        )
        
        # Publisher
        self.fused_cloud_pub = self.create_publisher(
            PointCloud2,
            '/fused_cloud',
            10
        )
        
        # Timer for periodic fusion
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.fusion_callback
        )
        
        self.get_logger().info('Point Cloud Fusion initialized')
        self.get_logger().info(f'Subscribing to: /hokuyo_scan, /oak/stereo/points, /lane_cloud')
        self.get_logger().info(f'Publishing fused cloud to: /fused_cloud')
    
    def scan_callback(self, msg):
        """Store latest laser scan"""
        self.latest_scan = msg
    
    def depth_cloud_callback(self, msg):
        """Store latest depth point cloud"""
        self.latest_depth_cloud = msg
    
    def lane_cloud_callback(self, msg):
        """Store latest lane point cloud"""
        self.latest_lane_cloud = msg
    
    def laserscan_to_pointcloud(self, scan_msg):
        """Convert LaserScan to point cloud in scan's native frame"""
        points = []
        
        angle = scan_msg.angle_min
        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                # Convert polar to Cartesian
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                z = 0.0  # 2D LiDAR at fixed height
                points.append([x, y, z])
            
            angle += scan_msg.angle_increment
        
        return np.array(points) if points else np.array([]).reshape(0, 3)
    
    def extract_points_from_cloud(self, cloud_msg):
        """Extract XYZ points from PointCloud2 in cloud's native frame"""
        points = []
        
        for point in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = point
            
            # Filter by range
            distance = math.sqrt(x*x + y*y + z*z)
            if distance < self.max_range:
                points.append([x, y, z])
        
        return np.array(points) if points else np.array([]).reshape(0, 3)
    
    def fusion_callback(self):
        """
        Fuse all point clouds
        Publishes in hokuyo_link frame (most stable)
        """
        all_points = []
        
        # 1. Add LiDAR scan points (base sensor)
        if self.latest_scan is not None:
            try:
                lidar_points = self.laserscan_to_pointcloud(self.latest_scan)
                if len(lidar_points) > 0:
                    all_points.append(lidar_points)
                    self.get_logger().debug(f'Added {len(lidar_points)} LiDAR points')
            except Exception as e:
                self.get_logger().error(f'Error processing LiDAR: {e}')
        
        # 2. Add lane detection points (CRITICAL - these are the lane boundaries!)
        if self.latest_lane_cloud is not None:
            try:
                lane_points = self.extract_points_from_cloud(self.latest_lane_cloud)
                if len(lane_points) > 0:
                    # Replicate lane points to increase their weight in costmap
                    weighted_lane_points = np.tile(lane_points, (int(self.lane_weight), 1))
                    all_points.append(weighted_lane_points)
                    self.get_logger().debug(f'Added {len(lane_points)} lane points (x{int(self.lane_weight)} weight)')
            except Exception as e:
                self.get_logger().error(f'Error processing lane cloud: {e}')
        
        # Publish fused cloud
        if all_points:
            try:
                fused_points = np.vstack(all_points)
                
                # Use hokuyo_link frame (most stable, Nav2 will transform)
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = 'hokuyo_link'  # Publish in LiDAR frame
                
                fused_cloud = pc2.create_cloud_xyz32(header, fused_points.tolist())
                self.fused_cloud_pub.publish(fused_cloud)
                
                self.get_logger().debug(f'Published {len(fused_points)} fused points')
                
            except Exception as e:
                self.get_logger().error(f'Error creating fused cloud: {e}')
        else:
            self.get_logger().debug('No data available for fusion')


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
