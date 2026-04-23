#!/usr/bin/env python3
"""Publish nav_msgs/Path from a YAML waypoint list (odom frame) for RViz."""

from math import cos, sin

import yaml
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = sin(yaw * 0.5)
    q.w = cos(yaw * 0.5)
    return q


class WaypointPathViz(Node):
    def __init__(self):
        super().__init__('waypoint_path_viz')
        # use_sim_time comes from launch (perception.launch.xml); do not declare here
        # or ParameterAlreadyDeclaredException breaks the node.
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('topic', 'igvc_course/waypoints')
        self.declare_parameter('publish_rate_hz', 1.0)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        topic = self.get_parameter('topic').get_parameter_value().string_value
        self._pub = self.create_publisher(Path, topic, qos)

        rate = max(0.1, self.get_parameter('publish_rate_hz').get_parameter_value().double_value)
        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        self._path = None
        self._load_waypoints()

    def _load_waypoints(self) -> None:
        path_str = self.get_parameter('waypoints_file').get_parameter_value().string_value
        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        if not path_str:
            self.get_logger().warn(
                'waypoints_file is empty; set param or launch arg to publish course path'
            )
            self._path = None
            return
        try:
            with open(path_str, encoding='utf-8') as f:
                data = yaml.safe_load(f)
        except OSError as e:
            self.get_logger().error(f'Cannot read waypoints_file "{path_str}": {e}')
            self._path = None
            return
        except yaml.YAMLError as e:
            self.get_logger().error(f'Invalid YAML in "{path_str}": {e}')
            self._path = None
            return

        entries = data.get('waypoints') if isinstance(data, dict) else None
        if not entries:
            self.get_logger().error(f'No "waypoints" list in "{path_str}"')
            self._path = None
            return

        msg = Path()
        msg.header.frame_id = frame_id
        for wp in entries:
            if not isinstance(wp, dict):
                continue
            x = float(wp.get('x', 0.0))
            y = float(wp.get('y', 0.0))
            z = float(wp.get('z', 0.0))
            yaw = float(wp.get('yaw', 0.0))
            ps = PoseStamped()
            ps.header.frame_id = frame_id
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = z
            ps.pose.orientation = yaw_to_quat(yaw)
            msg.poses.append(ps)

        self._path = msg
        self.get_logger().info(
            f'Loaded {len(msg.poses)} waypoints from {path_str} (frame={frame_id})'
        )

    def _on_timer(self) -> None:
        if self._path is None:
            return
        self._path.header.stamp = self.get_clock().now().to_msg()
        for ps in self._path.poses:
            ps.header.stamp = self._path.header.stamp
        self._pub.publish(self._path)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPathViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
