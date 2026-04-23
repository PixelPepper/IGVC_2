#!/usr/bin/env python3
"""
Navigate through the IGVC course autonomously.

The authoritative course definition lives in
src/orange_ros2/orange_gazebo/config/waypoints/igvc_course_waypoints.yaml.
This script loads that file and inserts intermediate midpoints between each
named waypoint for smoother path following.
"""

from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
import yaml


WAYPOINTS_FILE = (
    Path(__file__).resolve().parent
    / "src/orange_ros2/orange_gazebo/config/waypoints/igvc_course_waypoints.yaml"
)


def load_named_waypoints(path: Path) -> list[dict]:
    """Load the named odom-frame route from the source-of-truth YAML."""
    with path.open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    entries = data.get("waypoints")
    if not isinstance(entries, list) or not entries:
        raise ValueError(f'No "waypoints" list found in {path}')

    named_waypoints = []
    for i, wp in enumerate(entries, start=1):
        if not isinstance(wp, dict):
            continue
        point = wp.get("point", {}) if isinstance(wp.get("point"), dict) else {}
        named_waypoints.append(
            {
                "name": str(wp.get("name", f"Waypoint {i}")),
                "x": float(wp.get("x", point.get("x", 0.0))),
                "y": float(wp.get("y", point.get("y", 0.0))),
                "yaw": float(wp.get("yaw", 0.0)),
            }
        )

    if not named_waypoints:
        raise ValueError(f"No usable waypoint entries found in {path}")
    return named_waypoints


def build_course_sequence(named_waypoints: list[dict]) -> list[dict]:
    """Interleave midpoint goals so long legs are split into smaller hops."""
    route = []
    previous = {"name": "Start", "x": 0.0, "y": 0.0, "yaw": 0.0}

    for wp in named_waypoints:
        if previous["name"] == "Start":
            mid_name = f"Mid to {wp['name']}"
        elif wp["name"] == "Return to Start":
            mid_name = "Mid to Return"
        else:
            mid_name = f"Mid {previous['name']}-{wp['name']}"

        route.append(
            {
                "name": mid_name,
                "x": (previous["x"] + wp["x"]) / 2.0,
                "y": (previous["y"] + wp["y"]) / 2.0,
                "yaw": wp["yaw"],
            }
        )
        route.append(wp)
        previous = wp

    return route

class IGVCCourseNavigator(Node):
    def __init__(self):
        super().__init__('igvc_course_navigator')
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server ready!')
        self.waypoints_file = WAYPOINTS_FILE
        self.named_waypoints = load_named_waypoints(self.waypoints_file)
        self.waypoints = build_course_sequence(self.named_waypoints)
        self.get_logger().info(
            f'Loaded {len(self.named_waypoints)} named waypoints from {self.waypoints_file}'
        )
        
        self.current_waypoint = 0
        self.goal_handle = None
        
    def send_goal(self, x, y, yaw, name):
        """Send a navigation goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'odom'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        from math import sin, cos
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2.0)
        
        self.get_logger().info(f'Sending waypoint {self.current_waypoint + 1}/{len(self.waypoints)}: {name}')
        self.get_logger().info(f'Target: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection"""
        self.goal_handle = future.result()
        
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted! Robot navigating...')
        get_result_future = self.goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)
        
    def feedback_callback(self, feedback_msg):
        """Handle navigation feedback"""
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        # Print progress every 2 meters
        if int(distance) % 2 == 0:
            self.get_logger().info(f'Distance remaining: {distance:.2f}m')
        
    def get_result_callback(self, future):
        """Handle navigation result"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'✓ Waypoint {self.current_waypoint + 1} reached!')
            self.current_waypoint += 1
            
            if self.current_waypoint < len(self.waypoints):
                # Send next waypoint after a short delay
                time.sleep(1)
                wp = self.waypoints[self.current_waypoint]
                self.send_goal(wp['x'], wp['y'], wp['yaw'], wp['name'])
            else:
                self.get_logger().info('🎉 Course complete! All waypoints reached!')
                rclpy.shutdown()
        else:
            self.get_logger().error(f'Navigation failed with status: {status}')
            self.get_logger().info('Attempting next waypoint...')
            self.current_waypoint += 1
            if self.current_waypoint < len(self.waypoints):
                wp = self.waypoints[self.current_waypoint]
                self.send_goal(wp['x'], wp['y'], wp['yaw'], wp['name'])
            else:
                rclpy.shutdown()
    
    def run(self):
        """Start navigating through the course"""
        self.get_logger().info(f'Starting IGVC course navigation with {len(self.waypoints)} waypoints')
        wp = self.waypoints[self.current_waypoint]
        self.send_goal(wp['x'], wp['y'], wp['yaw'], wp['name'])

def main(args=None):
    rclpy.init(args=args)
    
    navigator = IGVCCourseNavigator()
    
    try:
        navigator.run()
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        navigator.get_logger().info('Navigation cancelled by user')
    except Exception as e:
        navigator.get_logger().error(f'Error: {e}')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
