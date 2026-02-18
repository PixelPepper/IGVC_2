#!/usr/bin/env python3
"""
Navigate through the IGVC course autonomously
Sends waypoints sequentially through the course
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import time

class IGVCCourseNavigator(Node):
    def __init__(self):
        super().__init__('igvc_course_navigator')
        
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Nav2 action server ready!')
        
        # IGVC course waypoints with INTERMEDIATE points for better path following
        self.waypoints = [
            # Start → WP6 (straight east)
            {'x': 9.5, 'y': 0.7, 'yaw': 0.0, 'name': 'Mid to WP6'},
            {'x': 18.902129, 'y': 0.687874, 'yaw': 0.0, 'name': 'Waypoint 6'},
            
            # WP6 → WP5 (turn south)
            {'x': 18.8, 'y': -14.5, 'yaw': 0.0, 'name': 'Mid WP6-WP5'},
            {'x': 18.668346, 'y': -29.292145, 'yaw': 0.0, 'name': 'Waypoint 5'},
            
            # WP5 → WP4 (turn west along bottom)
            {'x': 12.5, 'y': -30.0, 'yaw': 0.0, 'name': 'Mid WP5-WP4'},
            {'x': 6.476277, 'y': -30.752035, 'yaw': 0.0, 'name': 'Waypoint 4'},
            
            # WP4 → WP3 (continue west)
            {'x': 0.0, 'y': -30.5, 'yaw': 0.0, 'name': 'Mid WP4-WP3'},
            {'x': -6.026554, 'y': -30.380053, 'yaw': 0.0, 'name': 'Waypoint 3'},
            
            # WP3 → WP2 (continue west)
            {'x': -12.0, 'y': -30.4, 'yaw': 0.0, 'name': 'Mid WP3-WP2'},
            {'x': -17.978396, 'y': -30.465322, 'yaw': 0.0, 'name': 'Waypoint 2'},
            
            # WP2 → WP1 (turn north)
            {'x': -19.0, 'y': -15.0, 'yaw': 0.0, 'name': 'Mid WP2-WP1'},
            {'x': -19.106506, 'y': -0.705461, 'yaw': 0.0, 'name': 'Waypoint 1'},
            
            # WP1 → Return (back to start)
            {'x': -9.5, 'y': 0.0, 'yaw': 0.0, 'name': 'Mid to Return'},
            {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'name': 'Return to Start'},
        ]
        
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
