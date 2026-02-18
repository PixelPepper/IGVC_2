#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import math
import yaml

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')
        
        # Load waypoints
        with open('/home/tech/IGVC_SIM/waypoints.yaml', 'r') as f:
            data = yaml.safe_load(f)
            self.waypoints = data['waypoints']
        
        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # State
        self.current_waypoint = 0
        self.current_x = 0.0
        self.current_y = 0.0
        self.rate = self.create_rate(10)  # 10 Hz
        
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
    
    def control_loop(self):
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!')
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            return
        
        wp = self.waypoints[self.current_waypoint]
        dx = wp['x'] - self.current_x
        dy = wp['y'] - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.2:  # Reached waypoint
            self.get_logger().info(f'Reached waypoint {self.current_waypoint + 1}/{len(self.waypoints)}')
            self.current_waypoint += 1
            return
        
        # Simple proportional control
        angle_to_goal = math.atan2(dy, dx)
        
        twist = Twist()
        twist.linear.x = min(0.5, distance * 0.5)  # Max 0.5 m/s
        twist.angular.z = angle_to_goal * 1.0
        
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
