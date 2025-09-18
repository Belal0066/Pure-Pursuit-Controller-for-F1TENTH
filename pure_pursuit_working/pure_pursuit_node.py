#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import csv
import os
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
import tf_transformations
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        # Parameters
        self.declare_parameter('csv_path', '')
        self.declare_parameter('lookahead_distance', 1.5)
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('wheelbase', 0.3302)
        self.declare_parameter('max_steering_angle', 0.4189)
        
        # Get parameters
        csv_path = self.get_parameter('csv_path').get_parameter_value().string_value
        self.lookahead_distance = self.get_parameter('lookahead_distance').get_parameter_value().double_value
        self.speed = self.get_parameter('speed').get_parameter_value().double_value
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        
        # Load waypoints
        self.waypoints = self.load_csv_waypoints(csv_path)
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        
        # Current pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        # Publishers and subscribers
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        
        self.drive_publisher = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)
        
        # Control timer
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz
        
    def load_csv_waypoints(self, csv_path):
        """Load waypoints from CSV file, skipping comment/header lines."""
        waypoints = []
        try:
            with open(csv_path, 'r') as file:
                csv_reader = csv.reader(file)
                for row in csv_reader:
                    # Skip empty lines or lines starting with '#' (comments/headers)
                    if len(row) == 0 or row[0].strip().startswith('#'):
                        continue
                    
                    if len(row) >= 2:
                        try:
                            x = float(row[0])
                            y = float(row[1])
                            waypoints.append([x, y])
                        except ValueError:
                            continue
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {e}')
            return []

        return np.array(waypoints)

    
    def odom_callback(self, msg):
        """Update current pose from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        orientation = msg.pose.pose.orientation
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w])
    
    def get_closest_waypoint_index(self):
        """Find the index of the closest waypoint to current position"""
        if len(self.waypoints) == 0:
            return 0
        
        distances = np.sqrt((self.waypoints[:, 0] - self.current_x)**2 + 
                           (self.waypoints[:, 1] - self.current_y)**2)
        return np.argmin(distances)
    
    def get_lookahead_waypoint(self, closest_index):
        """Find waypoint at lookahead distance from current position"""
        # Start searching from closest waypoint
        current_distance = 0.0
        
        for i in range(len(self.waypoints)):
            # Use circular indexing to handle track loops
            waypoint_index = (closest_index + i) % len(self.waypoints)
            
            waypoint_distance = np.sqrt(
                (self.waypoints[waypoint_index, 0] - self.current_x)**2 + 
                (self.waypoints[waypoint_index, 1] - self.current_y)**2)
            
            if waypoint_distance >= self.lookahead_distance:
                return waypoint_index
        
        # If no waypoint found at lookahead distance, return the farthest one
        return (closest_index + len(self.waypoints) // 4) % len(self.waypoints)
    
    def pure_pursuit_control(self, target_x, target_y):
        """Calculate steering angle using pure pursuit algorithm"""
        # Transform target point to vehicle frame
        cos_yaw = np.cos(self.current_yaw)
        sin_yaw = np.sin(self.current_yaw)
        
        # Translate to vehicle origin
        dx = target_x - self.current_x
        dy = target_y - self.current_y
        
        # Rotate to vehicle frame (x: forward, y: left)
        target_x_vehicle = cos_yaw * dx + sin_yaw * dy
        target_y_vehicle = -sin_yaw * dx + cos_yaw * dy
        
        # Calculate steering angle using pure pursuit formula
        if target_x_vehicle > 0:  # Target is ahead
            lookahead_distance_actual = np.sqrt(target_x_vehicle**2 + target_y_vehicle**2)
            curvature = 2 * target_y_vehicle / (lookahead_distance_actual**2)
            steering_angle = np.arctan(curvature * self.wheelbase)
        else:
            # If target is behind, steer towards it aggressively
            steering_angle = np.arctan2(target_y_vehicle, abs(target_x_vehicle)) 
        
        # Clamp steering angle
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        return steering_angle
    
    def timer_callback(self):
        """Main control loop"""
        if len(self.waypoints) == 0:
            return
        
        # Find closest waypoint
        closest_index = self.get_closest_waypoint_index()
        
        # Find lookahead waypoint
        lookahead_index = self.get_lookahead_waypoint(closest_index)
        target_x = self.waypoints[lookahead_index, 0]
        target_y = self.waypoints[lookahead_index, 1]
        
        # Calculate steering angle
        steering_angle = self.pure_pursuit_control(target_x, target_y)
        
        # Create and publish drive command
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.speed = float(self.speed)
        drive_msg.drive.steering_angle = float(steering_angle)
        
        self.drive_publisher.publish(drive_msg)
        
        # Debug info (reduced frequency)
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 25 == 0:  # Every 0.5 seconds at 50Hz
            self.get_logger().info(
                f'Pos: ({self.current_x:.2f}, {self.current_y:.2f}), '
                f'Target: ({target_x:.2f}, {target_y:.2f}), '
                f'Steering: {steering_angle:.3f}, Speed: {self.speed:.2f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

