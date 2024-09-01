#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import tf_transformations
from math import cos, sin

class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__('dead_reckoning')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.timer = self.create_timer(0.1, self.move_robot)
        
        # Initialize variables for dead reckoning
        self.current_position = np.array([0.0, 0.0])
        self.current_orientation = 0.0
        self.velocity = Twist()
        
        # Noise parameters
        self.noise_std_dev = 0.01  # Standard deviation of the Gaussian noise
        
        # Movement parameters
        self.linear_speed = 0.1
        self.angular_speed = 0.0
        self.distance = 1.0
        self.direction = 1  # 1 for forward, -1 for backward
        self.travelled_distance = 0.0
        
    def move_robot(self):
        if self.travelled_distance < self.distance:
            # Set speed
            self.velocity.linear.x = self.direction * self.linear_speed
            self.velocity.angular.z = self.angular_speed
            self.publisher_.publish(self.velocity)
        else:
            self.velocity.linear.x = 0.0
            self.velocity.angular.z = 0.0
            self.publisher_.publish(self.velocity)
        
    def odom_callback(self, msg):
        # Add Gaussian noise to odometry data
        noisy_odom = msg
        noisy_odom.pose.pose.position.x += np.random.normal(0, self.noise_std_dev)
        noisy_odom.pose.pose.position.y += np.random.normal(0, self.noise_std_dev)
        
        # Update position and orientation based on noisy odometry data
        orientation_quat = noisy_odom.pose.pose.orientation
        orientation_euler = tf_transformations.euler_from_quaternion(
            [orientation_quat.x, orientation_quat.y, orientation_quat.z, orientation_quat.w])
        self.current_orientation = orientation_euler[2]
        
        self.current_position += np.array([
            noisy_odom.twist.twist.linear.x * cos(self.current_orientation),
            noisy_odom.twist.twist.linear.x * sin(self.current_orientation)
        ]) * 0.1  # Assuming the update rate is 10 Hz (0.1s)
        
        self.travelled_distance += abs(noisy_odom.twist.twist.linear.x * 0.1)
        
        # Here you can compare self.current_position with the true odometry data if you subscribe to a ground truth topic

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
