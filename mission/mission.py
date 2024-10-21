#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from time import sleep
import json

class RobotNavigationColorDetection(Node):
    def __init__(self):
        super().__init__('robot_navigation_color_detection')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cv_bridge = CvBridge()
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/amcl_pose',
            self.pose_callback,
            10)
        
        self.current_goal_index = 0
        self.max_retries_per_goal = 5
        self.retry_count = 0
        
        self.color_ranges = {
            'Red': ((0, 100, 0), (10, 255, 255)),
            'Blue': ((100, 150, 0), (140, 255, 255)),
            'Yellow': ((20, 100, 100), (30, 255, 255)),
            'Green': ((40, 40, 40), (70, 255, 255)),
            'Orange': ((5, 150, 150), (15, 255, 255)),
            'Purple': ((130, 50, 50), (160, 255, 255))
        }

        self.goals = self.load_goals_from_json('goals.json')
        self.color_count = {color: [] for color in self.color_ranges.keys()}
        self.stocktake_file = 'stocktake.txt'
        self.goal_in_progress = False
        self.current_pose = None

    def load_goals_from_json(self, json_file):
        try:
            with open(json_file, 'r') as f:
                return json.load(f)
        except Exception as e:
            self.get_logger().error(f"Failed to load goals from {json_file}: {str(e)}")
            return []

    def send_goal(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('All goals completed!')
            self.write_stocktake()
            rclpy.shutdown()
            return

        goal = self.goals[self.current_goal_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = goal['x']
        goal_msg.pose.pose.position.y = goal['y']
        goal_msg.pose.pose.position.z = goal['z']

        # Set orientation as quaternion
        goal_msg.pose.pose.orientation = self.create_quaternion_from_w(goal['w'])

        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info(f'Sending goal {self.current_goal_index + 1}: '
                               f'x={goal["x"]}, y={goal["y"]}, w={goal["w"]}')

        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def create_quaternion_from_w(self, w):
        # For demonstration, assuming rotation around Z-axis
        z = np.sin(np.arccos(w) / 2)
        w = np.cos(np.arccos(w) / 2)
        
        return Quaternion(x=0.0, y=0.0, z=z, w=w)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.retry_goal_or_move_on()
            return

        self.get_logger().info('Goal accepted')
        self.goal_in_progress = True
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCESS
            self.get_logger().info(f'Goal {self.current_goal_index + 1} reached.')
            self.log_goal_reached()
        else:
            self.get_logger().info(f'Goal {self.current_goal_index + 1} succeeded with status: {status}')

    def retry_goal_or_move_on(self):
        if self.retry_count < self.max_retries_per_goal:
            self.retry_count += 1
            self.get_logger().info(f'Retrying goal {self.current_goal_index + 1}, attempt {self.retry_count} of {self.max_retries_per_goal}...')
            sleep(1)
            self.send_goal()
        else:
            self.get_logger().info(f'Max retries reached for goal {self.current_goal_index + 1}. Moving to next goal.')
            self.retry_count = 0
            self.move_to_next_goal()

    def move_to_next_goal(self):
        self.retry_count = 0
        self.goal_in_progress = False
        self.current_goal_index += 1
        self.send_goal()

    def log_goal_reached(self):
        self.get_logger().info(f'Goal {self.current_goal_index + 1} reached: '
                                f'x={self.goals[self.current_goal_index]["x"]}, '
                                f'y={self.goals[self.current_goal_index]["y"]}, '
                                f'w={self.goals[self.current_goal_index]["w"]}')
        sleep(5)  # Wait before checking color
        self.check_pose_before_color()

    def check_pose_before_color(self):
        if self.current_pose is not None:
            goal = self.goals[self.current_goal_index]
            distance_to_goal = self.calculate_distance(self.current_pose, goal)
            orientation_error = self.calculate_orientation_error(self.current_pose, goal)
            self.get_logger().info(f"Distance to goal: {distance_to_goal:.2f} meters.")
            self.get_logger().info(f"Orientation error: {orientation_error:.2f} radians.")

            if distance_to_goal < 0.5 and orientation_error < 0.1:
                self.get_logger().info('Pose is valid. Checking for color...')
                self.check_color_and_move_on()
            else:
                self.get_logger().info('Pose is not valid. Adjusting orientation...')
                self.adjust_orientation(goal['w'])
        else:
            self.get_logger().warn('Current pose not available yet.')

    def adjust_orientation(self, desired_w):
        self.get_logger().info('Adjusting orientation...')
        # Send a goal to align with the desired orientation
        orientation_goal_msg = NavigateToPose.Goal()
        orientation_goal_msg.pose.header.frame_id = 'map'
        orientation_goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        orientation_goal_msg.pose.pose.position = self.current_pose.position
        orientation_goal_msg.pose.pose.orientation = self.create_quaternion_from_w(desired_w)

        future = self.action_client.send_goal_async(orientation_goal_msg)
        future.add_done_callback(self.orientation_response_callback)

    def orientation_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Orientation goal rejected')
            self.retry_goal_or_move_on()
            return

        self.get_logger().info('Orientation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.check_orientation_callback)

    def check_orientation_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == 4:  # SUCCESS
            self.get_logger().info('Orientation goal reached. Proceeding to color detection.')
            self.check_color_and_move_on()
        else:
            self.get_logger().info('Failed to reach the orientation goal.')

    def calculate_distance(self, pose1, goal):
        return np.sqrt((pose1.position.x - goal['x'])**2 + (pose1.position.y - goal['y'])**2)

    def calculate_orientation_error(self, pose1, goal):
        current_yaw = np.arctan2(2.0 * (pose1.orientation.w * pose1.orientation.z + pose1.orientation.x * pose1.orientation.y),
                                  1.0 - 2.0 * (pose1.orientation.y**2 + pose1.orientation.z**2))
        goal_yaw = goal['w']  # Use w directly if it's already in yaw form
        return abs(current_yaw - goal_yaw)

    def check_color_and_move_on(self):
        self.get_logger().info('Checking for color...')
        self.goal_in_progress = False
        self.move_to_next_goal()

    def image_callback(self, msg):
        if not self.goal_in_progress:
            return  # Ignore image callback if no goal is in progress

        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {str(e)}")
            return

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        expected_color = self.goals[self.current_goal_index]['color']
        
        if expected_color in self.color_ranges:
            lower, upper = self.color_ranges[expected_color]
            mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(cv_image, expected_color, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                
                if cv2.contourArea(largest_contour) > 0.1 * cv_image.shape[0] * cv_image.shape[1]:
                    self.get_logger().info(f'Color {expected_color} detected at goal {self.current_goal_index + 1}')
                    self.color_count[expected_color].append((self.goals[self.current_goal_index]['x'], self.goals[self.current_goal_index]['y']))
                    self.check_color_and_move_on()

        cv2.imshow("Camera View", cv_image)
        cv2.waitKey(1)  # Display the frame for a short time

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    # def write_stocktake(self):
    #     with open(self.stocktake_file, 'w') as file:
    #         for color, locations in self.color_count.items():
    #             file.write(f'{color}: {locations}\n')
    #         self.get_logger().info(f'Stocktake written to {self.stocktake_file}')
            
    def write_stocktake(self):
        with open(self.stocktake_file, 'w') as f:
            for color, locations in self.color_count.items():
                f.write(f"{color}: {len(locations)}\n")
                for x, y in locations:
                    f.write(f"  Location: ({x:.2f}, {y:.2f})\n")

def main(args=None):
    rclpy.init(args=args)
    node = RobotNavigationColorDetection()

    node.send_goal()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()



# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import PoseStamped
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from time import sleep
# import json

# class RobotNavigationColorDetection(Node):
#     def __init__(self):
#         super().__init__('robot_navigation_color_detection')
#         self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
#         self.cv_bridge = CvBridge()
#         self.image_subscriber = self.create_subscription(
#             Image,
#             '/camera/image_raw',
#             self.image_callback,
#             10)
#         self.pose_subscriber = self.create_subscription(
#             PoseStamped,
#             '/amcl_pose',
#             self.pose_callback,
#             10)
        
#         self.current_goal_index = 0
#         self.max_retries_per_goal = 5  # Set maximum retries for each goal
#         self.retry_count = 0  # Counter to track retries for the current goal
        
#         self.color_ranges = {
#             'Red': ((0, 100, 0), (10, 255, 255)),
#             'Blue': ((100, 150, 0), (140, 255, 255)),
#             'Yellow': ((20, 100, 100), (30, 255, 255)),
#             'Green': ((40, 40, 40), (70, 255, 255)),
#             'Orange': ((5, 150, 150), (15, 255, 255)),
#             'Purple': ((130, 50, 50), (160, 255, 255))
#         }

#         # Load goals from JSON file
#         self.goals = self.load_goals_from_json('goals.json')
#         self.color_count = {color: [] for color in self.color_ranges.keys()}
#         self.stocktake_file = 'stocktake.txt'
#         self.goal_in_progress = False  # To track if a goal is currently being processed
#         self.current_pose = None  # To store the current robot pose

#     def load_goals_from_json(self, json_file):
#         try:
#             with open(json_file, 'r') as f:
#                 return json.load(f)
#         except Exception as e:
#             self.get_logger().error(f"Failed to load goals from {json_file}: {str(e)}")
#             return []

#     def send_goal(self):
#         if self.current_goal_index >= len(self.goals):
#             self.get_logger().info('All goals completed!')
#             self.write_stocktake()
#             rclpy.shutdown()
#             return

#         goal = self.goals[self.current_goal_index]
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = 'map'
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

#         # Set position (x, y, z)
#         goal_msg.pose.pose.position.x = goal['x']
#         goal_msg.pose.pose.position.y = goal['y']
#         goal_msg.pose.pose.position.z = goal['z']

#         # Set orientation as quaternion (x, y, z, w) based on the given 'w'
#         w = goal['w']
#         goal_msg.pose.pose.orientation.x = 0.0
#         goal_msg.pose.pose.orientation.y = 0.0
#         goal_msg.pose.pose.orientation.z = 0.0
#         goal_msg.pose.pose.orientation.w = w

#         self.get_logger().info(f'Waiting for action server...')
#         self.action_client.wait_for_server()
#         self.get_logger().info(f'Action server is available. Sending goal {self.current_goal_index + 1}: x={goal["x"]}, y={goal["y"]}, color={goal["color"]}')
        
#         future = self.action_client.send_goal_async(goal_msg)
#         future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('Goal rejected')
#             self.retry_goal_or_move_on()
#             return

#         self.get_logger().info('Goal accepted')
#         self.goal_in_progress = True  # Mark goal as in progress
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.goal_result_callback)

#     def goal_result_callback(self, future):
#         result = future.result().result
#         status = future.result().status

#         if status == 4:  # SUCCESS
#             self.get_logger().info(f'Goal {self.current_goal_index + 1} reached. Logging to terminal and waiting 5 seconds...')
#             self.log_goal_reached()
#         else:
#             self.get_logger().info(f'Goal {self.current_goal_index + 1} succeeded with status: {status}')

#     def retry_goal_or_move_on(self):
#         if self.retry_count < self.max_retries_per_goal:
#             self.retry_count += 1
#             self.get_logger().info(f'Retrying goal {self.current_goal_index + 1}, attempt {self.retry_count} of {self.max_retries_per_goal}...')
#             sleep(1)  # Wait before retry
#             self.send_goal()  # Retry the current goal
#         else:
#             self.get_logger().info(f'Max retries reached for goal {self.current_goal_index + 1}. Moving to next goal.')
#             self.retry_count = 0  # Reset for next goal
#             self.move_to_next_goal()

#     def move_to_next_goal(self):
#         self.retry_count = 0
#         self.goal_in_progress = False  # Goal is no longer in progress
#         self.current_goal_index += 1
#         self.send_goal()

#     def log_goal_reached(self):
#         self.get_logger().info(f'Goal {self.current_goal_index + 1} reached: x={self.goals[self.current_goal_index]["x"]}, y={self.goals[self.current_goal_index]["y"]}')
#         sleep(5)  # Wait for 5 seconds before checking color
#         self.check_pose_before_color()

#     def check_pose_before_color(self):
#         # Check robot's pose before proceeding with color detection
#         if self.current_pose is not None:
#             goal = self.goals[self.current_goal_index]
#             distance_to_goal = self.calculate_distance(self.current_pose, goal)
#             self.get_logger().info(f"Distance to goal: {distance_to_goal:.2f} meters.")

#             if distance_to_goal < 0.5:  # If the robot is within 0.5 meters of the goal
#                 self.get_logger().info(f'Pose is valid. Checking for color at goal {self.current_goal_index + 1}...')
#                 self.check_color_and_move_on()
#             else:
#                 self.get_logger().info(f'Pose is not valid. Robot is too far from the goal. Re-attempting goal.')
#                 self.retry_goal_or_move_on()
#         else:
#             self.get_logger().warn('Current pose not available yet.')

#     def calculate_distance(self, pose1, goal):
#         # Calculate Euclidean distance between the robot's pose and the goal
#         return np.sqrt((pose1.position.x - goal['x'])**2 + (pose1.position.y - goal['y'])**2)

#     def check_color_and_move_on(self):
#         # Check if color detection was successful before moving to the next goal
#         self.get_logger().info('Checking for color...')
#         self.goal_in_progress = False
#         self.move_to_next_goal()

#     def image_callback(self, msg):
#         if not self.goal_in_progress:
#             return  # Ignore image callback if no goal is in progress

#         try:
#             cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
#         except Exception as e:
#             self.get_logger().error(f"Failed to convert image: {str(e)}")
#             return

#         hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
#         expected_color = self.goals[self.current_goal_index]['color']
        
#         if expected_color in self.color_ranges:
#             lower, upper = self.color_ranges[expected_color]
#             mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
#             contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
#             if contours:
#                 largest_contour = max(contours, key=cv2.contourArea)
#                 x, y, w, h = cv2.boundingRect(largest_contour)
#                 cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
#                 cv2.putText(cv_image, expected_color, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                
#                 if cv2.contourArea(largest_contour) > 0.1 * cv_image.shape[0] * cv_image.shape[1]:
#                     self.get_logger().info(f'Color {expected_color} detected at goal {self.current_goal_index + 1}')
#                     self.color_count[expected_color].append((self.goals[self.current_goal_index]['x'], self.goals[self.current_goal_index]['y']))
#                     self.check_color_and_move_on()

#         cv2.imshow("Camera View", cv_image)
#         cv2.waitKey(1)  # Display the frame for a short time

#     def pose_callback(self, msg):
#         self.current_pose = msg.pose.pose  # Store the current robot pose

#     def write_stocktake(self):
#         with open(self.stocktake_file, 'w') as file:
#             for color, locations in self.color_count.items():
#                 file.write(f'{color}: {locations}\n')
#             self.get_logger().info(f'Stocktake written to {self.stocktake_file}')

# def main(args=None):
#     rclpy.init(args=args)
#     node = RobotNavigationColorDetection()

#     node.send_goal()  # Start the navigation

#     rclpy.spin(node)  # Keep the node alive

#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import PoseStamped
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from math import pi
# import os
# import json
# from time import sleep

# class RobotNavigationColorDetection(Node):
#     def __init__(self):
#         super().__init__('robot_navigation_color_detection')
#         self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
#         self.cv_bridge = CvBridge()
#         self.image_subscriber = self.create_subscription(
#             Image,
#             '/camera/image_raw',
#             self.image_callback,
#             10)
#         self.current_goal_index = 0
#         self.max_retries_per_goal = 5  # Set maximum retries for each goal
#         self.retry_count = 0  # Counter to track retries for the current goal
#         self.color_ranges = {
#             'Red': ((0, 100, 0), (10, 255, 255)),
#             'Blue': ((100, 150, 0), (140, 255, 255)),
#             'Yellow': ((20, 100, 100), (30, 255, 255)),
#             'Green': ((40, 40, 40), (70, 255, 255)),
#             'Orange': ((5, 150, 150), (15, 255, 255)),
#             'Purple': ((130, 50, 50), (160, 255, 255))
#         }

#         # Load goals from JSON file
#         self.goals = self.load_goals_from_json('goals.json')
#         self.color_count = {color: [] for color in self.color_ranges.keys()}
#         self.stocktake_file = 'stocktake.txt'
#         self.goal_in_progress = False  # To track if a goal is currently being processed

#     def load_goals_from_json(self, json_file):
#         try:
#             with open(json_file, 'r') as f:
#                 return json.load(f)
#         except Exception as e:
#             self.get_logger().error(f"Failed to load goals from {json_file}: {str(e)}")
#             return []

#     def send_goal(self):
#         if self.current_goal_index >= len(self.goals):
#             self.get_logger().info('All goals completed!')
#             self.write_stocktake()
#             rclpy.shutdown()
#             return

#         goal = self.goals[self.current_goal_index]
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = 'map'
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

#         # Set position (x, y, z)
#         goal_msg.pose.pose.position.x = goal['x']
#         goal_msg.pose.pose.position.y = goal['y']
#         goal_msg.pose.pose.position.z = goal['z']

#         # Set orientation as quaternion (x, y, z, w) based on the given 'w'
#         w = goal['w']
#         # Assuming a unit quaternion with only 'w' set, we can calculate x, y, z = 0
#         goal_msg.pose.pose.orientation.x = 0.0
#         goal_msg.pose.pose.orientation.y = 0.0
#         goal_msg.pose.pose.orientation.z = 0.0
#         goal_msg.pose.pose.orientation.w = w

#         self.get_logger().info(f'Waiting for action server...')
#         self.action_client.wait_for_server()
#         self.get_logger().info(f'Action server is available. Sending goal {self.current_goal_index + 1}: x={goal["x"]}, y={goal["y"]}, color={goal["color"]}')
        
#         future = self.action_client.send_goal_async(goal_msg)
#         future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('Goal rejected')
#             self.retry_goal_or_move_on()
#             return

#         self.get_logger().info('Goal accepted')
#         self.goal_in_progress = True  # Mark goal as in progress
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.goal_result_callback)

#     def goal_result_callback(self, future):
#         result = future.result().result
#         status = future.result().status
#         print(status)

#         if status == 4:  # SUCCESS
#             self.get_logger().info(f'Goal {self.current_goal_index + 1} reached. Checking color...')
#             self.check_color_and_move_on()
#         # elif status == 6:  # FAILURE
#         #     self.get_logger().error(f'Goal {self.current_goal_index + 1} failed with status: {status}')
#         #     self.retry_goal_or_move_on()
#         else:  # In Progress
#             self.get_logger().info(f'Goal {self.current_goal_index + 1} succeeded with status: {status}')

#     def retry_goal_or_move_on(self):
#         if self.retry_count < self.max_retries_per_goal:
#             self.retry_count += 1
#             self.get_logger().info(f'Retrying goal {self.current_goal_index + 1}, attempt {self.retry_count} of {self.max_retries_per_goal}...')
#             sleep(1)  # Wait before retry
#             self.send_goal()  # Retry the current goal
#         else:
#             self.get_logger().info(f'Max retries reached for goal {self.current_goal_index + 1}. Moving to next goal.')
#             self.retry_count = 0  # Reset for next goal
#             self.move_to_next_goal()

#     def move_to_next_goal(self):
#         self.retry_count = 0
#         self.goal_in_progress = False  # Goal is no longer in progress
#         self.current_goal_index += 1
#         self.send_goal()

#     def check_color_and_move_on(self):
#         # Check if color detection was successful before moving to the next goal
#         self.get_logger().info('Checking for color...')
#         self.goal_in_progress = False
#         self.move_to_next_goal()

#     def image_callback(self, msg):
#         if not self.goal_in_progress:
#             return  # Ignore image callback if no goal is in progress

#         try:
#             cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
#         except Exception as e:
#             self.get_logger().error(f"Failed to convert image: {str(e)}")
#             return

#         hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
#         expected_color = self.goals[self.current_goal_index]['color']
        
#         if expected_color in self.color_ranges:
#             lower, upper = self.color_ranges[expected_color]
#             mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
#             contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
#             if contours:
#                 largest_contour = max(contours, key=cv2.contourArea)
#                 x, y, w, h = cv2.boundingRect(largest_contour)
#                 cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
#                 cv2.putText(cv_image, expected_color, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
                
#                 if cv2.contourArea(largest_contour) > 0.1 * cv_image.shape[0] * cv_image.shape[1]:
#                     self.get_logger().info(f'Color {expected_color} detected at goal {self.current_goal_index + 1}')
#                     self.color_count[expected_color].append((self.goals[self.current_goal_index]['x'], self.goals[self.current_goal_index]['y']))
#                     self.check_color_and_move_on()
            
#         cv2.imshow("Camera View", cv_image)
#         cv2.waitKey(1)

#     def write_stocktake(self):
#         with open(self.stocktake_file, 'w') as f:
#             for color, locations in self.color_count.items():
#                 f.write(f"{color}: {len(locations)}\n")
#                 for x, y in locations:
#                     f.write(f"  Location: ({x:.2f}, {y:.2f})\n")

# def main(args=None):
#     rclpy.init(args=args)
#     node = RobotNavigationColorDetection()
#     node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
#     node.send_goal()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
