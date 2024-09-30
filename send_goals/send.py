#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import json
from transforms3d.euler import euler2quat

class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goals = self.load_goals()
        self.current_goal_index = 0

    def load_goals(self):
        try:
            with open('goals.json', 'r') as f:
                data = json.load(f)
            self.get_logger().info(f"Loaded goals: {data['goals']}")
            return data['goals']
        except Exception as e:
            self.get_logger().error(f"Error loading goals: {str(e)}")
            return []

    def send_goal(self):
        if self.current_goal_index >= len(self.goals):
            self.get_logger().info('All goals completed!')
            rclpy.shutdown()
            return

        goal = self.goals[self.current_goal_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = goal['x']
        goal_msg.pose.pose.position.y = goal['y']
        
        theta = goal.get('theta', 0.0)
        q = euler2quat(0, 0, theta)
        goal_msg.pose.pose.orientation.x = q[1]
        goal_msg.pose.pose.orientation.y = q[2]
        goal_msg.pose.pose.orientation.z = q[3]
        goal_msg.pose.pose.orientation.w = q[0]

        self.get_logger().info(f'Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info(f'Action server is available. Sending goal {self.current_goal_index + 1}: x={goal["x"]}, y={goal["y"]}, theta={theta}')
        
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.current_goal_index += 1
            self.send_goal()
            return

        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:
            self.get_logger().info(f'Goal {self.current_goal_index + 1} succeeded!')
        else:
            self.get_logger().error(f'Goal {self.current_goal_index + 1} failed with status: {status}')
            self.get_logger().info(f'Trying to send the next goal...')
        
        self.current_goal_index += 1
        self.send_goal()

def main(args=None):
    rclpy.init(args=args)
    goal_sender = GoalSender()
    goal_sender.send_goal()
    rclpy.spin(goal_sender)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateToPose
# from geometry_msgs.msg import PoseStamped
# import json
# from transforms3d.euler import euler2quat

# class GoalSender(Node):
#     def __init__(self):
#         super().__init__('goal_sender')
#         self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
#         self.goals = self.load_goals()
#         self.current_goal_index = 0

#     def load_goals(self):
#         with open('goals.json', 'r') as f:
#             data = json.load(f)
#         return data['goals']

#     def send_goal(self):
#         if self.current_goal_index >= len(self.goals):
#             self.get_logger().info('All goals completed!')
#             rclpy.shutdown()
#             return

#         goal = self.goals[self.current_goal_index]
#         goal_msg = NavigateToPose.Goal()
#         goal_msg.pose.header.frame_id = 'map'
#         goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

#         goal_msg.pose.pose.position.x = goal['x']
#         goal_msg.pose.pose.position.y = goal['y']
        
#         q = euler2quat(0, 0, goal['theta'])
#         goal_msg.pose.pose.orientation.x = q[1]
#         goal_msg.pose.pose.orientation.y = q[2]
#         goal_msg.pose.pose.orientation.z = q[3]
#         goal_msg.pose.pose.orientation.w = q[0]

#         self.action_client.wait_for_server()
#         self.get_logger().info(f'Sending goal {self.current_goal_index + 1}: x={goal["x"]}, y={goal["y"]}, theta={goal["theta"]}')
#         future = self.action_client.send_goal_async(goal_msg)
#         future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('Goal rejected')
#             return

#         self.get_logger().info('Goal accepted')
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.goal_result_callback)

#     def goal_result_callback(self, future):
#         result = future.result().result
#         status = future.result().status
#         if status == 4:
#             self.get_logger().info(f'Goal {self.current_goal_index + 1} succeeded!')
#             self.current_goal_index += 1
#             self.send_goal()
#         else:
#             self.get_logger().error(f'Goal {self.current_goal_index + 1} failed with status: {status}')

# def main(args=None):
#     rclpy.init(args=args)
#     goal_sender = GoalSender()
#     goal_sender.send_goal()
#     rclpy.spin(goal_sender)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()