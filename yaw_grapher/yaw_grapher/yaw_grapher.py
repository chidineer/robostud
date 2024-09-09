import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import csv
import time
import os
import math

class YawListener(Node):

    def __init__(self):
        """
        Initializes the YawListener node, sets up the subscribers, and prepares to record data.
        """
        super().__init__('yaw_listener')

        # Create output directory if it doesn't exist
        self.output_dir = '/home/chidalu/uni/robostud/yaw_grapher/yaw_grapher/output'
        os.makedirs(self.output_dir, exist_ok=True)

        # Initialize subscribers
        self.sml_subscription = self.create_subscription(
            Float64,
            '/localised_yaw',
            self.localised_yaw_callback,
            10)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        self.amcl_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10)

        # Variables to store data
        self.yaw_data = []
        self.start_time = time.time()

        # File to save CSV data
        self.csv_file = open(os.path.join(self.output_dir, 'yaw_data.csv'), 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Time (s)', 'Scan Matcher (degrees)', 'Monte Carlo (degrees)', 'Ground Truth (degrees)'])

        # Set a timer to stop after 60 seconds
        self.timer = self.create_timer(1.0, self.check_time)

        # Variables to store the yaw values from /odom and /amcl_pose
        self.odom_yaw = None
        self.amcl_yaw = None
        self.localised_yaw = None

        # Variables for RMSE
        self.rmse_sml = []
        self.rmse_amcl = []

    def localised_yaw_callback(self, msg):
        self.localised_yaw = msg.data
        self.record_yaw()

    def odom_callback(self, msg):
        self.odom_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.record_yaw()

    def amcl_pose_callback(self, msg):
        self.amcl_yaw = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.record_yaw()

    def quaternion_to_yaw(self, orientation):
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw)

    def record_yaw(self):
        elapsed_time = time.time() - self.start_time
        if self.localised_yaw is not None and self.odom_yaw is not None and self.amcl_yaw is not None:
            self.yaw_data.append((elapsed_time, self.localised_yaw, self.amcl_yaw, self.odom_yaw))
            self.csv_writer.writerow([elapsed_time, self.localised_yaw, self.amcl_yaw, self.odom_yaw])

            # Calculate RMSE for each data point
            self.rmse_sml.append((self.localised_yaw - self.odom_yaw) ** 2)
            self.rmse_amcl.append((self.amcl_yaw - self.odom_yaw) ** 2)

    def check_time(self):
        elapsed_time = time.time() - self.start_time
        if elapsed_time >= 60:
            self.get_logger().info('Finished recording yaw data for 60 seconds.')

            # Close CSV file and stop recording
            self.csv_file.close()

            # Compute RMSE
            rmse_sml = math.sqrt(sum(self.rmse_sml) / len(self.rmse_sml))
            rmse_amcl = math.sqrt(sum(self.rmse_amcl) / len(self.rmse_amcl))

            # Write RMSE to text file
            with open(os.path.join(self.output_dir, 'rmse_report.md'), 'w') as rmse_file:
                rmse_file.write(f"## RMSE Report\n\n")
                rmse_file.write(f"- **Scan Matcher vs Ground Truth RMSE**: {rmse_sml:.4f} degrees\n")
                rmse_file.write(f"- **Monte Carlo vs Ground Truth RMSE**: {rmse_amcl:.4f} degrees\n")

            # Plot the yaw data and save the plot
            self.plot_yaw_data()

            # Shutdown the node after saving data and plotting
            rclpy.shutdown()

    def plot_yaw_data(self):
        if len(self.yaw_data) > 0:
            times, localised_yaws, amcl_yaws, odom_yaws = zip(*self.yaw_data)

            # Plot the yaw angles from different topics
            plt.figure()

            # Plot Scan Matcher every 5 seconds
            sml_times = [t for i, t in enumerate(times) if i % 15 == 0]
            localised_yaw_filtered = [localised_yaws[i] for i in range(len(times)) if i % 15 == 0]
            plt.plot(sml_times, localised_yaw_filtered, label='Scan Matcher (/localised_yaw)', color='blue')

            # Plot Monte Carlo and Ground Truth normally
            plt.plot(times, amcl_yaws, label='Monte Carlo (/amcl_pose)', color='green')
            plt.plot(times, odom_yaws, label='Ground Truth (/odom)', color='red')

            plt.xlabel('Time (s)')
            plt.ylabel('Yaw Angle (degrees)')
            plt.title('Yaw Angle vs Time')
            plt.ylim([-180, 180])  # Set y-axis limits to -180 to 180 degrees
            plt.grid(True)
            plt.legend()

            # Save the plot as an image file
            plt.savefig(os.path.join(self.output_dir, 'yaw_plot.png'))
            plt.show()

def main(args=None):
    print('heyo3')
    rclpy.init(args=args)
    yaw_listener = YawListener()

    # Spin the node to keep receiving messages
    rclpy.spin(yaw_listener)

if __name__ == '__main__':
    print('heyo3')
    main()
