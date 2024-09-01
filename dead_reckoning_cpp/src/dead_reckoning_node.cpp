#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <random>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>  // Needed for matrix to Euler conversion
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

using namespace std::chrono_literals;

class DeadReckoningNode : public rclcpp::Node
{
public:
    DeadReckoningNode()
        : Node("dead_reckoning_node")
    {
        // Initialize publisher and subscriber
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&DeadReckoningNode::odom_callback, this, std::placeholders::_1));
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "teleop/cmd_vel", 10, std::bind(&DeadReckoningNode::cmd_vel_callback, this, std::placeholders::_1));

        // Set random generator for Gaussian noise
        std::random_device rd;
        rng_ = std::mt19937(rd());
        noise_dist_ = std::normal_distribution<>(0.0, 0.01);  // Mean 0, standard deviation 0.01

        // Initialize variables
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Directly publish the received cmd_vel message
        cmd_vel_pub_->publish(*msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Convert Quaternion to tf2::Quaternion
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);

        // Convert tf2::Quaternion to roll, pitch, and yaw
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        // Add noise to position and orientation
        double noisy_x = msg->pose.pose.position.x + noise_dist_(rng_);
        double noisy_y = msg->pose.pose.position.y + noise_dist_(rng_);
        double noisy_theta = yaw + noise_dist_(rng_);

        // Update estimated position
        x_ = noisy_x;
        y_ = noisy_y;
        theta_ = noisy_theta;

        // Compare with ground truth
        RCLCPP_INFO(this->get_logger(), "Estimated position: x=%.2f, y=%.2f, theta=%.2f", x_, y_, theta_);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;  // New subscriber for teleop commands
    std::mt19937 rng_;
    std::normal_distribution<> noise_dist_;

    double x_, y_, theta_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeadReckoningNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
