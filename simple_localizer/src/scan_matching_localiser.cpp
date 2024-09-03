#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <cmath>

class SimpleLocalizer : public rclcpp::Node
{
public:
    SimpleLocalizer() : Node("simple_localizer")
    {
        // Subscribe to the laser scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&SimpleLocalizer::scanCallback, this, std::placeholders::_1));
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (previous_scan_.empty())
        {
            // Store the first scan
            previous_scan_ = msg->ranges;
            return;
        }

        // Process the scan and calculate the difference
        std::vector<float> current_scan = msg->ranges;
        float translation = 0.0;
        float rotation = 0.0;

        // Simple example of comparing scans
        for (size_t i = 0; i < current_scan.size(); ++i)
        {
            if (std::isfinite(current_scan[i]) && std::isfinite(previous_scan_[i]))
            {
                float diff = current_scan[i] - previous_scan_[i];
                // Implement scan matching logic here
                translation += diff; // Example logic for translation
            }
        }

        // Print or use the estimated translation and rotation
        RCLCPP_INFO(this->get_logger(), "Estimated translation: %f", translation);

        // Store current scan as the previous scan for the next callback
        previous_scan_ = current_scan;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    std::vector<float> previous_scan_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleLocalizer>());
    rclcpp::shutdown();
    return 0;
}

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <vector>
// #include <cmath>
// #include <chrono>

// class SimpleLocalizer : public rclcpp::Node
// {
// public:
//     SimpleLocalizer() : Node("simple_localizer"), current_x_(1.0), current_y_(-4.0), previous_time_(this->now())
//     {
//         // Subscribe to the laser scan topic
//         subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", 10, std::bind(&SimpleLocalizer::scanCallback, this, std::placeholders::_1));
//     }

// private:
//     void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
//     {
//         // Get the current time
//         auto current_time = this->now();

//         if (previous_scan_.empty())
//         {
//             // Store the first scan and time
//             previous_scan_ = msg->ranges;
//             previous_time_ = current_time;
//             return;
//         }

//         // Time difference between scans
//         double dt = (current_time - previous_time_).seconds();

//         // Process the scan and calculate the difference
//         std::vector<float> current_scan = msg->ranges;
//         float translation = 0.0;
//         float rotation = 0.0;

//         // Simple example of comparing scans
//         for (size_t i = 0; i < current_scan.size(); ++i)
//         {
//             if (std::isfinite(current_scan[i]) && std::isfinite(previous_scan_[i]))
//             {
//                 float diff = current_scan[i] - previous_scan_[i];
//                 translation += diff; // Example logic for translation

//                 // Example logic for rotation (very simplified)
//                 // Assume changes in scan points at different angles indicate rotation
//                 if (i > 0 && std::isfinite(current_scan[i - 1]) && std::isfinite(previous_scan_[i - 1]))
//                 {
//                     float angle_diff = std::atan2(current_scan[i] - current_scan[i - 1], 1.0) - std::atan2(previous_scan_[i] - previous_scan_[i - 1], 1.0);
//                     rotation += angle_diff; // Accumulate rotation differences
//                 }
//             }
//         }

//         // Estimate linear and angular velocities
//         float linear_velocity = translation / dt;
//         float angular_velocity = rotation / dt;

//         // Update the current pose based on the translation and rotation
//         current_x_ += translation * std::cos(rotation);
//         current_y_ += translation * std::sin(rotation);

//         // Calculate distance traveled from the start pose
//         float distance_traveled = std::sqrt(std::pow(current_x_ - start_x_, 2) + std::pow(current_y_ - start_y_, 2));

//         // Print the current pose, distance traveled, linear velocity, and angular velocity
//         RCLCPP_INFO(this->get_logger(), "Current Pose: x = %f, y = %f", current_x_, current_y_);
//         RCLCPP_INFO(this->get_logger(), "Distance Traveled: %f", distance_traveled);
//         RCLCPP_INFO(this->get_logger(), "Estimated Linear Velocity: %f m/s", linear_velocity);
//         RCLCPP_INFO(this->get_logger(), "Estimated Angular Velocity: %f rad/s", angular_velocity);

//         // Store current scan and time as the previous ones for the next callback
//         previous_scan_ = current_scan;
//         previous_time_ = current_time;
//     }

//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
//     std::vector<float> previous_scan_;
//     rclcpp::Time previous_time_;

//     // Start pose
//     const float start_x_ = 1.0;
//     const float start_y_ = -4.0;

//     // Current pose
//     float current_x_;
//     float current_y_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<SimpleLocalizer>());
//     rclcpp::shutdown();
//     return 0;
// }




// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/laser_scan.hpp>
// #include <vector>
// #include <cmath>

// class SimpleLocalizer : public rclcpp::Node
// {
// public:
//     SimpleLocalizer() : Node("simple_localizer"), current_x_(1.0), current_y_(-4.0)
//     {
//         // Subscribe to the laser scan topic
//         subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
//             "/scan", 10, std::bind(&SimpleLocalizer::scanCallback, this, std::placeholders::_1));
//     }

// private:
//     void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
//     {
//         if (previous_scan_.empty())
//         {
//             // Store the first scan
//             previous_scan_ = msg->ranges;
//             return;
//         }

//         // Process the scan and calculate the difference
//         std::vector<float> current_scan = msg->ranges;
//         float translation = 0.0;
//         float rotation = 0.0;

//         // Example: Calculate translation based on scan differences
//         for (size_t i = 0; i < current_scan.size(); ++i)
//         {
//             if (std::isfinite(current_scan[i]) && std::isfinite(previous_scan_[i]))
//             {
//                 float diff = current_scan[i] - previous_scan_[i];
//                 translation += diff; // This is a simplified example
//             }
//         }

//         // Update the current pose based on the translation (simplified assumption)
//         current_x_ += translation * std::cos(rotation);
//         current_y_ += translation * std::sin(rotation);

//         // Calculate distance traveled from the start pose
//         float distance_traveled = std::sqrt(std::pow(current_x_ - start_x_, 2) + std::pow(current_y_ - start_y_, 2));

//         // Print the current pose and distance traveled
//         RCLCPP_INFO(this->get_logger(), "Current Pose: x = %f, y = %f", current_x_, current_y_);
//         RCLCPP_INFO(this->get_logger(), "Distance Traveled: %f", distance_traveled);

//         // Store current scan as the previous scan for the next callback
//         previous_scan_ = current_scan;
//     }

//     rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
//     std::vector<float> previous_scan_;

//     // Start pose
//     const float start_x_ = 1.0;
//     const float start_y_ = -4.0;

//     // Current pose
//     float current_x_;
//     float current_y_;
// };

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<SimpleLocalizer>());
//     rclcpp::shutdown();
//     return 0;
// }