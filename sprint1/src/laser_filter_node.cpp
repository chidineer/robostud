#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserFilterNode : public rclcpp::Node
{
public:
    LaserFilterNode()
        : Node("laser_filter_node")
    {
        // Subscriber to the laser scan topic
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LaserFilterNode::laser_callback, this, std::placeholders::_1));

        // Publisher for the filtered laser scan
        filtered_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);

        // Get use nth point. Make 5 unless specified otherwise
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::shared_ptr<sensor_msgs::msg::LaserScan> filtered_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
        // auto filtered_msg = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);

        // Keep every nth point
        std::vector<float> filtered_ranges;
        for (size_t i = 0; i < msg->ranges.size(); i += nth_point_)
        {
            filtered_ranges.push_back(msg->ranges[i]);
        }

        filtered_msg->ranges = filtered_ranges;
        filtered_msg->angle_increment = msg->angle_increment * nth_point_;
        filtered_msg->scan_time = msg->scan_time;
        filtered_msg->time_increment = msg->time_increment * nth_point_;

        // Publish the filtered scan
        filtered_scan_pub_->publish(*filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_pub_;
    int nth_point_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserFilterNode>());
    rclcpp::shutdown();
    return 0;
}
