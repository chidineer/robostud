#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserScanSubscriber : public rclcpp::Node
{
public:
    LaserScanSubscriber()
    : Node("laser_scan_subscriber")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanSubscriber::topic_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/selected_scan", 10);

        angle_to_check_ = 90.0;  // Angle in degrees
        start_index_ = 0;        // Starting index for subset
        end_index_ = 10;         // Ending index for subset
    }

private:
    void topic_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
        // Calculate index for the specified angle
        double angle_increment = msg->angle_increment;
        double angle_min = msg->angle_min;
        int index_to_check = static_cast<int>((angle_to_check_ * M_PI / 180.0 - angle_min) / angle_increment);

        if (index_to_check >= 0 && index_to_check < static_cast<int>(msg->ranges.size()))
        {
            double range_at_angle = msg->ranges[index_to_check];
            RCLCPP_INFO(this->get_logger(), "Range at %f degrees: %f meters", angle_to_check_, range_at_angle);
        }

        // Select a subset of the ranges
        std::vector<float> selected_ranges(msg->ranges.begin() + start_index_, msg->ranges.begin() + end_index_);
        auto selected_scan = sensor_msgs::msg::LaserScan();

        selected_scan.header = msg->header;
        selected_scan.angle_min = msg->angle_min + start_index_ * angle_increment;
        selected_scan.angle_max = msg->angle_min + end_index_ * angle_increment;
        selected_scan.angle_increment = msg->angle_increment;
        selected_scan.time_increment = msg->time_increment;
        selected_scan.scan_time = msg->scan_time;
        selected_scan.range_min = msg->range_min;
        selected_scan.range_max = msg->range_max;
        selected_scan.ranges = selected_ranges;

        // Publish the selected scan data
        publisher_->publish(selected_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;

    double angle_to_check_;
    size_t start_index_;
    size_t end_index_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanSubscriber>());
    rclcpp::shutdown();
    return 0;
}
