#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class ShowCurrentMapNode : public rclcpp::Node
{
public:
    ShowCurrentMapNode() : Node("show_current_map_node")
    {
        // Load ground truth map
        ground_truth_map_ = cv::imread("/home/student/ros2_ws/src/autonomous_robot/map/ground_truth.pgm", cv::IMREAD_GRAYSCALE);
        if (ground_truth_map_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load ground truth map.");
            rclcpp::shutdown();
        }

        // Subscribe to the current map topic
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&ShowCurrentMapNode::mapCallback, this, std::placeholders::_1));
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        // Convert current map to OpenCV Mat
        cv::Mat current_map = convertOccupancyGridToMat(msg);

        cv::imshow("Current Map", current_map);
        cv::imshow("Grount Truth Map", ground_truth_map_);
        cv::waitKey(1); // Allow OpenCV window to refresh
    }

    cv::Mat convertOccupancyGridToMat(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        int width = msg->info.width;
        int height = msg->info.height;
        cv::Mat map_image(height, width, CV_8UC1);

        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = y * width + x;
                int occupancy = msg->data[index];
                map_image.at<uchar>(y, x) = (occupancy == -1) ? 128 : occupancy * 255 / 100;
            }
        }

        return map_image;
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    cv::Mat ground_truth_map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShowCurrentMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
