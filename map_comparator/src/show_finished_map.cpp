#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class ShowFinishedMapNode : public rclcpp::Node
{
public:
    ShowFinishedMapNode() : Node("show_finished_map_node")
    {
        // Load ground truth map
        ground_truth_map_ = cv::imread("/home/student/RS1/RS1_individual/ros2_ws_individual/src/warehouse_map/ground_truth.pgm", cv::IMREAD_GRAYSCALE);
        if (ground_truth_map_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load ground truth map.");
            rclcpp::shutdown();
        }

        // Load final map
        final_map_ = cv::imread("/home/student/RS1/RS1_individual/ros2_ws_individual/src/warehouse_map/real_warehouse.pgm", cv::IMREAD_GRAYSCALE);
        if (final_map_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load final map.");
            rclcpp::shutdown();
        }

        // Ensure both maps have the same size
        if (ground_truth_map_.size() != final_map_.size()) {
            cv::resize(final_map_, final_map_, ground_truth_map_.size());
        }

        // Blend the two maps (50% transparency for each map)
        cv::Mat blended_map;
        cv::addWeighted(ground_truth_map_, 0.5, final_map_, 0.5, 0.0, blended_map);

        // Create windows for displaying images
        cv::namedWindow("Ground Truth Map", cv::WINDOW_NORMAL);
        cv::namedWindow("Final Map", cv::WINDOW_NORMAL);
        cv::namedWindow("Blended Map", cv::WINDOW_NORMAL);

        // Show both maps
        cv::imshow("Ground Truth Map", ground_truth_map_);
        cv::imshow("Final Map", final_map_);
        cv::imshow("Blended Map", blended_map); // Show blended map

        // Call waitKey to keep the window open until a key is pressed
        cv::waitKey(0); // Wait indefinitely for a keypress
    }

private:
    cv::Mat ground_truth_map_;
    cv::Mat final_map_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ShowFinishedMapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
