#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ColorDetectorNode : public rclcpp::Node
{
public:
    ColorDetectorNode() : Node("color_detector_node")
    {
        // Subscribe to the image_raw topic
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&ColorDetectorNode::process_image, this, std::placeholders::_1));
        
        cv::namedWindow("Color Detection", cv::WINDOW_AUTOSIZE);
    }

private:
    void process_image(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert the ROS image message to an OpenCV image
        cv_bridge::CvImagePtr cv_image_ptr;
        try
        {
            cv_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Convert the image to HSV color space
        cv::Mat hsv_image;
        cv::cvtColor(cv_image_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        // Define color ranges and create masks
        // cv::Scalar red_lower(0, 120, 70), red_upper(10, 255, 255);
        cv::Scalar red_lower(0, 100, 0), red_upper(10, 255, 10);
        cv::Scalar blue_lower(100, 150, 0), blue_upper(140, 255, 255);
        cv::Scalar yellow_lower(20, 100, 100), yellow_upper(30, 255, 255);
        cv::Scalar green_lower(40, 40, 40), green_upper(70, 255, 255);
        cv::Scalar orange_lower(5, 150, 150), orange_upper(15, 255, 255);
        // cv::Scalar purple_lower(130, 50, 50), purple_upper(160, 255, 255);
        cv::Scalar purple_lower(130, 50, 50), purple_upper(160, 255, 255);

        cv::Mat red_mask, blue_mask, yellow_mask, green_mask, orange_mask, purple_mask;
        cv::inRange(hsv_image, red_lower, red_upper, red_mask);
        cv::inRange(hsv_image, blue_lower, blue_upper, blue_mask);
        cv::inRange(hsv_image, yellow_lower, yellow_upper, yellow_mask);
        cv::inRange(hsv_image, green_lower, green_upper, green_mask);
        cv::inRange(hsv_image, orange_lower, orange_upper, orange_mask);
        cv::inRange(hsv_image, purple_lower, purple_upper, purple_mask);

        cv::Mat combined_mask = red_mask | blue_mask | yellow_mask | green_mask | orange_mask | purple_mask;

        // Get the central region of the image
        int center_x = cv_image_ptr->image.cols / 2;
        int center_y = cv_image_ptr->image.rows / 2;
        int region_width = cv_image_ptr->image.cols / 4;  // Width of the central area (25% of the image)
        int region_height = cv_image_ptr->image.rows / 4;  // Height of the central area (25% of the image)

        cv::Rect central_region(center_x - region_width / 2, center_y - region_height / 2, region_width, region_height);

        // Find contours and draw bounding boxes
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(combined_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool color_detected = false;

        // Get the total area of the image
        int total_area = cv_image_ptr->image.cols * cv_image_ptr->image.rows;
        int area_threshold = static_cast<int>(0.05 * total_area); // 5% of the total area

        // Iterate over contours and check areas
        for (size_t i = 0; i < contours.size(); i++) {
            double contour_area = cv::contourArea(contours[i]);

            if (contour_area > area_threshold) {
                cv::Rect bounding_box = cv::boundingRect(contours[i]);

                // Check if the bounding box intersects with the central region
                if ((bounding_box & central_region).area() > 0) {
                    cv::rectangle(cv_image_ptr->image, bounding_box, cv::Scalar(0, 255, 0), 2);

                    // Check which color was detected by checking which mask matches
                    if (cv::countNonZero(red_mask(bounding_box)) > 0) {
                        std::cout << "Detected Red: RGBA (1.0, 0.0, 0.0, 1.0)" << std::endl;
                        color_detected = true;
                    } else if (cv::countNonZero(blue_mask(bounding_box)) > 0) {
                        std::cout << "Detected Blue: RGBA (0.0, 0.0, 1.0, 1.0)" << std::endl;
                        color_detected = true;
                    } else if (cv::countNonZero(yellow_mask(bounding_box)) > 0) {
                        std::cout << "Detected Yellow: RGBA (1.0, 1.0, 0.0, 1.0)" << std::endl;
                        color_detected = true;
                    } else if (cv::countNonZero(green_mask(bounding_box)) > 0) {
                        std::cout << "Detected Green: RGBA (0.0, 1.0, 0.0, 1.0)" << std::endl;
                        color_detected = true;
                    } else if (cv::countNonZero(orange_mask(bounding_box)) > 0) {
                        std::cout << "Detected Orange: RGBA (1.0, 0.5, 0.0, 1.0)" << std::endl;
                        color_detected = true;
                    } else if (cv::countNonZero(purple_mask(bounding_box)) > 0) {
                        std::cout << "Detected Purple: RGBA (0.5, 0.0, 0.5, 1.0)" << std::endl;
                        color_detected = true;
                    }
                }
            }
        }

        if (!color_detected) {
            std::cout << "Detecting..." << std::endl;
        }

        // Show the image
        cv::imshow("Color Detection", cv_image_ptr->image);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
