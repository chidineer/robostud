#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class CameraSubscriber : public rclcpp::Node
{
public:
    CameraSubscriber() : Node("camera_subscriber")
    {
        // Subscriber to the raw image topic
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, std::bind(&CameraSubscriber::image_callback, this, std::placeholders::_1));

        // Publisher for the image size topic
        image_size_publisher_ = this->create_publisher<std_msgs::msg::String>("/image_size", 10);

        // Publisher for the modified image topic
        modified_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/modified_image", 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Convert ROS image message to OpenCV image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Get image size and publish it
        std::string image_size = "I have listened to the original image and this is the image size: " +
                                 std::to_string(cv_ptr->image.cols) + "x" + std::to_string(cv_ptr->image.rows);
        auto size_msg = std::make_shared<std_msgs::msg::String>();
        size_msg->data = image_size;
        image_size_publisher_->publish(*size_msg);

        // Draw a circle at the center of the image
        cv::Point center(cv_ptr->image.cols / 2, cv_ptr->image.rows / 2);
        int radius = 50;
        cv::Scalar color(0, 255, 0); // Green color
        int thickness = 2;
        cv::circle(cv_ptr->image, center, radius, color, thickness);

        // Convert OpenCV image back to ROS Image message and publish
        modified_image_publisher_->publish(*cv_ptr->toImageMsg());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr image_size_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr modified_image_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraSubscriber>());
    rclcpp::shutdown();
    return 0;
}
