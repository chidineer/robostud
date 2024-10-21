#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "../src/colour_detector.hpp"

using namespace std;

enum ProductColour {
    RED,
    YELLOW,
    BLUE,
    GREEN,
    ORANGE,
    PURPLE,
    NOTHING
};


TEST(ColourDetector, TestBlueBoxDetection)
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autonomous_robot");
    std::string bag_filename = package_share_directory + "/ros_bags/blue_box";

    rosbag2_cpp::Reader reader1;
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_image;
    sensor_msgs::msg::Image::SharedPtr image_msg = std::make_shared<sensor_msgs::msg::Image>();
    reader1.open(bag_filename);

    while (reader1.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader1.read_next();
        if (msg->topic_name != "/camera/image_raw") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization_image.deserialize_message(&serialized_msg, image_msg.get());
        break;
    }

    sensor_msgs::msg::Image::SharedPtr bag_image = image_msg;

    std::shared_ptr<ColourDetector> colourDetectorPtr = std::make_shared<ColourDetector>();

    int box = colourDetectorPtr->detectBox(bag_image);

    bool detected_correct_box;

    if(box == ProductColour::BLUE)
    {
        detected_correct_box = true;
        cout << "ColourDetector detected the correct colour (Blue). Success!" << endl;
    }
    else
    {
        detected_correct_box = false;
        cout << "ColourDetector was unsuccessful (Blue)." << endl;
    }

    ASSERT_TRUE(detected_correct_box);

}

TEST(ColourDetector, TestGreenBoxDetection)
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autonomous_robot");
    std::string bag_filename = package_share_directory + "/ros_bags/green_box";

    rosbag2_cpp::Reader reader1;
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_image;
    sensor_msgs::msg::Image::SharedPtr image_msg = std::make_shared<sensor_msgs::msg::Image>();
    reader1.open(bag_filename);

    while (reader1.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader1.read_next();
        if (msg->topic_name != "/camera/image_raw") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization_image.deserialize_message(&serialized_msg, image_msg.get());
        break;
    }

    sensor_msgs::msg::Image::SharedPtr bag_image = image_msg;

    std::shared_ptr<ColourDetector> colourDetectorPtr = std::make_shared<ColourDetector>();

    int box = colourDetectorPtr->detectBox(bag_image);

    bool detected_correct_box;

    if(box == ProductColour::GREEN)
    {
        detected_correct_box = true;
        cout << "ColourDetector detected the correct colour (Green). Success!" << endl;
    }
    else
    {
        detected_correct_box = false;
        cout << "ColourDetector was unsuccessful (Green)." << endl;
    }

    ASSERT_TRUE(detected_correct_box);

}

TEST(ColourDetector, TestOrangeBoxDetection)
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autonomous_robot");
    std::string bag_filename = package_share_directory + "/ros_bags/orange_box";

    rosbag2_cpp::Reader reader1;
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_image;
    sensor_msgs::msg::Image::SharedPtr image_msg = std::make_shared<sensor_msgs::msg::Image>();
    reader1.open(bag_filename);

    while (reader1.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader1.read_next();
        if (msg->topic_name != "/camera/image_raw") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization_image.deserialize_message(&serialized_msg, image_msg.get());
        break;
    }

    sensor_msgs::msg::Image::SharedPtr bag_image = image_msg;

    std::shared_ptr<ColourDetector> colourDetectorPtr = std::make_shared<ColourDetector>();

    int box = colourDetectorPtr->detectBox(bag_image);

    bool detected_correct_box;

    if(box == ProductColour::ORANGE)
    {
        detected_correct_box = true;
        cout << "ColourDetector detected the correct colour (Orange). Success!" << endl;
    }
    else if(box == ProductColour::YELLOW){
        detected_correct_box = false;
        cout << "Unfortunately, we have detected a yellow box rather than an orange box." << endl;
    }
    else
    {
        detected_correct_box = false;
        cout << "ColourDetector was unsuccessful (Orange)." << endl;
    }

    ASSERT_TRUE(detected_correct_box);

}

TEST(ColourDetector, TestPurpleBoxDetection)
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autonomous_robot");
    std::string bag_filename = package_share_directory + "/ros_bags/purple_box";

    rosbag2_cpp::Reader reader1;
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_image;
    sensor_msgs::msg::Image::SharedPtr image_msg = std::make_shared<sensor_msgs::msg::Image>();
    reader1.open(bag_filename);

    while (reader1.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader1.read_next();
        if (msg->topic_name != "/camera/image_raw") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization_image.deserialize_message(&serialized_msg, image_msg.get());
        break;
    }

    sensor_msgs::msg::Image::SharedPtr bag_image = image_msg;

    std::shared_ptr<ColourDetector> colourDetectorPtr = std::make_shared<ColourDetector>();

    int box = colourDetectorPtr->detectBox(bag_image);

    bool detected_correct_box;

    if(box == ProductColour::PURPLE)
    {
        detected_correct_box = true;
        cout << "ColourDetector detected the correct colour (Purple). Success!" << endl;
    }
    else
    {
        detected_correct_box = false;
        cout << "ColourDetector was unsuccessful (Purple)." << endl;
    }

    ASSERT_TRUE(detected_correct_box);

}

TEST(ColourDetector, TestRedBoxDetection)
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autonomous_robot");
    std::string bag_filename = package_share_directory + "/ros_bags/red_box";

    rosbag2_cpp::Reader reader1;
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_image;
    sensor_msgs::msg::Image::SharedPtr image_msg = std::make_shared<sensor_msgs::msg::Image>();
    reader1.open(bag_filename);

    while (reader1.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader1.read_next();
        if (msg->topic_name != "/camera/image_raw") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization_image.deserialize_message(&serialized_msg, image_msg.get());
        break;
    }

    sensor_msgs::msg::Image::SharedPtr bag_image = image_msg;

    std::shared_ptr<ColourDetector> colourDetectorPtr = std::make_shared<ColourDetector>();

    int box = colourDetectorPtr->detectBox(bag_image);

    bool detected_correct_box;

    if(box == ProductColour::RED)
    {
        detected_correct_box = true;
        cout << "ColourDetector detected the correct colour (Red). Success!" << endl;
    }
    else
    {
        detected_correct_box = false;
        cout << "ColourDetector was unsuccessful (Red)." << endl;
    }

    ASSERT_TRUE(detected_correct_box);

}

TEST(ColourDetector, TestYelllowBoxDetection)
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autonomous_robot");
    std::string bag_filename = package_share_directory + "/ros_bags/yellow_box";

    rosbag2_cpp::Reader reader1;
    rclcpp::Serialization<sensor_msgs::msg::Image> serialization_image;
    sensor_msgs::msg::Image::SharedPtr image_msg = std::make_shared<sensor_msgs::msg::Image>();
    reader1.open(bag_filename);

    while (reader1.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader1.read_next();
        if (msg->topic_name != "/camera/image_raw") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization_image.deserialize_message(&serialized_msg, image_msg.get());
        break;
    }

    sensor_msgs::msg::Image::SharedPtr bag_image = image_msg;

    std::shared_ptr<ColourDetector> colourDetectorPtr = std::make_shared<ColourDetector>();

    int box = colourDetectorPtr->detectBox(bag_image);

    bool detected_correct_box;

    if(box == ProductColour::YELLOW)
    {
        detected_correct_box = true;
        cout << "ColourDetector detected the correct colour (Yellow). Success!" << endl;
    }
    else
    {
        detected_correct_box = false;
        cout << "ColourDetector was unsuccessful (Yellow)." << endl;
    }

    ASSERT_TRUE(detected_correct_box);

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
