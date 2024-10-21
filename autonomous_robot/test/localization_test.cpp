#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>


TEST(LocalizerAndNavigation, TestLocalization)
{
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("autonomous_robot");
    std::string bag_filename = package_share_directory + "/ros_bags/localization";

    rosbag2_cpp::Reader reader1;
    rclcpp::Serialization<geometry_msgs::msg::PoseWithCovarianceStamped> serialization_amcl_pose;
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr amcl_pose_msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    reader1.open(bag_filename);

    while (reader1.has_next()) {
        rosbag2_storage::SerializedBagMessageSharedPtr msg = reader1.read_next();
        if (msg->topic_name != "/amcl_pose") {
            continue;
        }
        rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        serialization_amcl_pose.deserialize_message(&serialized_msg, amcl_pose_msg.get());
        break;
    }

    rosbag2_cpp::Reader reader2;
    rclcpp::Serialization<nav_msgs::msg::Odometry> serialization_odometry;
    nav_msgs::msg::Odometry::SharedPtr odometry_msg = std::make_shared<nav_msgs::msg::Odometry>();
    reader2.open(bag_filename);

    // while (reader2.has_next()) {
    //     rosbag2_storage::SerializedBagMessageSharedPtr msg = reader2.read_next();
    //     if (msg->topic_name != "/odom") {
    //         continue;
    //     }
    //     rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
    //     serialization_odometry.deserialize_message(&serialized_msg, odometry_msg.get());
    //     break;
    // }

    geometry_msgs::msg::Pose bag_amcl_pose = amcl_pose_msg->pose.pose;
    // nav_msgs::msg::Odometry bag_odometry = *odometry_msg;

    // Ground Truths
    double ground_truth_x = 0.0;
    double ground_truth_y = 0.0;
    double ground_truth_z = 0.0;

    double tolerance = 0.5;

    // ASSERT_NEAR(bag_amcl_pose.position.x, bag_odometry.pose.pose.position.x, tolerance);
    // ASSERT_NEAR(bag_amcl_pose.position.y, bag_odometry.pose.pose.position.y, tolerance);
    // ASSERT_NEAR(bag_amcl_pose.position.z, bag_odometry.pose.pose.position.z, tolerance);

    ASSERT_NEAR(bag_amcl_pose.position.x, ground_truth_x, tolerance);
    ASSERT_NEAR(bag_amcl_pose.position.y, ground_truth_y, tolerance);
    ASSERT_NEAR(bag_amcl_pose.position.z, ground_truth_z, tolerance);

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
