#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "../src/tsp.hpp"

TEST(tsp, TestTSPSearch)
{
    std::shared_ptr<tsp> tsp_solver = std::make_shared<tsp>();

    geometry_msgs::msg::PoseArray goals;
    geometry_msgs::msg::Pose goal_1;
    goal_1.position.set__x(0.0);
    goals.poses.push_back(goal_1);

    std::vector<unsigned int> calculated_path = tsp_solver->optimizePath(goals);

    // Ground Truth

    std::vector<unsigned int> ground_truth_path = {0, 1, 2, 3, 4, 5, 6, 7};

    for(size_t i = 0; i <= calculated_path.size(); i++)
    {
        ASSERT_TRUE(calculated_path.at(i) == ground_truth_path.at(i));
    }

}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
