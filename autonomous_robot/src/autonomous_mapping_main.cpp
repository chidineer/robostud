#include "rclcpp/rclcpp.hpp"
#include "autonomous_mapping.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutonomousMapping>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}