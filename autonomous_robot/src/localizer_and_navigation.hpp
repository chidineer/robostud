#ifndef LOCALIZER_AND_NAVIGATION_HPP
#define LOCALIZER_AND_NAVIGATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <mutex>
#include <vector>
#include "cylinder_detector.hpp"
#include <memory>
#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "colour_detector.hpp"
#include "tsp.hpp"

class LocalizerAndNavigation : public rclcpp::Node
{
public:
    LocalizerAndNavigation(float x, float y, float yaw);

    enum class State
    {
        IDLE,
        RUNNING,
        TASKED
    };

    enum ProductColour {
        RED,
        YELLOW,
        BLUE,
        GREEN,
        ORANGE,
        PURPLE,
        NOTHING
    };

    struct Stocktake {
        unsigned int red;
        unsigned int yellow;
        unsigned int blue;
        unsigned int green;
        unsigned int orange;
        unsigned int purple;
    };
    
    // Public methods for each state
    void runIdleState();
    void runRunningState();
    void runTaskedState();

    // Getter for current state
    State getCurrentState() const { return current_state_; }

    void cancelGoalService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    double cylinderProximity();
    void stopNavigation();
    void stocktakeReport();
    void resetStocktake();

private:
    void publishInitialPose(float x, float y, float yaw);
    void goalCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
    void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);
    bool isGoalReached(const geometry_msgs::msg::Pose &pose);
    void publishNextGoal();
    geometry_msgs::msg::Quaternion createQuaternionFromYaw(float yaw);
    void stateMachine();

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_goal_;

    std::vector<geometry_msgs::msg::Pose> goals_;
    int current_goal_index_;
    bool started_;

    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    sensor_msgs::msg::Image::SharedPtr image_;

    State current_state_;
    Stocktake count_;

    geometry_msgs::msg::Pose current_pose_;
    int total_goals_;
    std::mutex mtx_;

    std::shared_ptr<CylinderDetector> cylinderDetectorPtr_; // Shared pointer to clyinder detector class.
    std::shared_ptr<ColourDetector> colourDetectorPtr_; // Shared pointer to colour detector class.
    std::shared_ptr<tsp> tsp_solver_;
};

#endif // LOCALIZER_AND_NAVIGATION_HPP
