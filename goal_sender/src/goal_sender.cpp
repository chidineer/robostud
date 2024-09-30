/**
 * @file goal_sender.cpp
 * @brief Implements a ROS2 node that sends navigation goals to a robot.
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <fstream>
#include <vector>
#include <cmath>

/**
 * @class GoalSender
 * @brief A ROS2 node that reads goal poses from a file and sends them to a robot for navigation.
 */
class GoalSender : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    /**
     * @brief Construct a new Goal Sender object
     */
    GoalSender() : Node("goal_sender")
    {
        this->declare_parameter("goals_file", "goals.txt");
        
        client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
            this,
            "navigate_to_pose");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&GoalSender::timer_callback, this));

        readGoalsFromFile();
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::PoseStamped> goals_;
    size_t current_goal_index_ = 0;
    bool goal_in_progress_ = false;

    /**
     * @brief Timer callback function to check and send goals
     */
    void timer_callback()
    {
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available");
            return;
        }

        if (current_goal_index_ >= goals_.size()) {
            RCLCPP_INFO(this->get_logger(), "All goals completed");
            rclcpp::shutdown();
            return;
        }

        if (!goal_in_progress_) {
            sendGoal();
        }
    }

    /**
     * @brief Send the next goal to the action server
     */
    void sendGoal()
    {
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = goals_[current_goal_index_];

        RCLCPP_INFO(this->get_logger(), "Sending goal %zu", current_goal_index_ + 1);

        goal_in_progress_ = true;
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&GoalSender::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&GoalSender::result_callback, this, std::placeholders::_1);

        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    /**
     * @brief Callback function for goal response
     * @param goal_handle The goal handle returned by the action server
     */
    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            goal_in_progress_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    /**
     * @brief Callback function for goal result
     * @param result The result of the goal execution
     */
    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
        goal_in_progress_ = false;
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                current_goal_index_++;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
    }

    /**
     * @brief Read goal poses from a file
     */
    void readGoalsFromFile()
    {
        std::string filename = this->get_parameter("goals_file").as_string();
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", filename.c_str());
            return;
        }

        double x, y, z, w;
        while (file >> x >> y >> z >> w) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = this->now();
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.orientation.z = z;
            pose.pose.orientation.w = w;
            goals_.push_back(pose);
        }

        RCLCPP_INFO(this->get_logger(), "Read %zu goals from file", goals_.size());
    }
};

/**
 * @brief Main function to initialize and run the GoalSender node
 * @param argc Number of command-line arguments
 * @param argv Array of command-line arguments
 * @return int Exit status
 */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalSender>());
    rclcpp::shutdown();
    return 0;
}