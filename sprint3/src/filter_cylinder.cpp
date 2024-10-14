/**
 * @file filter_cylinder_node.cpp
 * @brief Node for detecting and visualizing cylinders from laser scan data
 * 
 * This node processes laser scan data to detect cylinders of a specified diameter,
 * visualizes them on a map, and optionally places markers in Gazebo.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <opencv2/opencv.hpp> // For OpenCV
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <string>

/**
 * @brief Node class for filtering and detecting cylinders from laser scan data
 * 
 * This class processes laser scan data to detect cylinders of a specified diameter,
 * visualizes them on a loaded map image, and can optionally place markers in Gazebo.
 */
class FilterCylinderNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for FilterCylinderNode
     * 
     * Initializes the node, sets up subscribers and publishers, and loads the map image.
     */
    FilterCylinderNode() : Node("filter_cylinder")
    {
        // Declare the parameter for placing markers
        this->declare_parameter<bool>("place_markers", false);
        this->get_parameter("place_markers", place_markers_);

        // Subscribe to the /scan and /odom topics
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&FilterCylinderNode::laserscanCallback, this, std::placeholders::_1));

        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&FilterCylinderNode::odomCallback, this, std::placeholders::_1));

        // Publisher for markers in Gazebo
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);

        // Define a threshold for clustering points (in meters)
        clustering_threshold_ = 0.1; // 10 cm threshold for clustering

        // Cylinder diameter to detect (30 cm)
        cylinder_diameter_ = 0.30;

        // Load the map image
        map_image_ = cv::imread("/home/chidalu/uni/robostud/warehouse_navigation/map/real_warehouse.pgm", cv::IMREAD_GRAYSCALE);

        if (map_image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load the map image. Check if the file exists and is accessible.");
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Map image loaded successfully. Image size: %dx%d", map_image_.cols, map_image_.rows);
        }

        // Assume these map parameters (Adjust accordingly based on your map setup)
        map_resolution_ = 0.05;  // Map resolution in meters/pixel (adjust as necessary)
        map_origin_x_ = 9.5;   // X origin of the map in meters (adjust as necessary)
        map_origin_y_ = 14.5;   // Y origin of the map in meters (adjust as necessary)
    }

private:
    /// @brief Subscriber for laser scan messages
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    /// @brief Subscriber for odometry messages
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    /// @brief Publisher for visualization markers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    /// @brief Threshold for clustering points (in meters)
    double clustering_threshold_;
    /// @brief Diameter of the cylinder to detect (in meters)
    double cylinder_diameter_;
    /// @brief Flag to enable/disable marker placement
    bool place_markers_;

    /// @brief Original map image
    cv::Mat map_image_;
    /// @brief Edited map image for visualization
    cv::Mat edited_map_image_;
    /// @brief Map resolution in meters per pixel
    double map_resolution_;
    /// @brief X origin of the map in meters
    double map_origin_x_;
    /// @brief Y origin of the map in meters
    double map_origin_y_;

    /**
     * @brief Structure to represent a 2D point with an index
     */
    struct Point2D
    {
        double x;      ///< X coordinate
        double y;      ///< Y coordinate
        int index;     ///< Index in the original range data
    };

    /**
     * @brief Structure to represent the robot's 2D pose
     */
    struct Pose2D
    {
        double x;      ///< X coordinate
        double y;      ///< Y coordinate
        double theta;  ///< Orientation angle
    } robot_pose_;

    /**
     * @brief Callback function for processing odometry messages
     * @param msg Shared pointer to the odometry message
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        robot_pose_.x = msg->pose.pose.position.x;
        robot_pose_.y = msg->pose.pose.position.y;

        // Convert geometry_msgs::msg::Quaternion to tf2::Quaternion
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);

        // Extract orientation as yaw angle
        tf2::Matrix3x3 mat(quat);
        double roll, pitch;
        mat.getRPY(roll, pitch, robot_pose_.theta); // Extract yaw (theta)
    }

    /**
     * @brief Converts a range measurement to a 2D point
     * @param range The measured range
     * @param angle The angle of the measurement
     * @param index The index of the range in the original data
     * @return Point2D structure containing the converted point
     */
    Point2D rangeToPoint(double range, double angle, int index)
    {
        Point2D point;
        point.x = range * std::cos(angle);
        point.y = range * std::sin(angle);
        point.index = index;
        return point;
    }

    /**
     * @brief Callback function for processing laser scan messages
     * @param scan_msg Shared pointer to the laser scan message
     */
    void laserscanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
    {
        if (!scan_msg || scan_msg->ranges.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Invalid or empty laser scan message.");
            return;
        }

        std::vector<Point2D> points;
        std::vector<int> cylinder_indices;

        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            double range = scan_msg->ranges[i];
            if (range < scan_msg->range_min || range > scan_msg->range_max)
                continue;

            double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            points.push_back(rangeToPoint(range, angle, i));
        }

        std::vector<std::vector<Point2D>> clusters = clusterPoints(points);

        for (const auto &cluster : clusters)
        {
            if (cluster.size() >= 2)
            {
                double width = calculateClusterWidth(cluster);
                if (std::abs(width - cylinder_diameter_) < 0.05)
                {
                    Point2D cylinder_center = calculateClusterCenter(cluster);
                    RCLCPP_INFO(this->get_logger(), "Cylinder center at (%.2f, %.2f)", cylinder_center.x, cylinder_center.y);

                    if (place_markers_)
                    {
                        publishMarker(cylinder_center);
                    }

                    drawCylinderOnMap(cylinder_center);
                }
            }
        }
    }

    /**
     * @brief Calculates the center of a cluster in the world frame
     * @param cluster Vector of points representing a cluster
     * @return Point2D structure containing the center point in global coordinates
     */
    Point2D calculateClusterCenter(const std::vector<Point2D> &cluster)
    {
        Point2D center = {0.0, 0.0, 0};

        for (const auto &point : cluster)
        {
            double global_x = robot_pose_.x + std::cos(robot_pose_.theta) * point.x - std::sin(robot_pose_.theta) * point.y;
            double global_y = robot_pose_.y + std::sin(robot_pose_.theta) * point.x + std::cos(robot_pose_.theta) * point.y;

            center.x += global_x;
            center.y += global_y;
        }

        center.x /= cluster.size();
        center.y /= cluster.size();

        return center;
    }

    /**
     * @brief Draws a detected cylinder on the map
     * @param cylinder_center The center point of the detected cylinder in global coordinates
     */
    void drawCylinderOnMap(const Point2D &cylinder_center)
    {
        edited_map_image_ = map_image_.clone();

        RCLCPP_INFO(this->get_logger(), "Map origin: (%.2f, %.2f)", map_origin_x_, map_origin_y_);
        RCLCPP_INFO(this->get_logger(), "Map resolution: %.2f meters/pixel", map_resolution_);
        RCLCPP_INFO(this->get_logger(), "Cylinder center: (%.2f, %.2f)", cylinder_center.x, cylinder_center.y);

        int map_x = static_cast<int>((map_origin_x_ + cylinder_center.x) / map_resolution_);
        int map_y = static_cast<int>((map_origin_y_ + cylinder_center.y) / map_resolution_);

        if (map_x < 0 || map_x >= map_image_.cols || map_y < 0 || map_y >= map_image_.rows) {
            RCLCPP_ERROR(this->get_logger(), "Invalid map coordinates: (map_x: %d, map_y: %d)", map_x, map_y);
            return;
        }

        int radius = static_cast<int>((cylinder_diameter_ / 2) / map_resolution_);
        cv::circle(edited_map_image_, cv::Point(map_x, map_y), radius, cv::Scalar(0), -1);

        cv::imshow("Map with Cylinder", edited_map_image_);
        cv::waitKey(1);
    }

    /**
     * @brief Clusters points based on proximity
     * @param points Vector of Point2D structures to be clustered
     * @return Vector of point clusters
     */
    std::vector<std::vector<Point2D>> clusterPoints(const std::vector<Point2D> &points)
    {
        std::vector<std::vector<Point2D>> clusters;
        std::vector<Point2D> current_cluster;

        for (size_t i = 0; i < points.size(); ++i)
        {
            if (current_cluster.empty())
            {
                current_cluster.push_back(points[i]);
            }
            else
            {
                double distance = calculateDistance(current_cluster.back(), points[i]);
                if (distance < clustering_threshold_)
                {
                    current_cluster.push_back(points[i]);
                }
                else
                {
                    clusters.push_back(current_cluster);
                    current_cluster.clear();
                    current_cluster.push_back(points[i]);
                }
            }
        }

        if (!current_cluster.empty())
        {
            clusters.push_back(current_cluster);
        }

        return clusters;
    }

    /**
     * @brief Calculates the Euclidean distance between two points
     * @param p1 First point
     * @param p2 Second point
     * @return Double representing the distance between points
     */
    double calculateDistance(const Point2D &p1, const Point2D &p2)
    {
        return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
    }

    /**
     * @brief Calculates the width of a cluster
     * @param cluster Vector of points representing a cluster
     * @return Double representing the width of the cluster
     */
    double calculateClusterWidth(const std::vector<Point2D> &cluster)
    {
        if (cluster.size() < 2)
            return 0.0;

        const Point2D &first_point = cluster.front();
        const Point2D &last_point = cluster.back();
        return calculateDistance(first_point, last_point);
    }

    /**
     * @brief Publishes a visualization marker for Gazebo
     * @param point The point at which to place the marker
     */
    void publishMarker(const Point2D &point)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "cylinder_marker";
        marker.id = point.index;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        double global_x = robot_pose_.x + std::cos(robot_pose_.theta) * point.x - std::sin(robot_pose_.theta) * point.y;
        double global_y = robot_pose_.y + std::sin(robot_pose_.theta) * point.x + std::cos(robot_pose_.theta) * point.y;

        marker.pose.position.x = global_x;
        marker.pose.position.y = global_y;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker_publisher_->publish(marker);
    }
};

/**
 * @brief Main function
 * @param argc Argument count
 * @param argv Argument vector
 * @return Integer representing exit status
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FilterCylinderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}