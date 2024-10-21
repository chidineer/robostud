#ifndef CYLINDER_DETECTOR_HPP
#define CYLINDER_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <vector>
#include <string>

class CylinderDetector
{
public:
    CylinderDetector();

    struct Point2D
    {
        double x;
        double y;
        int index; // To keep track of the original range index
    };

    struct Pose2D
    {
        double x;
        double y;
        double theta;
    } robot_pose_;

    Point2D rangeToPoint(double range, double angle, int index);
    Point2D calculateClusterCenter(const std::vector<Point2D> &cluster);
    std::vector<std::vector<Point2D>> clusterPoints(const std::vector<Point2D> &points);
    double calculateDistance(const Point2D &p1, const Point2D &p2);
    double calculateClusterWidth(const std::vector<Point2D> &cluster);
    Point2D detectCylinder(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    

private:

    double clustering_threshold_;
    double cylinder_diameter_;
    bool place_markers_;

};

#endif // CLYINDER_DETECTOR_HPP
