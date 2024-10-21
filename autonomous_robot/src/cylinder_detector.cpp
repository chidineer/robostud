#include "cylinder_detector.hpp"

CylinderDetector::CylinderDetector(){
    
}

// Detect the Cylinder Location
CylinderDetector::Point2D CylinderDetector::detectCylinder(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    Point2D no_cylinder_found;
    no_cylinder_found.x = 1e10;
    no_cylinder_found.y = 1e10;

    // Add error handling to check scan message
    if (!scan_msg || scan_msg->ranges.empty()) {
        return no_cylinder_found;
    }

    std::vector<Point2D> points;
    std::vector<int> cylinder_indices;

    // Convert ranges to 2D points in the robot's coordinate frame
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
        double range = scan_msg->ranges[i];
        // Skip invalid or infinite ranges
        if (range < scan_msg->range_min || range > scan_msg->range_max)
            continue;

        // Calculate the angle corresponding to the current range
        double angle = scan_msg->angle_min + i * scan_msg->angle_increment;

        // Convert to 2D point and store
        points.push_back(rangeToPoint(range, angle, i));
    }

    // Cluster points based on proximity
    std::vector<std::vector<Point2D>> clusters = clusterPoints(points);

    // Identify the clusters that match the cylinder's width
    for (const auto &cluster : clusters)
    {
        if (cluster.size() >= 2)
        {
            double width = calculateClusterWidth(cluster);
            if (std::abs(width - cylinder_diameter_) < 0.05) // Allow 5 cm tolerance
            {
                Point2D cylinder_center = calculateClusterCenter(cluster);
                // RCLCPP_INFO(this->get_logger(), "Cylinder center at (%.2f, %.2f)", cylinder_center.x, cylinder_center.y);
                return cylinder_center;
            }
        }
    }
    return no_cylinder_found;
}

// Function to convert a range into a 2D point in the robot's coordinate frame
CylinderDetector::Point2D CylinderDetector::rangeToPoint(double range, double angle, int index)
{
    Point2D point;
    point.x = range * std::cos(angle);
    point.y = range * std::sin(angle);
    point.index = index;
    return point;
}

// Function to cluster points based on their proximity
std::vector<std::vector<CylinderDetector::Point2D>> CylinderDetector::clusterPoints(const std::vector<Point2D> &points)
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

// Function to calculate the distance between two 2D points
double CylinderDetector::calculateDistance(const Point2D &p1, const Point2D &p2)
{
    return std::sqrt(std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2));
}

// Function to calculate the width of a cluster (distance between first and last point)
double CylinderDetector::calculateClusterWidth(const std::vector<Point2D> &cluster)
{
    if (cluster.size() < 2)
        return 0.0;

    const Point2D &first_point = cluster.front();
    const Point2D &last_point = cluster.back();
    return calculateDistance(first_point, last_point);
}

// Function to calculate the center of a cluster in the world frame
CylinderDetector::Point2D CylinderDetector::calculateClusterCenter(const std::vector<CylinderDetector::Point2D> &cluster)
{
    Point2D center = {0.0, 0.0, 0};

    for (const auto &point : cluster)
    {
        center.x += point.x;
        center.y += point.y;
    }

    // Compute the average (center) of the transformed points
    center.x /= cluster.size();
    center.y /= cluster.size();

    return center;
}