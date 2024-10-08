#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <cmath>
#include <vector>

class CylinderFilterNode : public rclcpp::Node
{
public:
    CylinderFilterNode() : Node("cylinder_filter_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        this->declare_parameter("cylinder_diameter", 0.3);
        cylinder_diameter_ = this->get_parameter("cylinder_diameter").as_double();

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&CylinderFilterNode::laserCallback, this, std::placeholders::_1));

        filtered_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("filtered_scan", 10);
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 10, std::bind(&CylinderFilterNode::mapCallback, this, std::placeholders::_1));
        updated_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("updated_map", 10);
    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto filtered_scan = *msg;
        std::vector<geometry_msgs::msg::Point> cylinder_points;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min)
            {
                double angle = msg->angle_min + i * msg->angle_increment;
                double x = msg->ranges[i] * cos(angle);
                double y = msg->ranges[i] * sin(angle);

                if (isCylinderPoint(x, y, cylinder_points))
                {
                    filtered_scan.ranges[i] = msg->range_max + 1.0;
                }
            }
        }

        filtered_scan_pub_->publish(filtered_scan);
        cylinder_position_ = calculateCylinderCenter(cylinder_points);
    }

    bool isCylinderPoint(double x, double y, std::vector<geometry_msgs::msg::Point>& cylinder_points)
    {
        for (const auto& point : cylinder_points)
        {
            double distance = std::hypot(x - point.x, y - point.y);
            if (distance <= cylinder_diameter_)
            {
                return true;
            }
        }

        geometry_msgs::msg::Point new_point;
        new_point.x = x;
        new_point.y = y;
        cylinder_points.push_back(new_point);

        if (cylinder_points.size() >= 3)
        {
            double max_distance = 0.0;
            for (size_t i = 0; i < cylinder_points.size(); ++i)
            {
                for (size_t j = i + 1; j < cylinder_points.size(); ++j)
                {
                    double distance = std::hypot(cylinder_points[i].x - cylinder_points[j].x,
                                                 cylinder_points[i].y - cylinder_points[j].y);
                    max_distance = std::max(max_distance, distance);
                }
            }

            if (max_distance <= cylinder_diameter_)
            {
                return true;
            }
        }

        return false;
    }

    geometry_msgs::msg::Point calculateCylinderCenter(const std::vector<geometry_msgs::msg::Point>& cylinder_points)
    {
        geometry_msgs::msg::Point center;
        for (const auto& point : cylinder_points)
        {
            center.x += point.x;
            center.y += point.y;
        }
        center.x /= cylinder_points.size();
        center.y /= cylinder_points.size();
        return center;
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        auto updated_map = *msg;

        try
        {
            geometry_msgs::msg::TransformStamped transform = tf_buffer_.lookupTransform(
                msg->header.frame_id, "base_link", tf2::TimePointZero);

            double map_x = (cylinder_position_.x + transform.transform.translation.x - msg->info.origin.position.x) / msg->info.resolution;
            double map_y = (cylinder_position_.y + transform.transform.translation.y - msg->info.origin.position.y) / msg->info.resolution;

            int radius = static_cast<int>(cylinder_diameter_ / (2 * msg->info.resolution));

            for (int dy = -radius; dy <= radius; ++dy)
            {
                for (int dx = -radius; dx <= radius; ++dx)
                {
                    if (dx * dx + dy * dy <= radius * radius)
                    {
                        int cell_x = static_cast<int>(map_x) + dx;
                        int cell_y = static_cast<int>(map_y) + dy;

                        if (cell_x >= 0 && cell_x < static_cast<int>(msg->info.width) &&
                            cell_y >= 0 && cell_y < static_cast<int>(msg->info.height))
                        {
                            updated_map.data[cell_y * msg->info.width + cell_x] = 100;  // Mark as occupied
                        }
                    }
                }
            }
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform cylinder position: %s", ex.what());
        }

        updated_map_pub_->publish(updated_map);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_scan_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr updated_map_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    double cylinder_diameter_;
    geometry_msgs::msg::Point cylinder_position_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylinderFilterNode>());
    rclcpp::shutdown();
    return 0;
}