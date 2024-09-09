// ROS Includes
#include <rclcpp/rclcpp.hpp>  
#include <sensor_msgs/msg/laser_scan.hpp>  
#include <geometry_msgs/msg/twist.hpp> 
#include <opencv2/opencv.hpp>  
#include <yaml-cpp/yaml.h>  
#include <nav_msgs/msg/odometry.hpp>  
#include <std_msgs/msg/float64.hpp>  
#include <fstream>  
#include <vector>  

class ScanMatchingLocaliser : public rclcpp::Node 
{ 
public:
    ScanMatchingLocaliser(const std::string& map_yaml_path) 
        : Node("scan_matching_localiser"), angle_difference_(0.0), relative_orientation_(0.0), move_called_(false) {

        // Set up subs
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ScanMatchingLocaliser::scanCallback, this, std::placeholders::_1));

        // Set up pubs
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        yaw_pub_ = this->create_publisher<std_msgs::msg::Float64>("/localised_yaw", 10);

        // Initialise Map File at startup, save in memory
        loadMapFromFile(map_yaml_path);
        // Set robot's initlal poition
        initial_position_ = cv::Point2d(1.0, -4.0);
        RCLCPP_INFO(this->get_logger(), "Scan Matcher Node started with map %s.", map_yaml_path.c_str());
        
        first_image_captured_ = false;
        second_image_captured_ = false;
    
    }

private:


    // Ren when laser scan. Quasi Main Func
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        // Convert LaserScan to cv::Mat (polar coordinates to Cartesian)
        cv::Mat img = laserScanToMat(msg);
        
        // If first frame
        if (!first_image_captured_) 
        {
            // Get Image A - Map Section around robot
            first_image_ = extractMapSection(initial_position_);  // Extracts a section of the map around the robot (Image A)
            first_image_captured_ = true;

            // Get Image B - Edges from Image A
            first_image_edges_ = extractEdges(first_image_);  // Applies edge detection to Image A
            
            // Display the first image and its edges
            cv::imshow("A Image", first_image_);
            cv::imshow("A Image Edges (B)", first_image_edges_);
            cv::waitKey(1); // Process GUI events and update the window
        } 
        else if (!second_image_captured_) 
        {
            second_image_ = img.clone();  // Store the laser scan as the second image (Image C)
            second_image_captured_ = true;

            // Display the second image (laser scan)
            cv::imshow("C Image (Laser Scan)", second_image_);
            cv::waitKey(1);

            // Get Image C - Detect and match features between the first and second images
            calculateYawChange();

            // Move the robot based on the yaw difference
            moveForward();

            // Save images for debugging/visualization
            saveImages(first_image_edges_, second_image_, "/home/chidalu/uni/robostud/sprint_2_localiser/src");

            // Update first image with the second image for the next step
            first_image_ = second_image_.clone();
            first_image_edges_ = extractEdges(first_image_);
        } 
        else 
        {
            // If images are captured, we update the position and orientation
            // Rotate and move the robot
            moveForward();

            // Update first and second images with the new laser scan and map section
            first_image_ = second_image_.clone();  // Use the previous second image as the new first image
            second_image_ = img.clone();  // Update the second image with the current laser scan
            
            // Display the new second image
            cv::imshow("Second Image (Laser Scan)", second_image_);
            cv::waitKey(1);

            // Calculate yaw change again
            calculateYawChange();

            // Save images
            saveImages(first_image_edges_, second_image_, "/home/chidalu/uni/robostud/sprint_2_localiser/src");
        }

        // Update total relative orientation after each iteration
        relative_orientation_ += angle_difference_;
        relative_orientation_ = normalizeAngle(relative_orientation_);  // Normalize the total orientation

        // Log and publish the total relative orientation in degrees
        double relative_orientation_degrees = relative_orientation_ * 180.0 / CV_PI;
        RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f degrees", relative_orientation_degrees);

        // Publish the yaw to /localised_yaw for plotting
        std_msgs::msg::Float64 yaw_msg;
        yaw_msg.data = relative_orientation_degrees;
        yaw_pub_->publish(yaw_msg);
    }

    // Load map
    void loadMapFromFile(const std::string& yaml_file) 
    {
        YAML::Node map_config = YAML::LoadFile(yaml_file);

        std::string image_file = map_config["image"].as<std::string>();
        double resolution = map_config["resolution"].as<double>();
        std::vector<double> origin = map_config["origin"].as<std::vector<double>>();

        map_image_ = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
        if (map_image_.empty()) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load map image from %s", image_file.c_str());
            return;
        }

        map_resolution_ = resolution;
        map_origin_ = cv::Point2d(origin[0], origin[1]);

        RCLCPP_INFO(this->get_logger(), "Map loaded successfully from %s", yaml_file.c_str());
    }

    // Function to extract a section of the map around the robot
    cv::Mat extractMapSection(const cv::Point2d& robot_pose) {
        int size = 100;  // Define the size of the section to extract (in pixels)
        int x_start = std::max(0, int(robot_pose.x - size / 2));
        int y_start = std::max(0, int(robot_pose.y - size / 2));
        int x_end = std::min(map_image_.cols, int(robot_pose.x + size / 2));
        int y_end = std::min(map_image_.rows, int(robot_pose.y + size / 2));

        cv::Rect region(x_start, y_start, x_end - x_start, y_end - y_start);
        return map_image_(region).clone();  // Extract and return the section of the map
    }

    // Function to extract edges from the map section using OpenCV
    cv::Mat extractEdges(const cv::Mat& image) {
        cv::Mat edges;
        cv::Canny(image, edges, 50, 150);  // Apply Canny edge detection
        return edges;
    }

    // Function to convert laser scans to an image (Image C)
    cv::Mat laserScanToMat(const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        int img_size = 500;
        float max_range = scan->range_max;
        cv::Mat image = cv::Mat::zeros(img_size, img_size, CV_8UC1);

        for (size_t i = 0; i < scan->ranges.size(); i++) {
            float range = scan->ranges[i];
            if (range < scan->range_min || range > scan->range_max) {
                continue;
            }

            float angle = scan->angle_min + i * scan->angle_increment;
            int x = static_cast<int>((range * cos(angle)) * img_size / (2 * max_range)) + img_size / 2;
            int y = static_cast<int>((range * sin(angle)) * img_size / (2 * max_range)) + img_size / 2;

            if (x >= 0 && x < img_size && y >= 0 && y < img_size) {
                image.at<uchar>(y, x) = 255;
            }
        }

        return image;
    }

    // Detect and match features between the first and second images
    void calculateYawChange() {
        std::vector<cv::Point2f> srcPoints, dstPoints;
        detectAndMatchFeatures(first_image_edges_, second_image_, srcPoints, dstPoints);

        if (srcPoints.size() < 3 || dstPoints.size() < 3) {
            RCLCPP_ERROR(this->get_logger(), "Not enough points for yaw estimation.");
            return;
        }

        try {
            cv::Mat transform_matrix = cv::estimateAffinePartial2D(srcPoints, dstPoints);
            if (!transform_matrix.empty()) {
                angle_difference_ = atan2(transform_matrix.at<double>(1, 0), transform_matrix.at<double>(0, 0));
                relative_orientation_ += angle_difference_;
                relative_orientation_ = normalizeAngle(relative_orientation_);

                // double relative_orientation_degrees = relative_orientation_ * 180.0 / CV_PI;
                // RCLCPP_INFO(this->get_logger(), "Relative Orientation: %f degrees", relative_orientation_degrees);

                // std_msgs::msg::Float64 yaw_msg;
                // yaw_msg.data = relative_orientation_degrees;
                // yaw_pub_->publish(yaw_msg);
            }
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error in yaw change calculation: %s", e.what());
        }
    }

    // Rotate the robot
    void rotateRobot() {
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.angular.z = 0.5;  // Set angular velocity for rotation
        cmd_pub_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::seconds(2));  // Rotate for 2 seconds

        // Stop rotation
        twist_msg.angular.z = 0.0;
        cmd_pub_->publish(twist_msg);
        RCLCPP_INFO(this->get_logger(), "Rotation completed.");
    }

    // Move the robot forward
    void moveForward() {
        double distance_moved = 0.0;
        double target_distance = 6.28; // Circumference of the circle (2 * PI * radius, with radius = 1m)
        double linear_speed = -0.1;  // Linear speed in m/s
        double angular_speed = linear_speed / 1.0; // Angular speed, keeping radius of 1 meter
        double time_step = 0.1;  // Time step for the control loop

        auto twist_msg = geometry_msgs::msg::Twist();
        rclcpp::Rate rate(1.0 / time_step);

        twist_msg.linear.x = linear_speed;
        twist_msg.angular.z = angular_speed;  // Apply angular velocity for circular motion

        cmd_pub_->publish(twist_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(time_step * 1000)));
        distance_moved += linear_speed * time_step;  // Update distance traveled

        twist_msg.linear.x = 0.0;
        twist_msg.angular.z = 0.0;
        cmd_pub_->publish(twist_msg);

        // Update the robot's position on the map
        initial_position_.x += distance_moved / map_resolution_ * cos(relative_orientation_);
        initial_position_.y += distance_moved / map_resolution_ * sin(relative_orientation_);
    }

    // Function to normalize angles to the range [-π, π]
    double normalizeAngle(double theta) {
        while (theta > M_PI) {
            theta -= 2.0 * M_PI;
        }
        while (theta < -M_PI) {
            theta += 2.0 * M_PI;
        }
        return theta;
    }

    // Function to detect and match features between two images
    void detectAndMatchFeatures(const cv::Mat& img1, const cv::Mat& img2,
                                std::vector<cv::Point2f>& srcPoints, std::vector<cv::Point2f>& dstPoints) {
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;

        orb->detectAndCompute(img1, cv::noArray(), keypoints1, descriptors1);
        orb->detectAndCompute(img2, cv::noArray(), keypoints2, descriptors2);

        cv::BFMatcher matcher(cv::NORM_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher.match(descriptors1, descriptors2, matches);

        std::sort(matches.begin(), matches.end(), [](const cv::DMatch& a, const cv::DMatch& b) {
            return a.distance < b.distance;
        });

        size_t numGoodMatches = static_cast<size_t>(matches.size() * 0.15);
        std::vector<cv::DMatch> goodMatches(matches.begin(), matches.begin() + numGoodMatches);

        for (const auto& match : goodMatches) {
            srcPoints.push_back(keypoints1[match.queryIdx].pt);
            dstPoints.push_back(keypoints2[match.trainIdx].pt);
        }
    }

    // Save images for debugging and visualization
void saveImages(const cv::Mat& first_image_edges, const cv::Mat& second_image, const std::string& save_path)
{
    std::string first_image_path = save_path + "/first_image_edges.png";
    std::string second_image_path = save_path + "/second_image.png";

    cv::imwrite(first_image_path, first_image_edges);
    cv::imwrite(second_image_path, second_image);

    // RCLCPP_INFO(this->get_logger(), "Images saved: %s and %s", first_image_path.c_str(), second_image_path.c_str());
}
    // Subs and Pubs
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_pub_;

    cv::Mat map_image_;
    double map_resolution_; 
    cv::Point2d map_origin_;  
    cv::Point2d initial_position_;  

    cv::Mat first_image_, second_image_;  
    cv::Mat first_image_edges_;  

    bool first_image_captured_ = false;
    bool second_image_captured_ = false;

    double angle_difference_; 
    double relative_orientation_ = 0.0; 

    bool move_called_;  
};

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Missing MAP YAML File");
        return -1;
    }

    std::string map_yaml_path = argv[1];
    rclcpp::spin(std::make_shared<ScanMatchingLocaliser>(map_yaml_path));
    rclcpp::shutdown();
    return 0;
}
