#include "autonomous_mapping.hpp"

AutonomousMapping::AutonomousMapping() : Node("autonomous_mapping")
{
    // Record start time
    auto start_time = std::chrono::high_resolution_clock::now();
    auto start_setup = std::chrono::high_resolution_clock::now();

    // Create services for restarting exploration and saving map
    restart_explore_service_ = this->create_service<std_srvs::srv::Trigger>(
        "restart_explore_lite", std::bind(&AutonomousMapping::restartExploreLiteService, this, std::placeholders::_1, std::placeholders::_2));

    save_map_service_ = this->create_service<std_srvs::srv::Trigger>(
        "save_map", std::bind(&AutonomousMapping::saveMapService, this, std::placeholders::_1, std::placeholders::_2));

    startEnvironment();   // Start the environment, then SLAM and Nav2

    auto end_setup = std::chrono::high_resolution_clock::now();

    // Calculate and print the elapsed time
    auto setup_duration = std::chrono::duration_cast<std::chrono::seconds>(end_setup - start_setup);

    int explore_iterations = 10;  // Number of times to run explore lite
    int explore_duration = 180;  // Duration for each run in seconds

    for (int i = 0; i < explore_iterations; ++i)
    {
        runExploreLite();     // Start exploration
        std::this_thread::sleep_for(std::chrono::seconds(explore_duration));
        killExploreLite();
    }
    // Record end time
    auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate and print the elapsed time
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time);
    
    RCLCPP_INFO(this->get_logger(), "Setup completed in %ld seconds", setup_duration.count());
    RCLCPP_INFO(this->get_logger(), "Mapping task completed in %ld seconds", duration.count());
    saveMap();

    
}

void AutonomousMapping::restartExploreLiteService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    killExploreLite();
    runExploreLite();
    response->success = true;
    response->message = "Explore lite restarted.";
}

void AutonomousMapping::saveMapService(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                       std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    saveMap();
    response->success = true;
    response->message = "Map saved successfully.";
}

void AutonomousMapping::startEnvironment()
{
    RCLCPP_INFO(this->get_logger(), "Starting TurtleBot3 simulation environment...");

    // Launch Nav2 (run in background)
    RCLCPP_INFO(this->get_logger(), "Starting Nav2...");
    std::string nav2_cmd = "ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True &";
    int nav2_status = system(nav2_cmd.c_str());
    if (nav2_status != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start Nav2.");
        return;
    }

    // Wait for Nav2 to start properly
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Launch SLAM toolbox (run in background)
    RCLCPP_INFO(this->get_logger(), "Starting SLAM Toolbox...");
    std::string slam_cmd = "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True &";
    int slam_status = system(slam_cmd.c_str());
    if (slam_status != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start SLAM toolbox.");
        return;
    }

    // Wait for Nav2 to start properly
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Launch SLAM toolbox (run in background)
    RCLCPP_INFO(this->get_logger(), "Starting Rviz2...");
    std::string rviz_cmd = "ros2 run rviz2 rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz &";
    int rviz_status = system(rviz_cmd.c_str());
    if (rviz_status != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start SLAM toolbox.");
        return;
    }

    std::this_thread::sleep_for(std::chrono::seconds(5));
}

void AutonomousMapping::killExploreLite()
{
    RCLCPP_INFO(this->get_logger(), "Killing explore lite...");
    system("pkill -f explore_lite");
}

void AutonomousMapping::runExploreLite()
{
    RCLCPP_INFO(this->get_logger(), "Running explore lite...");
    std::string explore_cmd = "ros2 launch explore_lite explore.launch.py use_sim_time:=true &";
    int explore_status = system(explore_cmd.c_str());
    if (explore_status != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start explore lite.");
    }
}

void AutonomousMapping::saveMap()
{
    std::string map_folder = "/home/student/ros2_ws/src/autonomous_robot/map/";

    // Check if map.pgm and map.yaml already exist and delete them
    std::string map_pgm_file = map_folder + "map.pgm";
    std::string map_yaml_file = map_folder + "map.yaml";
    
    if (std::filesystem::exists(map_pgm_file)) {
        RCLCPP_INFO(this->get_logger(), "Deleting existing map.pgm...");
        std::filesystem::remove(map_pgm_file);
    }
    if (std::filesystem::exists(map_yaml_file)) {
        RCLCPP_INFO(this->get_logger(), "Deleting existing map.yaml...");
        std::filesystem::remove(map_yaml_file);
    }

    // Save the new map using map_saver_cli
    std::string save_cmd = "ros2 run nav2_map_server map_saver_cli -f " + map_folder + "map";
    int save_status = system(save_cmd.c_str());
    if (save_status != 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to save the map.");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Map saved successfully at %s", map_pgm_file.c_str());

    // Modify map.yaml to use an absolute path for the .pgm file
    std::ifstream yaml_file_in(map_yaml_file);
    std::ofstream yaml_file_out(map_folder + "map_temp.yaml");
    
    if (!yaml_file_in.is_open() || !yaml_file_out.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open map.yaml for modification.");
        return;
    }

    std::string line;
    while (std::getline(yaml_file_in, line)) {
        // Look for the line that starts with "image:" and replace the relative path with the absolute path
        if (line.rfind("image:", 0) == 0) {  // Line starts with "image:"
            yaml_file_out << "image: " << map_pgm_file << std::endl;
        } else {
            yaml_file_out << line << std::endl;
        }
    }

    yaml_file_in.close();
    yaml_file_out.close();

    // Replace the original map.yaml with the modified one
    std::filesystem::remove(map_yaml_file);  // Remove old map.yaml
    std::filesystem::rename(map_folder + "map_temp.yaml", map_yaml_file);  // Rename the temp file

    // Display the saved map using OpenCV
    cv::Mat map_image = cv::imread(map_pgm_file, cv::IMREAD_GRAYSCALE);
    if (!map_image.empty()) {
        cv::imshow("Saved Map", map_image);
        cv::waitKey(0);  // Wait for user to close the window
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to load and display the map image.");
    }
}
