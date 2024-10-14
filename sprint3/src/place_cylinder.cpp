/**
 * @file place_cylinder_node.cpp
 * @brief Node for spawning a cylinder in Gazebo at specified coordinates
 * 
 * This node creates a cylinder model in Gazebo at specified x and y coordinates
 * using the spawn_entity service. The cylinder has a diameter of 30 cm and a height of 1 m.
 */

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <cstdlib>

/**
 * @brief Node class for placing cylinders in Gazebo simulation
 * 
 * This class creates a ROS 2 node that can spawn cylinder models in Gazebo
 * at specified coordinates using the Gazebo spawn_entity service.
 */
class PlaceCylinderNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for PlaceCylinderNode
     * 
     * Initializes the node, declares parameters for x and y coordinates,
     * and triggers the cylinder spawning process.
     */
    PlaceCylinderNode() : Node("place_cylinder")
    {
        // Declare parameters for x and y coordinates
        this->declare_parameter<double>("x", 0.0);
        this->declare_parameter<double>("y", 0.0);

        // Spawn the cylinder
        putCylinder();
    }

private:
    /**
     * @brief Spawns a cylinder in Gazebo using the spawn_entity service
     * 
     * This function retrieves x and y coordinates from parameters, creates an SDF model
     * description for the cylinder, and calls the Gazebo spawn_entity service to create
     * the cylinder in the simulation.
     */
    void putCylinder()
    {
        // Retrieve x and y coordinates from the parameters
        double x, y;
        this->get_parameter("x", x);
        this->get_parameter("y", y);
        double z = 0.0; // Always 0 for z

        // Create a client to call the /spawn_entity service in Gazebo
        auto client = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
        while (!client->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the /spawn_entity service to be available...");
        }

        // Create a request for spawning the cylinder
        auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
        request->name = "cylinder";
        request->xml = R"(
            <sdf version="1.6">
                <model name="cylinder">
                    <pose>0 0 0 0 0 0</pose>
                    <link name="link">
                        <inertial>
                            <mass>1.0</mass>
                        </inertial>
                        <collision name="collision">
                            <geometry>
                                <cylinder>
                                    <radius>0.15</radius> <!-- 30 cm diameter -->
                                    <length>1.0</length>  <!-- 1 m tall -->
                                </cylinder>
                            </geometry>
                        </collision>
                        <visual name="visual">
                            <geometry>
                                <cylinder>
                                    <radius>0.15</radius>
                                    <length>1.0</length>
                                </cylinder>
                            </geometry>
                        </visual>
                    </link>
                </model>
            </sdf>
        )";

        // Set the pose for the cylinder (x, y, z)
        request->initial_pose.position.x = x;
        request->initial_pose.position.y = y;
        request->initial_pose.position.z = z;
        request->initial_pose.orientation.w = 1.0;

        // Call the service and wait for response
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Cylinder placed at (%f, %f, %f)", x, y, z);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to place the cylinder");
        }
    }
};

/**
 * @brief Main function
 * @param argc Argument count
 * @param argv Argument vector
 * @return Integer representing exit status
 * 
 * Initializes ROS 2, creates the PlaceCylinderNode, and spins the node.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the node and set x and y coordinates as arguments
    auto node = std::make_shared<PlaceCylinderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}