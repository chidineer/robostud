from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to your map file
    map_file = os.path.join(
        get_package_share_directory('amcl_localisation'),
        'config',
        'warehouse_map.yaml'
    )

    # Path to your AMCL parameters file
    amcl_params_file = os.path.join(
        get_package_share_directory('amcl_localisation'),
        'config',
        'amcl_params.yaml'
    )

    # Nodes to launch
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_file, {'use_sim_time': True}]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': map_file}]
    )

    return LaunchDescription([
        map_server_node,
        amcl_node,
    ])
