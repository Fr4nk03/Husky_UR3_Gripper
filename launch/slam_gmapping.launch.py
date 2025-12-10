from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Find the directory where the gmapping package is installed (if available in ROS 2)
    # Note: If you are using the official gmapping_ros2 wrapper, replace 'gmapping' 
    # with the actual package name it uses (e.g., 'slam_gmapping').
    # For this example, we assume the node executable is called 'slam_gmapping'.

    return LaunchDescription([
        Node(
            # This is often the name of the ROS 2 wrapper package for gmapping
            package='slam_gmapping', 
            # The executable name within that package
            executable='slam_gmapping', 
            name='gmapping_node',
            output='screen',
            parameters=[
                {'use_sim_time': True}, 
                {'odom_frame': 'odom'}, 
                {'base_frame': 'base_link'},
                {'map_frame': 'map'},
                {'laser_frame': 'husky_ur3/base_link/base_laser'},
            ],
            remappings=[
                #('/scan', '/my_laser_scan_topic'),
            ],
        ),
    ])
