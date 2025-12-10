import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # --- 1. Get configuration files ---
    # NOTE: You must replace 'your_robot_package' with your actual package name
    pkg_share_dir = get_package_share_directory('husky_ur3_gripper_description') 
    
    # Define paths to the YAML configuration files
    ekf_local_config = os.path.join(pkg_share_dir, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_share_dir, 'config', 'ekf_global.yaml')
    navsat_config = os.path.join(pkg_share_dir, 'config', 'navsat_transform.yaml')
    
    local_odometry_topic = 'odom/filtered_local'
    navsat_output_topic = 'odometry/gps_fused'
    global_odometry_topic = '/robot_pose'

    # --- 2. Local EKF Node (ekf_local_node) ---
    # This filter runs at a high rate to produce smooth odometry (odom -> base_link).
    # It fuses IMU (for high-rate orientation) and Wheel Odometry/VO (for XY velocity).
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        output='screen',
        parameters=[ekf_local_config],
        # Remaps ensure clear separation of inputs/outputs
        remappings=[
        	('odometry/filtered', local_odometry_topic),
        ]
    )

    # --- 3. Navsat Transform Node ---
    # Converts GPS Lat/Lon to an Odometry message in a local metric frame.
    # The output topic (/gps/filtered) is consumed by the Global EKF.
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[navsat_config],
        # Re-mappings for inputs
        remappings=[
        	('odometry/filtered', local_odometry_topic),
        	('odometry/gps', navsat_output_topic),
        	('gps/fix', 'gps/fix'),         
        ]
    )
    
    # --- 4. Global EKF Node (ekf_global_node) ---
    # This filter fuses the Local EKF's smooth output with the globally-corrected GPS data.
    # It solves for the map -> odom transform.
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        output='screen',
        parameters=[ekf_global_config],
        # Remaps for inputs/outputs
        remappings=[
       		('odometry/filtered', global_odometry_topic),  
        ]
    )

    # --- 5. Return the Launch Description ---
    return LaunchDescription([
        ekf_local_node,
        navsat_transform_node,
        ekf_global_node,
    ])
