import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # Define the path to the default kinematics config file
    ur_description_pkg_path = get_package_share_directory('ur_description')

    # Get the path to your new URDF file
    pkg_path = get_package_share_directory('husky_ur3_gripper_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'husky_ur3_gripper_old.urdf.xacro')
    
    # Process the xacro file into a URDF string
    robot_description_config = xacro.process_file(
        xacro_file
    )
    robot_description = robot_description_config.toxml()

    # Get the path to a basic RViz config file
    ur_viz_pkg_path = get_package_share_directory('ur_description')
    rviz_config_file = os.path.join(ur_viz_pkg_path, 'rviz', 'view_robot.rviz')

    # ========== NODES TO LAUNCH ============

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}] 
    )

    # 2. Joint State Publisher GUI
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 3. RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
