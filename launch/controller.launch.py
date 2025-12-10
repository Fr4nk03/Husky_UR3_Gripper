import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

def generate_launch_description():
    # --- Configuration ---
    # Increased delay to ensure the controller_manager services are fully available
    CONTROLLER_SPAWNER_DELAY = 2.0 
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    use_sim_time_config = LaunchConfiguration('use_sim_time')

    pkg_name = 'husky_ur3_gripper_description'
    pkg_path = get_package_share_directory(pkg_name)
    
    # Get the URDF and run XACRO command
    urdf_file = os.path.join(pkg_path, 'urdf', 'husky_ur3_gripper_old.urdf.xacro')
    
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file, ' ',
    ])
    
    # Wrap in ParameterValue
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str
        )
    }
    
    
    # Get Controller Configuration
    controllers_config_file = os.path.join(pkg_path, 'config', 'husky_ur3_controllers.yaml')
    
    
    # Spawner for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time_config}],
        output='screen'
    )
    
    # Spawner for husky_velocity_controller
    husky_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['husky_velocity_controller', '-c', '/controller_manager'],
        parameters=[{'use_sim_time': use_sim_time_config}],
        output='screen'
    )
    
    # Spawn husky_velocity_controller after joint_state_broadcaster exits successfully
    delayed_husky_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[husky_velocity_controller_spawner],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,    
        # The main control node must start first
        #control_node, 
        
        # Spawners start AFTER a delay to give the controller_manager time to initialize
        # gripper_spawner_delayed,
        #main_controller_spawner_delayed
        
        joint_state_broadcaster_spawner,
        delayed_husky_controller_spawner
    ])
