import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    pkg_path = get_package_share_directory('husky_ur3_gripper_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'husky_ur3_gripper.urdf.xacro')
    
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ])
    
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str
        )
    }
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )
    
    return LaunchDescription([
        robot_state_publisher
    ])
