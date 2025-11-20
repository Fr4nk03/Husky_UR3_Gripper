from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    
    pkg_name = 'husky_ur3_gripper_description'
    pkg_path = get_package_share_directory(pkg_name)
    
    # 1. Declare Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # 2. Include the Simulation Setup (Gazebo, Spawner, RSP)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_path, 'launch', 'start_simulation.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 3. Include the Control Setup (Controller Manager, Controllers)
    '''
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_path, 'launch', 'controller.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    '''
    
    control_launch = TimerAction(
        period=7.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(pkg_path, 'launch', 'controller.launch.py')]),
                launch_arguments={'use_sim_time': use_sim_time}.items()
            )
        ]
    )
    
    # 4. Include the Teleoperation Interface
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(pkg_path, 'launch', 'teleop.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        
        simulation_launch,
        control_launch,
        teleop_launch,
    ])
