import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    
    # NVIDIA GPU Offloading
    set_nv_prime = SetEnvironmentVariable(
        name='__NV_PRIME_RENDER_OFFLOAD',
        value='1'
    )
    
    set_glx_vendor = SetEnvironmentVariable(
        name='__GLX_VENDOR_LIBRARY_NAME',
        value='nvidia'
    )
    
    # Qt5 OpenGL Fix
    set_qt_xcb = SetEnvironmentVariable(
        name='QT_XCB_GL_INTEGRATION',
        value='xcb_glx'
    )
    
    # Software Rendering Fallback (use if GPU fails)
    set_libgl_software = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='0'  # Set to '1' if you need software rendering
    )
    
    # Ignition Gazebo specific rendering settings
    set_ign_rendering = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_INDIRECT',
        value='0'
    )
    
    
    # Package paths
    
    pkg_path = get_package_share_directory('husky_ur3_gripper_description')
    husky_description_path = get_package_share_directory('husky_description')
    ur_description_path = get_package_share_directory('ur_description')
    gripper_description_path = get_package_share_directory('rh_p12_rn_a_description')
    
    # Set Gazebo resource paths to find meshes
    set_gazebo_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=':'.join([
            os.path.join(husky_description_path, '..'),
            os.path.join(ur_description_path, '..'),
            os.path.join(gripper_description_path, '..'),
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')
        ])
    )
    
    '''
    # Gazebo plugin paths
    set_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH', 
        value='/opt/ros/humble/lib/'
    )
    
    
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH', 
        value='/opt/ros/humble/lib/'
    )
    '''
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Process URDF with xacro
    urdf_file = os.path.join(pkg_path, 'urdf', 'husky_ur3_gripper_old.urdf.xacro')
    
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_file
    ])
    
    # Wrap in ParameterValue
    robot_description = {
        'robot_description': ParameterValue(
            robot_description_content, value_type=str
        )
    }
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )
    
    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', 'empty.sdf', '-r', '-v', '3'], 
        output='screen'
    )
    
    # Spawn robot - DELAYED to wait for Gazebo
    spawn_robot = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'husky_ur3',
                    '-topic', 'robot_description',
                    '-z', '0.3'
                ],
                output='screen'
            )
        ]
    )
    
    
    # ROS-Gazebo Bridge - DELAYED
    bridge = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='ros_gz_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
                    '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                    #'/husky_velocity_controller/cmd_vel_unstamped@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    
                    '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
                    '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    
                    '/rh_p12_rn_position/command@std_msgs/msg/Float64@gz.msgs.Double',
                    #'/rh_p12_rn_position/command@std_msgs/msg/Float64MultiArray@gz.msgs.Double'
                ],
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    

    # Return LaunchDescription
    return LaunchDescription([
        set_gazebo_resource_path,
        #set_plugin_path,
        declare_use_sim_time,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        bridge
    ])
