import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launches the teleoperation nodes for the Husky-UR3 robot."""
    TELEOP_SPAWNER_DELAY = 5.0
    
    
    # Define the package that contains the custom gripper control node
    # gripper_pkg_name = LaunchConfiguration('gripper_pkg_name', default='husky_ur3_gripper_description')
    
    # 1. Teleop Keyboard for Mobile Base
    # This node publishes geometry_msgs/Twist messages to the '/cmd_vel' topic.
    teleop_keyboard_node = TimerAction(
        period=TELEOP_SPAWNER_DELAY,
        actions=[
            Node(
                package='teleop_twist_keyboard',
                executable='teleop_twist_keyboard',
                name='teleop_twist_keyboard',
                prefix='xterm -e', 
                output='screen',
        

                #remappings=[
                    #('/cmd_vel', '/husky_velocity_controller/cmd_vel_unstamped')
                #]

            )
        ]
    )
    
    '''
    # 2. Arm & Gripper Teleop: Uses GUI sliders to publish Float64 position commands
    # This node detects all joints in the URDF and needs to remap its default joint-name topics 
    # to the specific /cmd_pos topics the Ignition plugins are listening to.
    teleop_arm_gripper_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='arm_gripper_teleop_gui',
        output='screen',
        # Map the default joint command topic (the joint name) to the specific 
        # command topic the bridge/plugin expects (/joint_name/cmd_pos).
        remappings=[
            ('ur3_shoulder_pan_joint', '/ur3_shoulder_pan_joint/cmd_pos'),
            ('ur3_shoulder_lift_joint', '/ur3_shoulder_lift_joint/cmd_pos'),
            ('ur3_elbow_joint', '/ur3_elbow_joint/cmd_pos'),
            ('ur3_wrist_1_joint', '/ur3_wrist_1_joint/cmd_pos'),
            ('ur3_wrist_2_joint', '/ur3_wrist_2_joint/cmd_pos'),
            ('ur3_wrist_3_joint', '/ur3_wrist_3_joint/cmd_pos'),
            # Gripper joint
            ('rh_r1', '/rh_r1/cmd_pos'), 
        ]
    )
    
    # 2. Gripper Controller (Custom Node)
    gripper_controller_node = Node(
        package=gripper_pkg_name,
        executable='gripper_controller',
        name='gripper_controller',
        output='screen'
    )
    '''
    
    return LaunchDescription([
        teleop_keyboard_node,
        # gripper_controller_node
        # teleop_arm_gripper_gui
    ])
