# Husky_UR3_Gripper
A husky mobile base integrated with UR3 arms, Robotis RH-P12-RN Gripper, and a ZED2 Camera

# Get Started
```
- Bring up Gazebo with the robot model
$ ros2 launch husky_ur3_gripper_description start_simulation.launch.py

- Teleoperation
$ ros2 launch husky_ur3_gripper_description teleop.launch.py

- Bring up RViz
$ rviz2

OR
$ ros2 launch husky_ur3_gripper_description test_husky_ur3.launch.py
```

A mars rover is included in the mars_thanksgiving world. Git clone https://github.com/osrf/gazebo_models/tree/master/mars_rover and copy it to models folder.

## Control the Gripper
```
- gripper open
$ ros2 topic pub -1 /rh_p12_rn_position/command std_msgs/Float64 "data: 0.0"

- gripper close
$ ros2 topic pub -1 /rh_p12_rn_position/command std_msgs/Float64 "data: 1.05"
```
