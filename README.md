# Robust Holistic Control-Based Trajectory Prediction in Mobile Manipulation System

## Integrated Model
- For some of the existing materials, the dimensions of the materials were different from my robot, so I designed my own


#### How to Use
```bash
mkdir -p hc_ws/src
cd hc_ws/src && git clone https://github.com/j-wye/Holistic_Control.git
cd ../ && colcon build
source install/setup.bash

# View only Manipulator
ros2 launch mobile_manipulator view.launch.py robot_type:=arm

# View only Mobile base
ros2 launch mobile_manipulator view.launch.py robot_type:=base

# View Integrate version with Mobile Base and Manipulator (default is integrate)
ros2 launch mobile_manipulator view.launch.py
```

Following image is integrate version of mobile manipulator

<img src="./mobile_manipulator/img/success_model.png" width=50% >

install this first before launch following as:
```bash
sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros2-control*
sudo apt install -y ros-${ROS_DISTRO}-topic-based-ros2-control*
sudo apt install -y ros-${ROS_DISTRO}-picknik-*
sudo apt install -y ros-${ROS_DISTRO}-diff-drive-controller*
sudo apt install -y ros-${ROS_DISTRO}-ros2-control*
```

Want to test integrate model move on a rviz following as:

succes1:
```bash
ros2 topic pub -r 10 /joint_states sensor_msgs/msg/JointState "{name: ['fl_joint', 'fr_joint', 'rl_bracket_joint', 'rl_wheel_joint', 'rr_bracket_joint', 'rr_wheel_joint', 'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6', 'right_finger_bottom_joint', 'right_finger_tip_joint', 'left_finger_bottom_joint', 'left_finger_tip_joint'], position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 2.0], velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 2.0], effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 2.0]}"
```
success2:
```bash
ros2 topic pub -r 10 /dynamic_joint_states control_msgs/msg/DynamicJointState "{joint_names: ['fl_joint', 'fr_joint', 'rl_bracket_joint', 'rl_wheel_joint', 'rr_bracket_joint', 'rr_wheel_joint', 'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6', 'right_finger_bottom_joint', 'right_finger_tip_joint', 'left_finger_bottom_joint', 'left_finger_tip_joint'], \
interface_values: \
[{interface_names: 'position', values: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 2.0]}, {interface_names: 'velocity', values: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 2.0]}]}"
```
test1:
```bash
ros2 topic pub -r 10 /dynamic_joint_states control_msgs/msg/DynamicJointState "{joint_names: ['fl_joint', 'fr_joint', 'rl_bracket_joint', 'rl_wheel_joint', 'rr_bracket_joint', 'rr_wheel_joint', 'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6', 'right_finger_bottom_joint', 'right_finger_tip_joint', 'left_finger_bottom_joint', 'left_finger_tip_joint'], \
interface_values: \
[{interface_names: ['position'], values: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 2.0]}, {interface_names: ['velocity'], values: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0, 1.0, 2.0]}]}"
```
test2:
```bash
ros2 topic pub -r 10 /dynamic_joint_states control_msgs/msg/DynamicJointState "{joint_names: ['fl_joint', 'fr_joint', 'rl_bracket_joint', 'rl_wheel_joint', 'rr_bracket_joint', 'rr_wheel_joint', 'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6', 'right_finger_bottom_joint', 'right_finger_tip_joint', 'left_finger_bottom_joint', 'left_finger_tip_joint'], \
interface_values: \
[interface_names: "fl_joint", values: [1.0, 1.0, 0.0]]}"
```