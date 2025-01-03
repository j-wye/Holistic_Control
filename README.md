# Robust Holistic Control-Based Trajectory Prediction in Mobile Manipulation System

## Integrated Model
- Manipulator Arm modeling was existied but mobile base wasn't, so I measured and designed by own

#### How to Use :
```bash
mkdir -p hc_ws/src && cd hc_ws
git clone https://github.com/j-wye/Holistic_Control.git src/Holistic_Control
mv src/Holistic_Control/* src/ && rm -rf src/Holistic_Control*
colcon build && source install/setup.bash
# vcs import src/controller_packages --input src/mobile_manipulator/settings.humble.repos
# wget https://raw.githubusercontent.com/j-wye/Holistic_Control/main/whole_settings.sh
```
#### How to execute :
```bash
# View only Manipulator
ros2 launch mobile_manipulator view.launch.py robot_type:=arm

# View only Mobile base
ros2 launch mobile_manipulator view.launch.py robot_type:=base

# View Integrate version with Mobile Base and Manipulator (default is integrate)
ros2 launch mobile_manipulator view.launch.py

# Launch with Gazebo and Rviz
ros2 launch mobile_manipulator gazebo.launch.py rviz:=true
```

Following image is integrate version of mobile manipulator

<img src="./mobile_manipulator/img/success_model.png" width=50%>
<img src="./mobile_manipulator/img/urdf_structure.png" >

<!-- install this first before launch following as:
```bash
sudo apt install -y ros-${ROS_DISTRO}-gazebo-ros2-control*
sudo apt install -y ros-${ROS_DISTRO}-ros2-control*
sudo apt install -y ros-${ROS_DISTRO}-topic-based-ros2-control*
sudo apt install -y ros-${ROS_DISTRO}-picknik-*
sudo apt install -y ros-${ROS_DISTRO}-diff-drive-controller*
``` -->

Want to test integrate model move on a rviz following as:

1. Move Manipulator Arm Joints with command

  - Use Joint Trajectory Controller (Means position control, not a velocity control)
    - ros2 topic pub:
      ```bash
      ros2 topic pub --once --qos-reliability best_effort /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
      "{\
      joint_names: ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6', 'right_finger_bottom_joint'],\
      points: [{\
        positions: [0.0, -1.0, 1.0, 0.5, 0.0, 0.0, 0.8],\
        time_from_start: {sec: 5, nanosec: 0}\
        }]\
      }"
      ```
    - ros2 action send_goal:
      ```bash
      ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
      "{\
        trajectory: {\
          header: {\
            stamp: {\
              sec: 0,\
              nanosec: 0\
            },\
            frame_id: ''\
          },\
          joint_names: ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6'],\
          points: [\
            {\
              positions: [0.0, -1.0, 1.0, 0.5, 0.0, 0.0],\
              velocities: [0.0, 0.5, 0.5, 0.5, 0.0, 0.0],\
              time_from_start: {\
                sec: 1.0,\
                nanosec: 0.0\
              }\
            }\
          ]\
        },\
        path_tolerance: [],\
        goal_tolerance: [],\
        goal_time_tolerance: {\
          sec: 1,\
          nanosec: 0\
        }\
      }"
      ```
2. Move Gripper with command:
  - ros2 action send_goal:
    ```bash
    ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
    "{\
      command: {\
        position: 0.8,\
        max_effort: 10.0\
      }\
    }"
    ```
3. Mobile Base Move with command:
  - ros2 topic pub:
    ```bash
    ros2 topic pub -r 100 /diff_drive_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "\
    {\
      linear: {x: 0.0, y: 0.0, z: 0.0}, \
      angular: {x: 0.0, y: 0.0, z: -1.0}\
    }"
    ```
  
  - ros2 run teleop_twist_keyboard :
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=base_controller/cmd_vel_unstamped
    ```
  
  - asd: