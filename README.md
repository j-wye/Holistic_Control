# Robust Holistic Control-Based Trajectory Prediction in Mobile Manipulation System

## Integrated Model
- For some of the existing materials, the dimensions of the materials were different from my robot, so I designed my own

```bash
# Only view Manipulator
ros2 launch mobile_manipulator arm_and_gripper.launch.py

# Only view Mobile base
ros2 launch mobile_manipulator base.launch.py

# Integrated version with Mobile Base and Manipulator
ros2 launch mobile_manipulator view_robot.launch.py
```