pip install setuptools==58.2.0
sudo apt install ros-$ROS_DISTRO-kinematics-interface-kdl
# vcs import src/controller_packages --input src/mobile_manipulator/settings.humble.repos
sudo apt install ros-humble-openni2-* -y


# For optimization build with all cores
NUM_THREADS=$(lscpu | grep '^CPU(s):' | awk '{print $2}')
echo "alias cb='colcon build --parallel-workers $NUM_THREADS --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc
source ~/.bashrc && cb
source install/setup.bash

# Install RTAB-Map
cd ~/hc_ws/src && mkdir RTAB_Map && cd RTAB_Map
git clone https://github.com/introlab/rtabmap.git -b humble-devel
git clone https://github.com/introlab/rtabmap_ros.git -b ros2
cd ~/hc_ws
rosdep update && rosdep install --from-paths src --ignore-src -r -y
sb

echo "export RCUTILS_LOGGING_USE_STDOUT=1" >> ~/.bashrc
echo "export RCUTILS_LOGGING_BUFFERED_STREAM=1" >> ~/.bashrc
# Optional, but if you like colored logs:
echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
# Recommend to use cyclonedds than fastrtps
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp