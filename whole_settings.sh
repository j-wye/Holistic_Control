pip install setuptools==58.2.0
sudo apt install ros-$ROS_DISTRO-kinematics-interface-kdl
vcs import src/controller_packages --input src/mobile_manipulator/settings.humble.repos

# For optimization build with all cores
NUM_THREADS=$(lscpu | grep '^CPU(s):' | awk '{print $2}')
colcon build --parallel-workers $NUM_THREADS --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "alias cb='colcon build --parallel-workers $NUM_THREADS --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc
source install/setup.bash