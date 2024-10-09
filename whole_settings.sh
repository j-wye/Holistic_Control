mkdir -p hc_ws/src
cd hc_ws
git clone https://github.com/j-wye/Holistic_Control.git src/Holistic_Control
mv src/Holistic_Control/* src/
rm -rf src/Holistic_Control*
vcs import src/controller_packages --input src/mobile_manipulator/settings.humble.repos
pip install setuptools==58.0.4
sudo apt install ros-$ROS_DISTRO-kinematics-interface-kdl

# For optimization build with all cores
NUM_THREADS=$(lscpu | grep '^CPU(s):' | awk '{print $2}')
colcon build --parallel-workers $NUM_THREADS --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "alias cb='colcon build --parallel-workers $NUM_THREADS --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc
source install/setup.bash