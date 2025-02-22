pip install setuptools==58.2.0
sudo apt install ros-$ROS_DISTRO-kinematics-interface-kdl
sudo apt install 
# Before colcon build, install the following dependencies:
sudo apt install ros-$ROS_DISTRO-robot-* ros-$ROS_DISTRO-libpointmatcher* libeigen3-dev -y

# First install Mobile Manipulator packages
cd && mkdir -p hc_ws/src && cd hc_ws
git clone https://github.com/j-wye/Holistic_Control.git src/Holistic_Control
mv src/Holistic_Control/* src/ && rm -rf src/Holistic_Control*

# Install RTAB-Map
# If you want to use multi-camera, before installing RTAB-Map, you need to install the following dependencies (opengv):
cd && git clone https://github.com/laurentkneip/opengv.git
cd opengv
git checkout 91f4b19c73450833a40e463ad3648aae80b3a7f3
wget https://gist.githubusercontent.com/matlabbe/a412cf7c4627253874f81a00745a7fbb/raw/accc3acf465d1ffd0304a46b17741f62d4d354ef/opengv_disable_march_native.patch
git apply opengv_disable_march_native.patch
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc) && sudo make install && cd
sudo rm -rf opengv

# Now install RTAB-Map and RTAB-Map ROS with dependencies on ROS 2 Humble:
cd ~/hc_ws/src && mkdir RTAB_Map && cd RTAB_Map
mkdir rtabmap_db
git clone https://github.com/introlab/rtabmap.git -b humble-devel
cd rtabmap/build
cmake -DWITH_OPENGV=ON ..
make -j$(nproc) && sudo make install
cd ../..
git clone https://github.com/introlab/rtabmap_ros.git -b humble-devel
cd ~/hc_ws
rosdep update && rosdep install --from-paths src --ignore-src -r -y

echo "export RCUTILS_LOGGING_USE_STDOUT=1" >> ~/.bashrc
echo "export RCUTILS_LOGGING_BUFFERED_STREAM=1" >> ~/.bashrc
# Optional, but if you like colored logs:
echo "export RCUTILS_COLORIZED_OUTPUT=1" >> ~/.bashrc
# Recommend to use cyclonedds than fastrtps
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source ~/.bashrc


# Now build the workspace
NUM_THREADS=$(lscpu | grep '^CPU(s):' | awk '{print $2}')
colcon build --symlink-install --parallel-workers $NUM_THREADS --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON -DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
source ~/.bashrc