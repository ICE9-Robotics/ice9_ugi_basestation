#!/bin/bash
cd $(dirname "$(readlink -f "$0")")/..
dir=$(pwd)

# Pull submodules
git submodule update --init --recursive --force
cd basestation_ws/src/unitree_ros_to_real
git fetch --tag
git checkout tags/v3.8.0
rm -rf unitree_legged_real

# Build ros packages
cd ../..
rosdep install --from-paths src --ignore-src -r -y
catkin_make
echo "source ${dir}/basestation_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Configure mapviz
sed -i "s|UTGO1PRO.png|${dir}/basestation_ws/src/ice9_unitree_basestation/config/unitree.png|" ${dir}/basestation_ws/src/ice9_unitree_basestation/config/unitree.mvc