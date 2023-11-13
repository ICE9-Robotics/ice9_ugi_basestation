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
echo "source ${dir}/basestation_ws/devel/setup.bash" >> ${HOME}/.bashrc

# Configure mapviz
cp ${dir}/basestation_ws/src/ice9_unitree_basestation/config/unitree.mvc ${dir}/basestation_ws/src/ice9_unitree_basestation/config/unitree.local.mvc
sed -i "s|UTGO1PRO.png|${dir}/basestation_ws/src/ice9_unitree_basestation/config/unitree.png|" ${dir}/basestation_ws/src/ice9_unitree_basestation/config/unitree.local.mvc

# Desktop launcher
echo "[Desktop Entry]
Name=Unitree Basestation
Comment=Launch the base station for Unitree Go1
Exec=${dir}/install/files/basestation.desktop.sh
Icon=${dir}/install/files/ice9_logo.png
Terminal=true
Type=Application" > Unitree_Basestation.desktop
chmod +x Unitree_Basestation.desktop
gio set Unitree_Basestation.desktop "metadata::trusted" true
mv Unitree_Basestation.desktop ~/Desktop/Unitree_Basestation.desktop

mkdir -p ${HOME}/.config/ice9_basestation
echo "export BASESTATION_DIR=${dir}" > ${HOME}/.config/ice9_basestation/setup.bash
