#!/bin/bash
source ${HOME}/.config/ice9_basestation/setup.bash

# Husarnet
local_hostname=`husarnet status | grep localhost | awk '{print $3}'`
if [ ! ${local_hostname} ]; then
    echo "Can't get Husarnet status. Please check if the software has been properly installed."
    read -p "Press Enter to continue ..."
    exit 1
fi

# ROS setup
export ROS_IPV6=on
export ROS_MASTER_URI=http://master:11311
export ROS_HOSTNAME=${local_hostname}
source ${BASESTATION_DIR}/basestation_ws/devel/setup.bash

cd ${BASESTATION_DIR}/launcher
python launcher.py