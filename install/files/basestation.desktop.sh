#!/bin/bash
cd $(dirname "$(readlink -f "$0")")/../..
dir=$(pwd)

# ROS setup
export ROS_IPV6=on
export ROS_MASTER_URI=http://master:11311
export ROS_HOSTNAME=basestation_0
source ${dir}/basestation_ws/devel/setup.bash

# Wait for ros master
while [[ ! $(rostopic list) ]]; do
	sleep 1
    echo "Waiting for ros master ..."
done

# Launch
roslaunch ice9_unitree_basestation basestation.launch
read "Press Enter to continue ..."
