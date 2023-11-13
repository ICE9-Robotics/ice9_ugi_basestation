#!/bin/bash

# Ubuntu 18.04
# This script prepares a fresh Ubuntu installation for general ROS development

# Define software switches
utilities_enable=true
ros_enable=true
husarnet_enable=true

distrib_release=`cat /etc/lsb-release | grep DISTRIB_RELEASE | cut -d'=' -f2`
if [[ ${distrib_release} != "18.04" ]]; then
	echo "Ubuntu distribution release does not match. Expect 18.04, got ${distrib_release}!" 
	return
fi

sudo apt-get update
sudo apt-get upgrade -y

# Utility software
if [ ${utilities_enable} ]; then
	# Terminator, Simple screen recorder
	sudo apt-get install terminator simplescreenrecorder -y

	# VS code
	if [ -f /etc/apt/sources.list.d/vscode.list ]; then
		echo "Visual Studio code has already been installed. Skipping ..."
	else
		curl -s https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
		sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
		sudo apt install code -y
	fi
fi

# ROS
if [ ${ros_enable} ]; then
	if [ -f /etc/apt/sources.list.d/ros-latest.list ]; then
		echo "ROS has already been installed. Skipping ..."
	else
		sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
		sudo apt-get install curl -y # if you haven't already installed curl
		curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
		sudo apt-get update
		sudo apt-get install ros-melodic-desktop-full -y
		echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
		source ~/.bashrc
		sudo apt-get install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential -y
		sudo apt-get install python-rosdep -y
		sudo rosdep init --include-eol-distros
		rosdep update
	fi
fi

# Husarnet
if [ ${husarnet_enable} ]; then
	if [ -f /etc/apt/sources.list.d/husarnet.list ]; then
	    echo "Husarnet already installed."
	else
	    sudo -A apt-get install ca-certificates -y
	    curl https://install.husarnet.com/install.sh | sudo -A bash
		sudo husarnet daemon service-install
	fi
fi
