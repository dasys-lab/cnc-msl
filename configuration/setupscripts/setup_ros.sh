#!/bin/bash

## Wenn es einen Fehler gibt, breche das Skript ab
set -e
source ./funcs.sh

## Installiere ROS

msg "ROS Repository wird eingerichtet"

ROS_REPO_FILE="/etc/apt/sources.list.d/ros-latest.list"

if [ -f $ROS_REPO_FILE ]; 
then
# FIXME: dont know how to use $ROS_REPO_FILE in the sudo sh -c command
  sudo grep -q -F "deb http://packages.ros.org/ros/ubuntu trusty main" $ROS_REPO_FILE || sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" >> /etc/apt/sources.list.d/ros-latest.list'
else
 sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" >> /etc/apt/sources.list.d/ros-latest.list'
fi

wget http://packages.ros.org/ros.key -O - | sudo apt-key add -


msg "ROS Pakete werden installiert und eingerichtet"

sudo apt-get update
sudo apt-get -y install ros-indigo-desktop-full ros-indigo-qt-gui-core ros-indigo-qt-build python-rosinstall

set +e
sudo rosdep init
rosdep update
rosdep fix-permissions
set -e

msg "ROS Workspace wird angelegt und eingerichtet"

add_to_bashrc "source /opt/ros/indigo/setup.bash"

. ~/.bashrc

mkdir -p ~/cnws/src
cd ~/cnws/src
if [ ! -f ~/cnws/src/CMakeLists.txt ];
then
  catkin_init_workspace
fi
