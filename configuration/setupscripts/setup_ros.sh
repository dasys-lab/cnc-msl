#!/bin/bash

## Wenn es einen Fehler gibt, breche das Skript ab
set -e
source ./funcs.sh

## Installiere ROS

msg "ROS Repository wird eingerichtet"

ROS_REPO_FILE="/etc/apt/sources.list.d/ros-latest.list"

add_to "deb http://packages.ros.org/ros/ubuntu trusty main" "${ROS_REPO_FILE}"

wget http://packages.ros.org/ros.key -O - | sudo apt-key add -


msg "ROS Pakete werden installiert und eingerichtet"

rospackages='ros-indigo-desktop ros-indigo-gazebo5 ros-indigo-qt-gui-core ros-indigo-qt-build python-rosinstall ros-indigo-pcl-conversions'

sudo apt-get update
eval sudo apt-get "${1}" install $rospackages

set +e
sudo rosdep init
rosdep update
rosdep fix-permissions
set -e

msg "Gazebo 2 wird deinstalliert und Gazebo 5 wird installiert"

sudo apt-get remove gazebo2*
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install ros-indigo-gazebo5-ros-pkgs ros-indigo-gazebo5-ros-control
  
msg "ROS Workspace wird angelegt und eingerichtet"

add_to "source /opt/ros/indigo/setup.bash" "~/.bashrc"

source /opt/ros/indigo/setup.bash

mkdir -p ~/cnws/src
cd ~/cnws/src
if [ ! -f ~/cnws/src/CMakeLists.txt ];
then
  catkin_init_workspace
fi

