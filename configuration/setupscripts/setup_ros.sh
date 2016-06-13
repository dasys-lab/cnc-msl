#!/bin/bash

## Wenn es einen Fehler gibt, breche das Skript ab
set -e
source ./funcs.sh

## Installiere ROS

msg "ROS Repository wird eingerichtet"

# Distributionsversion auslesen
codename=`lsb_release -cs`

# Linux Mint korrektur
case "${codename}" in
    qiana | rebecca | rafaela | rosa)
        codename=trusty
        ;;
    sarha)
        codename=xenial
        ;;
esac
    


add_to "deb http://packages.ros.org/ros/ubuntu ${codename} main" "/etc/apt/sources.list.d/ros-latest.list" "as_root"
add_to "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable ${codename} main" "/etc/apt/sources.list.d/gazebo-stable.list" "as_root"

wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -


msg "ROS Pakete werden installiert und eingerichtet"

rospackages='ros-indigo-desktop ros-indigo-gazebo5-ros-pkgs ros-indigo-qt-gui-core ros-indigo-qt-build python-rosinstall ros-indigo-pcl-conversions ros-indigo-mrpt-map'


sudo apt-get update
if [ -z "$1" ]
then 
   eval sudo apt-get install $rospackages
else
   eval sudo apt-get "${1}" install $rospackages
fi

set +e
sudo rosdep init
rosdep update
rosdep fix-permissions
set -e

msg "ROS Workspace wird angelegt und eingerichtet"

add_to "source /opt/ros/indigo/setup.bash" ~/.bashrc

source /opt/ros/indigo/setup.bash

mkdir -p ~/cnws/src
cd ~/cnws/src
if [ ! -f ~/cnws/src/CMakeLists.txt ];
then
  catkin_init_workspace
fi

