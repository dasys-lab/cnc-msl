#!/bin/bash

USER=USERTPL
WORKSPACE=WORKSPACETPL
WORKSPACE_SETUP=$WORKSPACE/devel/setup.sh
ROS_SETUP=/opt/ros/indigo/setup.sh

source $ROS_SETUP
source $WORKSPACE_SETUP

export DOMAIN_FOLDER="$WORKSPACE/src"
export DOMAIN_CONFIG_FOLDER="$WORKSPACE/src/etc"

rosrun msl_beagle_board_black_udp msl_beagle_board_black_udp
