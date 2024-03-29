#!/bin/bash

USER=USERTPL
WORKSPACE=WORKSPACETPL
WORKSPACE_SETUP=$WORKSPACE/devel/setup.bash
ROS_SETUP=/opt/ros/kinetic/setup.bash

sudo su - $USER << EOF
	source $ROS_SETUP
	source $WORKSPACE_SETUP

	export DOMAIN_FOLDER="$WORKSPACE/src/cnc-msl"
	export DOMAIN_CONFIG_FOLDER="$WORKSPACE/src/cnc-msl/etc"

	rosrun process_manager process_manager -autostart
EOF

