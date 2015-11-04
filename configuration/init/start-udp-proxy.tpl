#!/bin/sh

USER=USERTPL
WORKSPACE=WORKSPACETPL
WORKSPACE_SETUP=$WORKSPACE/devel/setup.sh
ROS_SETUP=/opt/ros/indigo/setup.sh

sudo su - $USER << EOF
	source $ROS_SETUP
	source $WORKSPACE_SETUP

	export DOMAIN_FOLDER="$WORKSPACE/src/cnc-msl"
	export DOMAIN_CONFIG_FOLDER="$WORKSPACE/src/cnc-msl/etc"

	rosrun msl_udp_proxy msl_udp_proxy
EOF

