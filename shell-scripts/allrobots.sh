#!/bin/bash
robots=(mops myo nase savvy hairy)

# help?
if [[ $1 == "--help" ]]; then
	echo "Launches separate terminals for each robot, connects via SSH and lets you execute commands simultanously."
	echo "Usage: allrobots.sh [option]"
	echo ""
	echo "Options:"
	echo "--help	prints this text"
	echo "--clear	kills all screens"
	echo "--setup	installs screen, copies the robot's SSH ids and adds an alias for running the script to your .bashrc"
	exit
fi

# check for online robots
onlineRobots=()
for robot in  "${robots[@]}"
do
	if [[ $(ping -c 1 -W 1 "$robot" | grep "1 received" | wc -l) = 1 ]]; then
		onlineRobots+=("$robot")
	else
		echo "notice: $robot is offline"
	fi
done

if [[ ${#onlineRobots[@]} = 0 ]]; then
	echo "all robots offline :("
	exit
fi


# launch parameter set?
if [[ $1 == "--setup" ]]; then
	sudo apt-get -y --force-yes install screen
	if [[ $(cat ~/.bashrc | grep allrobots | wc -l) = 0 ]]; then
		echo "alias allrobots='~/cnws/src/cnc-msl/shell-scripts/allrobots.sh'" >> ~/.bashrc
	fi
	for robot in "${onlineRobots[@]}"
        do
                gnome-terminal --title "$robot" -e "bash -c \"ssh-copy-id $robot\""
        done
	exit
fi

if [[ $1 == "--clear" ]]; then
	killall screen
	exit
fi

# parameter not set - usual launch

# start screens for each robot
for robot in "${onlineRobots[@]}"
do
	if [[ $(screen -list | grep "$robot" | wc -l) = 0 ]]; then
		gnome-terminal --title "$robot" -e "bash -c \"screen -S $robot\""
	else
		echo "notice: $robot screen already running"
	fi
done

# wait for screens to start and print connecting message
for robot in "${onlineRobots[@]}"
do
	while [[ $(screen -list | grep "$robot" | wc -l) = 0 ]]
	do
		:
	done

	screen -S "$robot" -X stuff "##########################^M# connecting to $robot...^M##########################^M^M"
	screen -S "$robot" -X stuff "if [[ \"\\\$HOSTNAME\" != \"$robot\" ]]; then ssh cn@$robot; fi^M"
	screen -S "$robot" -X stuff "if [[ \"\\\$HOSTNAME\" == \"$HOSTNAME\" ]]; then exit; fi^M"
	screen -S "$robot" -X stuff "clear^M"
done

# command input loop
echo ""
echo "Every command you enter will be executed on ALL remote shells. "
echo "Type exit to close all terminals."
cmd="hello"
while [ "$cmd" != "exit" ]
do
	read -e -p ">  " cmd
	for robot in "${onlineRobots[@]}"
	do
		screen -S "$robot" -X stuff "$cmd^M"
	done
done

# exit screens
if [[ "$cmd" == "exit" ]]; then
	killall screen
fi
