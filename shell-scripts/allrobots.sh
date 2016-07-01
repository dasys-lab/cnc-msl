#!/bin/bash
robots=(mops myo nase savvy hairy)

title="allrobots"
echo -e '\033]2;'allrobots'\007'

# help?
if [[ $1 == "--help" ]]; then
	echo "Launches separate terminals for each robot, connects via SSH and lets you execute commands simultanously."
	echo "Usage: allrobots.sh [option]"
	echo ""
	echo "Options:"
	echo "--help      prints this text"
	echo "--pullmake  invokes 'mr up' and 'catkin_make' on all remote shells"
	echo "--clear     kills all screens"
	echo "--setup     installs screen, copies the robot's SSH ids and adds an alias for running the script to your .bashrc"
	exit
fi

pullmake="cd ~/cnws;if [[ \$(mr up | tee /dev/tty | grep failed | wc -l) = 0 ]]; then catkin_make; fi"

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
	sudo apt-get -y --force-yes install screen wmctrl
	if [[ $(cat ~/.bashrc | grep allrobots | wc -l) = 0 ]]; then
		echo "alias allrobots='~/cnws/src/cnc-msl/shell-scripts/allrobots.sh'" >> ~/.bashrc
	fi
	for robot in "${onlineRobots[@]}"
        do
                gnome-terminal --title "$robot" -e "bash -c \"ssh-copy-id cn@$robot\""
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

# give focus back to the allrobots terminal
wmctrl -a allrobots

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

	if [[ $1 == "--pullmake" ]]; then
		screen -S "$robot" -X stuff "cd cnws^M"
		screen -S "$robot" -X stuff "$pullmake^M"
	fi
done

# command input loop
echo ""
echo "Every command you enter will be executed on ALL remote shells. "
echo "Ctrl+D closes all shells, Ctrl+C leaves them open and only quits this script."
returnval=0
while [ "$returnval" -eq 0 ]
do
	read -e -p ">  " cmd
	returnval=$?
	
	if [[ "$cmd" == "pullmake" ]]; then
		cmd="$pullmake"
	fi
	
	for robot in "${onlineRobots[@]}"
	do
		screen -S "$robot" -X stuff "$cmd^M"
	done
done

# exit screens
if [[ "$cmd" == "exit" || "$returnval" -eq 1 ]]; then
	killall screen
fi


echo -e '\033]2;'Terminal'\007'
