#!/bin/bash
declare -a robots=("mops" "myo" "nase" "savvy" "hairy")

# setup?
if [[ $1 == "setup" ]]; then
	sudo apt-get -y install screen
	for robot in "${robots[@]}"
        do
                gnome-terminal --title "$robot" -e "bash -c \"ssh-copy-id $robot\""
        done
	exit
fi

# setup parameter not set - usual launch

# start screens for each robot
for robot in "${robots[@]}"
do
	gnome-terminal --title "$robot" -e "bash -c \"screen -S $robot\""
done

# wait for screens to start and print connecting message
for robot in "${robots[@]}"
do
	while [[ $(screen -list | grep "$robot" | wc -l) = 0 ]]
	do
		:
	done

	screen -S "$robot" -X stuff "# connecting to $robot...^M"
	screen -S "$robot" -X stuff "ssh cn@$robot^M"
done

# command input loop
echo "hit enter on a blank line to exit"
cmd="hello"
while [ "$cmd" != "" ]
do
	read -e -p "execute on all robots: " cmd
	for robot in "${robots[@]}"
	do
		screen -S "$robot" -X stuff "$cmd^M"
	done
done

# exit screens
for robot in "${robots[@]}"
do
        screen -S "$robot" -X stuff "exit^M;exit^M"
done



