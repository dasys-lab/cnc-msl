#!/bin/bash -i


echo "open consoles"
gnome-terminal --tab -e "bash -i -c \" roscore; set-title h; exec bash\"" --tab -e "bash -i -c \" ROBOT=nase rosrun process_manager process_manager; exec bash\"" --tab -e "bash -i -c \" rosrun robot_control robot_control; exec bash\"" --tab -e "bash -i -c \" echo ROBOT=nase rosrun msl_base msl_base -m WM16 -sim; exec bash\"" 

