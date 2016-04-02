#!/bin/bash
robots=( hairy nase savvy myo mops )
cmds =( 'mr up' )

for robot in "${robots[@]}"
do
	for cmd in "${cmds[@]}"
	do
		ssh cn@$i "echo \$HOSTNAME - $cmd"
        	ssh cn@$i "$cmd"
        done
done
