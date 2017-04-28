#!/bin/bash
if [[ $(cat ~/.mrconfig | grep "git branch" | wc -l) = 0 ]]; then
	sed -i '1i[DEFAULT]\nbranch = git branch\n' ~/.mrconfig
	echo "done!"
else
	echo "nothing to do here"
fi
