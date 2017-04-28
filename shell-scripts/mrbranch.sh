#!/bin/bash
if [[ $(cat ~/.mrconfig | grep "git branch" | wc -l) = 0 ]]; then
	echo "" >> ~/.mrconfig
	echo "[DEFAULT]" >> ~/.mrconfig
	echo "branch = git branch" >> ~/.mrconfig

	echo "done!"
else
	echo "nothing to do here"
fi
