#!/bin/bash
while [ 1 ]
do
	if [ `iwconfig wlan0 | grep "not associated"`]; then
		ifdown wlan0
		ifup wlan0
		sleep 5
	fi
	sleep 1
done
