#!/bin/bash
# this script provides fast switching between wlan profiles including ad-hoc mode. you have to use a seperate file per profile that is included in /etc/network/interfaces via source <file>. there are several files prepared in the subdirectory networkconfig.
interfaces="/etc/network/interfaces"


if [[ $(cat "$interfaces" | grep "source configs/$1" | wc -l) = 0 ]]; then
	echo "Config $1 unknown!"
	exit 1
fi

if [ "$(id -u)" != "0" ]; then
	echo "Must be run as root"
	exit 1
fi

rebootnow=nope
if [[ $(cat /etc/network/interfaces | grep -E '^source configs/adhoc' | wc -l) > 0 ]]; then
	echo "Caution: I am currently in adhoc mode. Leaving this mode requires a reboot."
	echo -n "Change mode and reboot now? [Y/n] "
	read rebootnow
	if [[ $rebootnow != "y" && $rebootnow != "" ]]; then
		exit 1
	fi
fi

# disable all configurations
sed -i 's/^source/#source/g' "$interfaces"

# activate the chosen configuration
sed -i "s!#source configs/$1!source configs/$1!" "$interfaces"

if [[ "$1" == "adhoc" ]]; then
	sed -i "s/wireless-essid Adhoc/wireless-essid $HOSTNAME/" /etc/network/configs/adhoc
fi

ifdown wlan0
ifup wlan0

if [[ $rebootnow == "y" || $rebootnow == "" ]]; then
	reboot
fi

exit

# legacy, not used
# determine current state of the configuration
if [[ $(cat "$interfaces" | grep "#source adhoc" | wc -l) = 0 ]]; then
	hasAdhoc=true
	isActive=false
else
	if [[ $(cat "$interfaces" | grep "source adhoc" | wc -l) = 0 ]]; then
		hasAdhoc=true
		isActive=true
	else
		hasAdhoc=false
		isActive=false
	fi
fi

echo "hasadhoc=$hasAdhoc, isactive=$isActive"
echo "$1"
