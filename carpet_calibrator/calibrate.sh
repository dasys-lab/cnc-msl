#!/bin/bash
roslaunch carpet_calibrator start.launch

echo "Is there an *.raw file in $DOMAIN_CONFIG_FOLDER/$HOSTNAME?"
read -r -p "[y/n]" response
response=${response,,} # tolower

if [[ $response =~ ^(yes|y| ) ]]; then
	echo "open image..."
	eog $DOMAIN_CONFIG_FOLDER/$HOSTNAME/CarpetCalibImage.png&

	echo "Does this image look good?"
	read -r -p "[y/n]" response
	response=${response,,} # tolower
	if [[ $response =~ ^(yes|y| ) ]]; then
		killall eog
		rosrun carpet_calibrator carpet_calibrator_line_points
		rosrun msl_vision msl_carpet_calculator
	fi 
fi
