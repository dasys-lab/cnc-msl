#!/bin/bash

echo "Did you check the gain and calibrate the center of the mirror?"
read -r -p "[Y/n]" response
response=${response,,}

if [[ $response =~ ^(yes|y|"") ]]; then
	
	roslaunch carpet_calibrator start.launch&
	sleep 5

	read -r -p "please press start in the basestation confirm with ENTER!"
	
	echo "Is there an *.raw file in $DOMAIN_CONFIG_FOLDER/$HOSTNAME?"
	read -r -p "[Y/n]" response
	response=${response,,}
	
	if [[ $response =~ ^(yes|y|"") ]]; then
		echo "open image..."
		eog $DOMAIN_CONFIG_FOLDER/$HOSTNAME/CarpetCalibImage.png&
	
		echo "Does this image look good?"
		read -r -p "[Y/n]" response
		response=${response,,} # tolower
		if [[ $response =~ ^(yes|y|"") ]]; then
			killall eog
			rosrun carpet_calibrator carpet_calibrator_line_points
			rosrun msl_vision msl_carpet_calculator
		else
			echo "please try"
		fi 
		
	else
		echo "please try again!"
	fi

fi
