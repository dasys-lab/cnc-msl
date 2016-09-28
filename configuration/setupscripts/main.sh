#!/bin/bash
set -e

source "./funcs.sh"

if [[ $EUID -eq 0 ]]; then
   echo "This script must NOT be run as root!" 1>&2
   exit 1
fi

if askSure "Installation von Paketen ohne Nachfrage?"
then
	apt_options='-y'
fi

./install_packages.sh "${apt_options}"
./setup_ros.sh "${apt_options}"
./download_repos.sh
./setup_env.sh
