#!/bin/bash
set -e

if [[ $EUID -eq 0 ]]; then
   echo "This script must NOT be run as root!" 1>&2
   exit 1
fi

./install_packages.sh
./setup_ros.sh
./download_repos.sh
./install_vartypes.sh
./setup_env.sh
