#!/bin/bash
set -e

./install_packages.sh
./setup_ros.sh
./download_repos.sh
./install_vartypes.sh
./setup_env.sh
