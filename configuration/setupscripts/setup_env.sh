#!/bin/bash

set -e
source ./funcs.sh

## Add .bashrc source command for workspace

msg ".bashrc Einträge werden gemacht"

add_to_bashrc "source ~/cnws/devel/setup.bash"

# Add configs for ALICA bash prompt

add_to_bashrc "# The next 2 lines determine the application domain for ALICA (Team-Modelling software)"
add_to_bashrc "export DOMAIN_FOLDER=\"$HOME/cnws/src/cnc-msl\""
add_to_bashrc "export DOMAIN_CONFIG_FOLDER=\"$HOME/cnws/src/cnc-msl/etc\""

# Add fancy bash prompt

add_to_bashrc "#fancy prompt that also shows the current branch"
add_to_bashrc "export PS1='\[\033[01;32m\]\u@\h\[\033[01;34m\] \w \[\033[01;31m\]\$(__git_ps1 \"[%s]\")\[\033[01;34m\]\$\[\033[00m\] '"

source ~/.bashrc
