#!/bin/sh

# Execute in cnws/src
# WARNING: Will change files which are not under version control
# !!! USE AT OWN RISK !!!
find . -maxdepth 3 -name "config" -path "*.git*" -exec sed -i 's/#*\(\s*\)\(url = git@github.com:carpe-noctem-cassel.*\)/\1\2/' {} \;
find . -maxdepth 3 -name "config" -path "*.git*" -exec sed -i '/\(\s*\)\(url = cn@172.16.40.190:repos.*\)/d' {} \;

