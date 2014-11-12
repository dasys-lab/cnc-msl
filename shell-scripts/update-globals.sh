#!/bin/bash
cd $ES_ROOT
echo "# git fetch"
git fetch
if [ "echo $?" ]; then
 echo "# git checkout origin/master -- etc/Globals.conf"
 git checkout origin/master -- etc/Globals.conf
 echo "# git pull"
 git pull
else 
 echo "Git Fetch Failure"
fi
echo "# git pull"
git pull
