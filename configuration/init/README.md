# Carpe-Noctem autostart scripts for Ubuntu

This folder contains scripts to start processes like the roscore and msl_udp proxy
at boot time using ubuntus upstart.

For more information about upstart take a look at the [Upstart Cookbook](http://upstart.ubuntu.com/cookbook).

First you have to make sure that the varialbes WORKSPACE_USER and WORKSPACE_PATH are set
correctly in the Makefile.
To install the scripts after that, simply run `sudo make install`.

After the next reboot the process manager should be started automatically.
