cnc-setup
=========

> scripts to install [ROS](http://www.ros.org/) Kinetic and domain specific
> repositories of [carpe-noctem-cassel](https://github.com/carpe-noctem-cassel/cnc-msl)

Scripts
-------

### msl-setup.sh

This script installs ROS Kinetic on Ubuntu 16.04 and clones all repositories needed
for [MSL](https://en.wikipedia.org/wiki/RoboCup_Middle_Size_League).

Usage:
 
	$ sudo ./msl-setup.sh

### ttb-setup.sh

Just like the script above, this script installs ROS Kinetic.
Instead of cloning all repositories for MSL it clones all repositories
to work with the [TurtleBot 2](http://www.turtlebot.com/).

Usage:
 
	$ sudo ./ttb-setup.sh

Notes/Issues
------------

It is highly recommended to update your system before running the script
by `sudo apt-get update && sudo apt-get upgrade`.

The script won't build the workspace because you might want to call catkin
with parameters for your ide(e.g. eclipse).

License
-------

ROS Kinetic and other software products used and listed in the script have
their respective licenses.

The scripts and this note itself is licensed under the MIT License:

```
MIT License

Copyright (c) 2016 Phileas Vöcking

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```