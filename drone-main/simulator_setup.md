# Developer Setup

## Linux Setup

1. Install gazebo.
2. Install qgroundcontrol.
3. Install mavsdk.
4. Clone PX4-Autopilot source code.
5. Clone this repo.

## Building Gazebo and QGroundControl from source:

For officially supported linux distributions you should not need to install gazebo or qgroundcontrol
from source. Just install the package from the repo or follow the official instructions.

If you are building gazebo from source, you may need to install specific versions of libraries. For
archlinux you need to install ignition-msg6 from source. ignition-msg6 is not in the aur currently,
but it's easy to build and install from source. (Making the aur package is on my todo list.)

## Gazebo Simulator Setup

Build and install the gazebo dependencies not in the aur (ignition-msg6). Then install gazebo-git.

	make ignition-msg6
	PX4-Autopilot/Tools/gazebo_sitl_multiple_run.sh -s "iris:3,plane:2"
