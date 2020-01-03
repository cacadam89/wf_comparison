#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

# start avahi daemon
/etc/init.d/dbus start &>/dev/null
service avahi-daemon start &>/dev/null

# source the cartographer-specific env
source ~/wf_comparison_ws/install_isolated/setup.bash

exec "$@"
