#!/bin/bash
set -e

# setup ros2 environment
source "$ROSDEV_DIR/.ros/amd64/$ROS_DISTRO/setup.bash" 2> /dev/null || source "/opt/ros/$ROS_DISTRO/setup.bash"
exec "$@"
