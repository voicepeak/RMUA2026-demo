#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /opt/rmua_msgs/setup.bash
set -u
# ROS_IP and ROS_MASTER_URI are supplied by the official launcher, never set here.
exec "$@"
