#!/bin/bash
set -e

# Source ROS 2 Jazzy setup
source /opt/ros/jazzy/setup.bash

export PATH=$PATH:/opt/ros/jazzy/bin

exec "$@"
