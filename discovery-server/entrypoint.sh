#!/bin/bash
set -e

# Source ROS 2 Jazzy setup
source /opt/ros/jazzy/setup.bash

exec "$@"
