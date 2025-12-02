#!/bin/bash
set -e

# Source shared environment setup
source /opt/ros/ros2_env.sh

echo "------------------------------------------------"
echo "  ROS 2 CLUSTER CLIENT"
echo "  Connecting to Brain at: ${ROS_DISCOVERY_SERVER:-NOT SET}"
echo "  Namespace: ${ROS_NAMESPACE:-/}"
echo "------------------------------------------------"

if [ $# -eq 0 ]; then
    exec sleep infinity
fi

exec "$@"
