#!/bin/bash
set -e

# Source shared environment setup
source /opt/ros/ros2_env.sh

# Use NODE_NAME for unique identification
NODE_ID="${NODE_NAME:-$(hostname)}"

echo "------------------------------------------------"
echo "  ROS 2 CLUSTER CLIENT"
echo "  Connecting to Brain at: ${ROS_DISCOVERY_SERVER:-NOT SET}"
echo "  Node ID: ${NODE_ID}"
echo "  Node Role: ${NODE_ROLE:-idle}"
echo "------------------------------------------------"

# Determine what to run based on NODE_ROLE
# Each node gets a unique name: {device_name}_{role}
case "${NODE_ROLE:-idle}" in
    talker)
        echo "Starting talker node as ${NODE_ID}_talker..."
        exec ros2 run demo_nodes_cpp talker --ros-args --remap __node:=${NODE_ID}_talker
        ;;
    listener)
        echo "Starting listener node as ${NODE_ID}_listener..."
        exec ros2 run demo_nodes_cpp listener --ros-args --remap __node:=${NODE_ID}_listener
        ;;
    both)
        echo "Starting talker and listener..."
        ros2 run demo_nodes_cpp talker --ros-args --remap __node:=${NODE_ID}_talker &
        exec ros2 run demo_nodes_cpp listener --ros-args --remap __node:=${NODE_ID}_listener
        ;;
    idle|*)
        echo "Idle mode - use 'balena ssh <uuid> ros-node' to run commands"
        exec sleep infinity
        ;;
esac
