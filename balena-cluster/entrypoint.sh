#!/bin/bash
set -e

# Source shared environment setup
source /opt/ros/ros2_env.sh

# Get unique node ID from Balena device name (auto-injected by Balena)
# Fall back to hostname if not in Balena
NODE_ID="${BALENA_DEVICE_NAME_AT_INIT:-${HOSTNAME:-node}}"

# Sanitize node ID - ROS2 node names can only have alphanumeric and underscores
NODE_ID=$(echo "$NODE_ID" | tr -cd '[:alnum:]_')

echo "------------------------------------------------"
echo "  ROS 2 CLUSTER CLIENT"
echo "  Connecting to Brain at: ${ROS_DISCOVERY_SERVER:-NOT SET}"
echo "  Node ID: ${NODE_ID}"
echo "  Node Role: ${NODE_ROLE:-idle}"
echo "------------------------------------------------"

# Determine what to run based on NODE_ROLE
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
