#!/bin/bash
set -e

# Source shared environment setup
source /opt/ros/ros2_env.sh

# Get unique node ID from Balena device name (auto-injected by Balena)
# Fall back to hostname if not in Balena
NODE_ID="${BALENA_DEVICE_NAME_AT_INIT:-${HOSTNAME:-node}}"

# Sanitize node ID - ROS2 node names can only have alphanumeric and underscores
NODE_ID=$(echo "$NODE_ID" | tr -cd '[:alnum:]_')
export NODE_ID

echo "------------------------------------------------"
echo "  ROS 2 CLUSTER CLIENT"
echo "  Connecting to Brain at: ${ROS_DISCOVERY_SERVER:-NOT SET}"
echo "  Node ID: ${NODE_ID}"
echo "  Node Role: ${NODE_ROLE:-idle}"
echo "------------------------------------------------"

# Determine what to run based on NODE_ROLE
case "${NODE_ROLE:-idle}" in
    talker)
        echo "Starting cluster talker..."
        exec python3 /opt/ros/cluster_talker.py
        ;;
    listener)
        echo "Starting cluster listener..."
        exec python3 /opt/ros/cluster_listener.py
        ;;
    status)
        echo "Starting cluster status monitor..."
        exec python3 /opt/ros/cluster_status.py
        ;;
    both)
        echo "Starting talker and listener..."
        python3 /opt/ros/cluster_talker.py &
        exec python3 /opt/ros/cluster_listener.py
        ;;
    demo_talker)
        echo "Starting demo talker..."
        exec ros2 run demo_nodes_cpp talker --ros-args --remap __node:=${NODE_ID}_talker
        ;;
    demo_listener)
        echo "Starting demo listener..."
        exec ros2 run demo_nodes_cpp listener --ros-args --remap __node:=${NODE_ID}_listener
        ;;
    idle|*)
        echo "Idle mode - use 'balena ssh <uuid> ros-node' to run commands"
        echo ""
        echo "Available commands:"
        echo "  python3 /opt/ros/cluster_talker.py   - Cluster-aware talker"
        echo "  python3 /opt/ros/cluster_listener.py - Cluster-aware listener"
        echo "  python3 /opt/ros/cluster_status.py   - Cluster status monitor"
        echo ""
        exec sleep infinity
        ;;
esac
