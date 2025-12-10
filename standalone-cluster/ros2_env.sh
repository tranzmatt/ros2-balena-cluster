#!/bin/bash
# Shared ROS2 environment setup for both entrypoint and interactive shells

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Set defaults
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Get unique node ID from environment or hostname
if [ -z "$NODE_ID" ]; then
    NODE_ID="${HOSTNAME:-node}"
fi
NODE_ID=$(echo "$NODE_ID" | tr -cd '[:alnum:]_')

# Set namespace based on device name
if [ -z "$ROS_NAMESPACE" ]; then
    export ROS_NAMESPACE="/${NODE_ID}"
fi

# Export for use in scripts
export NODE_ID

# ROS_DISCOVERY_SERVER is passed from environment / .env file
# The env var alone is sufficient for CLIENT mode

# Print status for interactive shells
if [ -t 1 ]; then
    echo "ROS2 Environment loaded:"
    echo "  ROS_DISCOVERY_SERVER: ${ROS_DISCOVERY_SERVER:-NOT SET}"
    echo "  NODE_ID: ${NODE_ID}"
    echo "  ROS_NAMESPACE: ${ROS_NAMESPACE}"
fi
