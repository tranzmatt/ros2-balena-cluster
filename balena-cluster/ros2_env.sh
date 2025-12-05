#!/bin/bash
# Shared ROS2 environment setup for both entrypoint and interactive shells

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Set defaults
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# Get unique node ID from Balena (auto-injected) or hostname
NODE_ID="${BALENA_DEVICE_NAME_AT_INIT:-${HOSTNAME:-node}}"
NODE_ID=$(echo "$NODE_ID" | tr -cd '[:alnum:]_')

# Set namespace based on device name
if [ -z "$ROS_NAMESPACE" ]; then
    export ROS_NAMESPACE="/${NODE_ID}"
fi

# Export for use in scripts
export NODE_ID

# ROS_DISCOVERY_SERVER is passed from Balena env vars - no XML needed!
# The env var alone is sufficient for CLIENT mode
