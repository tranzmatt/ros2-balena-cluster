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

# Configure Fast DDS for discovery server
#if [ -n "$ROS_DISCOVERY_SERVER" ] && [ ! -f /tmp/fastdds_runtime.xml ]; then
#    SERVER_ADDR=$(echo "$ROS_DISCOVERY_SERVER" | cut -d':' -f1)
#    SERVER_PORT=$(echo "$ROS_DISCOVERY_SERVER" | cut -d':' -f2)
#    SERVER_PORT="${SERVER_PORT:-11811}"
#    
#    if [ "$ROS_SUPER_CLIENT" = "true" ]; then
#        CONFIG_TEMPLATE="/opt/ros/fastdds_super_client.xml"
#    else
#        CONFIG_TEMPLATE="/opt/ros/fastdds_config.xml"
#    fi
#    
#    sed -e "s|<address>172.32.1.250</address>|<address>${SERVER_ADDR}</address>|g" \
#        -e "s|<port>11811</port>|<port>${SERVER_PORT}</port>|g" \
#        "$CONFIG_TEMPLATE" > /tmp/fastdds_runtime.xml
#fi
#
#if [ -f /tmp/fastdds_runtime.xml ]; then
#    export FASTRTPS_DEFAULT_PROFILES_FILE="/tmp/fastdds_runtime.xml"
#fi
