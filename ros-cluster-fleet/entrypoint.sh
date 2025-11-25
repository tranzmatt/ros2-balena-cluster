#!/bin/bash
# File: ros_entrypoint.sh
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# --- CLUSTER CONFIGURATION ---
if [ -z "$DISCOVERY_SERVER_IP" ]; then
    echo "WARNING: DISCOVERY_SERVER_IP not set. Defaulting to Multicast."
else
    # Format: IP_ADDRESS:PORT
    # We append the default port 11811
    SERVER_ADDRESS="${DISCOVERY_SERVER_IP}:11811"
    
    echo "------------------------------------------------"
    echo "  ROS 2 CLUSTER CLIENT"
    echo "  Connecting to Brain at: $SERVER_ADDRESS"
    echo "------------------------------------------------"
    
    export ROS_DISCOVERY_SERVER="$SERVER_ADDRESS"
fi

exec "$@"
