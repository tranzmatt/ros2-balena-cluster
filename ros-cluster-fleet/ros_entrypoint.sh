#!/bin/bash
# File: ros_entrypoint.sh
set -e

# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# --- CLIENT CONFIGURATION ---
if [ -z "$DISCOVERY_SERVER_IP" ]; then
    echo "WARNING: DISCOVERY_SERVER_IP not set. Defaulting to standard Multicast."
else
    # The format required is IP:PORT. We assume port 11811 if not specified.
    # If your server IP is 192.168.1.100, this becomes "192.168.1.100:11811"
    
    echo "------------------------------------------------"
    echo "  ROS 2 CLUSTER CLIENT"
    echo "  Target: $DISCOVERY_SERVER_IP:11811"
    echo "------------------------------------------------"
    
    export ROS_DISCOVERY_SERVER="${DISCOVERY_SERVER_IP}:11811"
fi

exec "$@"
