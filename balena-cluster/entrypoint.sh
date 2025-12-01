#!/bin/bash
set -e

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Set defaults
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

echo "------------------------------------------------"
echo "  ROS 2 CLUSTER CLIENT"

# Configure Discovery Server connection
if [ -n "$ROS_DISCOVERY_SERVER" ] && [ "$ROS_DISCOVERY_SERVER" != "" ]; then
    echo "  Connecting to Brain at: $ROS_DISCOVERY_SERVER"
    echo "------------------------------------------------"
    
    # Parse server address and port
    SERVER_ADDR=$(echo "$ROS_DISCOVERY_SERVER" | cut -d':' -f1)
    SERVER_PORT=$(echo "$ROS_DISCOVERY_SERVER" | cut -d':' -f2)
    SERVER_PORT="${SERVER_PORT:-11811}"
    
    # Select config template based on super client mode
    if [ "$ROS_SUPER_CLIENT" = "true" ]; then
        CONFIG_TEMPLATE="/opt/ros/fastdds_super_client.xml"
    else
        CONFIG_TEMPLATE="/opt/ros/fastdds_config.xml"
    fi
    
    # Create runtime config with substituted address
    RUNTIME_CONFIG="/tmp/fastdds_runtime.xml"
    sed -e "s|<address>172.32.1.250</address>|<address>${SERVER_ADDR}</address>|g" \
        -e "s|<port>11811</port>|<port>${SERVER_PORT}</port>|g" \
        "$CONFIG_TEMPLATE" > "$RUNTIME_CONFIG"
    
    export FASTRTPS_DEFAULT_PROFILES_FILE="$RUNTIME_CONFIG"
else
    echo "  WARNING: ROS_DISCOVERY_SERVER not set!"
    echo "  Using Simple Discovery (multicast)"
    echo "------------------------------------------------"
fi

# Source workspace if it exists
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

echo "  Node ready. Executing: $@"
echo "------------------------------------------------"

# Execute the command passed to the container
exec "$@"
