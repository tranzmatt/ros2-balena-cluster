#!/bin/bash
set -e

# Source shared environment setup
source /opt/ros/ros2_env.sh

# Get unique node ID from environment or hostname
# In standalone mode, NODE_ID can be set in .env file
if [ -z "$NODE_ID" ]; then
    NODE_ID="${HOSTNAME:-node}"
fi

# Sanitize node ID - ROS2 node names can only have alphanumeric and underscores
NODE_ID=$(echo "$NODE_ID" | tr -cd '[:alnum:]_')
export NODE_ID

echo "================================================"
echo "  ROS 2 STANDALONE CLUSTER NODE"
echo "  Discovery Server: ${ROS_DISCOVERY_SERVER:-NOT SET}"
echo "  Node ID: ${NODE_ID}"
echo "  Node Role: ${NODE_ROLE:-idle}"
echo "================================================"

# Warn if discovery server not set
if [ -z "$ROS_DISCOVERY_SERVER" ]; then
    echo ""
    echo "WARNING: ROS_DISCOVERY_SERVER not set!"
    echo "Set it in your .env file or docker-compose.yml"
    echo ""
fi

# Check for webcam if camera role
if [[ "${NODE_ROLE}" == *"camera"* ]]; then
    echo ""
    echo "Checking for USB webcam..."
    if command -v v4l2-ctl &> /dev/null; then
        v4l2-ctl --list-devices 2>/dev/null || echo "No video devices found"
    fi
    echo ""
fi

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
    camera)
        echo "Starting cluster camera publisher..."
        exec python3 /opt/ros/cluster_camera.py
        ;;
    camera_viewer)
        echo "Starting cluster image viewer..."
        exec python3 /opt/ros/cluster_image_viewer.py
        ;;
    camera_talker)
        echo "Starting camera + talker..."
        python3 /opt/ros/cluster_camera.py &
        exec python3 /opt/ros/cluster_talker.py
        ;;
    full)
        echo "Starting camera + talker + listener..."
        python3 /opt/ros/cluster_camera.py &
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
        echo ""
        echo "Idle mode - use 'docker exec -it ros-node bash' to run commands"
        echo ""
        echo "Available commands:"
        echo "  python3 /opt/ros/cluster_talker.py    - Cluster-aware talker"
        echo "  python3 /opt/ros/cluster_listener.py  - Cluster-aware listener"
        echo "  python3 /opt/ros/cluster_status.py    - Cluster status monitor"
        echo "  python3 /opt/ros/cluster_camera.py    - USB webcam publisher"
        echo "  python3 /opt/ros/cluster_image_viewer.py - Image receiver/viewer"
        echo ""
        echo "Camera troubleshooting:"
        echo "  v4l2-ctl --list-devices              - List video devices"
        echo "  v4l2-ctl -d /dev/video0 --all        - Show device capabilities"
        echo ""
        echo "ROS2 commands:"
        echo "  ros2 node list                       - List all nodes in cluster"
        echo "  ros2 topic list                      - List all topics"
        echo "  ros2 topic echo /cluster_chat        - Listen to chat messages"
        echo ""
        exec sleep infinity
        ;;
esac
