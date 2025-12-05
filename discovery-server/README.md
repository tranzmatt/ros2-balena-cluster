# ROS2 Jazzy Discovery Server

Fast DDS Discovery Server for x64 Linux with a monitor container for cluster verification.

## Quick Start

```bash
# Start the discovery server and monitor
docker compose up -d

# Check status
docker compose ps
```

## Services

### discovery-server
The Fast DDS discovery server listening on port 11811 (UDP).

### monitor
A ROS2 client pre-configured to connect to the discovery server. **This is your window into the cluster.**

## Using the Monitor

The monitor container is how you verify the cluster is working:

```bash
# Exec into the monitor
docker exec -it ros-monitor bash

# List all nodes in the cluster
ros2 node list

# List all topics
ros2 topic list

# Echo a topic from a Pi node
ros2 topic echo /chatter

# Publish a test message that Pi nodes can receive
ros2 topic pub /test std_msgs/String "data: hello from desktop"

# Run a listener to hear Pi talkers
ros2 run demo_nodes_cpp listener
```

## Camera Streams

View images from the cluster:

```bash
docker exec -it ros-monitor bash

# List camera topics
ros2 topic list | grep camera

# Check image rate
ros2 topic hz /cluster_camera/compressed

# Run the image viewer (tracks all camera sources)
python3 /opt/ros/cluster_image_viewer.py

# Save images to disk
SAVE_IMAGES=true SAVE_PATH=/tmp/images python3 /opt/ros/cluster_image_viewer.py
```

## Verifying Cluster Connectivity

1. **Start a talker on any Pi:**
   ```bash
   balena ssh <uuid> ros-node
   ros2 run demo_nodes_cpp talker
   ```

2. **Listen from the desktop monitor:**
   ```bash
   docker exec -it ros-monitor bash
   ros2 topic echo /chatter
   ```

3. If you see messages, the cluster is working!

## Firewall

Ensure UDP port 11811 is accessible:

```bash
# UFW
sudo ufw allow 11811/udp
```

## Troubleshooting

### Monitor shows no nodes

```bash
# Reset the ROS2 daemon inside monitor
docker exec -it ros-monitor bash
ros2 daemon stop
ros2 daemon start
ros2 node list
```

### Check server is actually running

```bash
# View logs
docker compose logs discovery-server

# Check UDP port
ss -uln | grep 11811
```
