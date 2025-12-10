# ROS2 Jazzy Standalone Cluster Node

A standalone Docker-based ROS2 node that connects to a FastDDS Discovery Server. Runs on any Linux system with Docker - no Balena required.

## Quick Start

### 1. Configure the Node

```bash
# Copy the example config
cp .env.example .env

# Edit to set your discovery server IP and node settings
nano .env
```

**Minimum required settings:**
```bash
ROS_DISCOVERY_SERVER=192.168.1.100:11811   # Your desktop's IP
NODE_ID=pi5_kitchen                         # Unique name for this node
NODE_ROLE=camera                            # What this node does
```

### 2. Build and Run

```bash
# Build the container
docker compose build

# Start the node
docker compose up -d

# View logs
docker compose logs -f
```

### 3. Verify It's Working

From your desktop monitor container:
```bash
docker exec -it ros-monitor bash
ros2 node list
```

You should see your standalone node appear!

## Configuration Options

### NODE_ROLE Options

| Role | Description |
|------|-------------|
| `idle` | Container stays running for manual commands |
| `talker` | Publishes text messages to `/cluster_chat` |
| `listener` | Listens to `/cluster_chat` messages |
| `status` | Monitors message stats from all talkers |
| `both` | Runs both talker and listener |
| `camera` | Publishes webcam images to `/cluster_camera/compressed` |
| `camera_viewer` | Receives and tracks images from all cameras |
| `camera_talker` | Camera + text messages |
| `full` | Camera + talker + listener |
| `demo_talker` | ROS2 demo talker node |
| `demo_listener` | ROS2 demo listener node |

### Camera Settings

| Variable | Default | Description |
|----------|---------|-------------|
| `CAMERA_DEVICE` | auto | Force specific video device index (0, 1, etc.) |
| `CAMERA_WIDTH` | `640` | Capture width in pixels |
| `CAMERA_HEIGHT` | `480` | Capture height in pixels |
| `CAMERA_FPS` | `1.0` | Frames per second to publish |
| `CAMERA_JPEG_QUALITY` | `80` | JPEG quality (1-100) |

### Image Viewer Settings

| Variable | Default | Description |
|----------|---------|-------------|
| `SAVE_IMAGES` | `false` | Save received images to disk |
| `SAVE_PATH` | `/shared/images` | Where to save images |
| `SAVE_INTERVAL` | `10` | Save every N frames |

## Manual Commands

When running in `idle` mode, exec into the container:

```bash
docker exec -it ros-node bash

# List cluster nodes
ros2 node list

# List topics
ros2 topic list

# Run camera manually
python3 /opt/ros/cluster_camera.py

# Run talker
python3 /opt/ros/cluster_talker.py

# Listen to chat
ros2 topic echo /cluster_chat
```

## Camera Troubleshooting

### Check if webcam is detected

```bash
docker exec -it ros-node bash

# List video devices
v4l2-ctl --list-devices

# Test OpenCV capture
python3 -c "import cv2; c=cv2.VideoCapture(0, cv2.CAP_V4L2); print('OK' if c.isOpened() else 'FAIL')"
```

### Force a specific camera device

If auto-detection fails, set in `.env`:
```bash
CAMERA_DEVICE=0
```

### Permission issues

The container runs privileged with `/dev` mounted. If you still have issues:
```bash
# On the host
sudo chmod 666 /dev/video0
```

## Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                      x64 Linux Desktop                         │
│    ┌──────────────────────────────────────────────────────┐   │
│    │         Fast DDS Discovery Server                     │   │
│    │              172.32.1.250:11811                       │   │
│    └──────────────────────────────────────────────────────┘   │
│    ┌──────────────────────────────────────────────────────┐   │
│    │         Monitor Container                             │   │
│    └──────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────────┘
                              │
          ┌───────────────────┼───────────────────┐
          │                   │                   │
   ┌──────┴──────┐     ┌──────┴──────┐     ┌──────┴──────┐
   │   Pi 5 #1   │     │   Pi 5 #2   │     │   Pi 5 #3   │
   │  (Balena)   │     │ (Standalone)│     │ (Standalone)│
   │             │     │  This repo! │     │             │
   └─────────────┘     └─────────────┘     └─────────────┘
```

## Running Multiple Nodes

You can run multiple standalone nodes on different machines. Just ensure each has a unique `NODE_ID` in their `.env` file.

## Running Alongside Balena Nodes

Standalone nodes work seamlessly with Balena-managed nodes - they all connect to the same discovery server and can communicate via ROS2 topics.

## Systemd Service (Optional)

To run on boot without manual intervention:

```bash
# Create service file
sudo tee /etc/systemd/system/ros2-cluster-node.service << 'EOF'
[Unit]
Description=ROS2 Cluster Node
After=docker.service
Requires=docker.service

[Service]
Type=simple
WorkingDirectory=/path/to/standalone-cluster
ExecStart=/usr/bin/docker compose up
ExecStop=/usr/bin/docker compose down
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# Enable and start
sudo systemctl daemon-reload
sudo systemctl enable ros2-cluster-node
sudo systemctl start ros2-cluster-node
```

## Updating

```bash
# Pull latest changes
git pull

# Rebuild container
docker compose build --no-cache

# Restart
docker compose down
docker compose up -d
```
