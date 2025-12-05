# ROS2 Jazzy Cluster

A complete ROS2 Jazzy cluster setup with:
- **Discovery Server** running on x64 Linux (your desktop)
- **Cluster Nodes** on Raspberry Pi 5 managed by Balena
- **USB Webcam Support** for image streaming across the cluster

## Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                      x64 Linux Desktop                         │
│                                                                │
│    ┌──────────────────────────────────────────────────────┐   │
│    │         Fast DDS Discovery Server                     │   │
│    │              172.32.1.250:11811                       │   │
│    └──────────────────────────────────────────────────────┘   │
│    ┌──────────────────────────────────────────────────────┐   │
│    │         Monitor Container                             │   │
│    │         (ros2 node list, ros2 topic echo, etc.)      │   │
│    └──────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────────┘
                              │
          ┌───────────────────┼───────────────────┐
          │                   │                   │
   ┌──────┴──────┐     ┌──────┴──────┐     ┌──────┴──────┐
   │   Pi 5 #1   │     │   Pi 5 #2   │     │   Pi 5 #N   │
   │  (Balena)   │     │  (Balena)   │     │  (Balena)   │
   │  📷 Camera  │     │  📷 Camera  │     │  ROS2 Node  │
   └─────────────┘     └─────────────┘     └─────────────┘
```

## Quick Start

### 1. Start the Discovery Server (x64 Desktop)

```bash
cd discovery-server
docker compose up -d
```

### 2. Deploy Balena Cluster Nodes

Set fleet variables in Balena Dashboard:
- `ROS_DISCOVERY_SERVER` = `192.168.100.10:11811`
- `ROS_DOMAIN_ID` = `0`
- `RMW_IMPLEMENTATION` = `rmw_fastrtps_cpp`
- `ROS_LOCALHOST_ONLY` = `0`

```bash
cd balena-cluster
balena push ros2-balena-rpi5
```

### 3. Verify the Cluster Works

**Step A: Check nodes are discovered (from desktop)**
```bash
docker exec -it ros-monitor bash
ros2 node list
```

**Step B: Start a talker on one Pi**
```bash
balena ssh <device-uuid> ros-node
ros2 run demo_nodes_cpp talker
```

**Step C: Listen from desktop monitor**
```bash
docker exec -it ros-monitor bash
ros2 topic echo /chatter
```

You should see messages from the Pi appearing on your desktop!

**Step D: Listen from another Pi (Pi-to-Pi)**
```bash
balena ssh <other-device-uuid> ros-node
ros2 run demo_nodes_cpp listener
```

## Camera Streaming

### Setting Up Camera Nodes

For Pi devices with USB webcams attached:

1. Set the device variable in Balena Dashboard:
   ```
   NODE_ROLE = camera
   ```

2. Optionally configure camera settings (fleet or device level):
   ```
   CAMERA_WIDTH = 640
   CAMERA_HEIGHT = 480
   CAMERA_FPS = 1.0
   CAMERA_JPEG_QUALITY = 80
   ```

### Viewing Camera Streams

**From the desktop monitor:**
```bash
docker exec -it ros-monitor bash

# List camera topics
ros2 topic list | grep camera

# Check frame rate
ros2 topic hz /cluster_camera/compressed

# View image headers
ros2 topic echo /cluster_camera/compressed --field header

# Run the image viewer
python3 /opt/ros/cluster_image_viewer.py
```

**Save images from all cameras:**
```bash
docker exec -it ros-monitor bash
SAVE_IMAGES=true SAVE_PATH=/tmp/images python3 /opt/ros/cluster_image_viewer.py
```

### Camera NODE_ROLE Options

| Role | Description |
|------|-------------|
| `camera` | Publish webcam images only |
| `camera_viewer` | Receive and track images |
| `camera_talker` | Camera + text messages |
| `full` | Camera + talker + listener |

## Repository Structure

```
ros2-cluster/
├── README.md                 # This file
├── discovery-server/         # x64 Linux discovery server + monitor
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── entrypoint.sh
│   ├── cluster_image_viewer.py
│   └── README.md
└── balena-cluster/           # Raspberry Pi 5 Balena nodes
    ├── Dockerfile.ros2
    ├── docker-compose.yml
    ├── entrypoint.sh
    ├── fastdds_discovery_client.xml
    ├── fastdds_super_client.xml
    ├── cluster_talker.py
    ├── cluster_listener.py
    ├── cluster_status.py
    ├── cluster_camera.py
    ├── cluster_image_viewer.py
    └── README.md
```

## Troubleshooting

### Pi shows "Connecting to Brain at: 127.0.0.1:11811"

The `ROS_DISCOVERY_SERVER` variable isn't being passed. Verify:
1. Fleet variable exists in Balena Dashboard
2. Name is exactly `ROS_DISCOVERY_SERVER` (case sensitive)
3. Redeploy after adding variables

### Desktop monitor can't see Pi nodes

1. Check discovery server is running: `docker compose ps`
2. Verify Pi can reach desktop: `ping 192.168.100.10` from Pi
3. Check firewall allows UDP 11811

### ros2 node list is empty

```bash
# Reset the ROS2 daemon
ros2 daemon stop
ros2 daemon start
ros2 node list
```

### Camera not detected

```bash
# On the Pi via balena ssh
balena ssh <uuid> ros-node

# List video devices
v4l2-ctl --list-devices

# Test OpenCV capture
python3 -c "import cv2; c=cv2.VideoCapture(0); print('OK' if c.isOpened() else 'FAIL')"
```

### No images reaching viewer

1. Check camera is publishing: `ros2 topic hz /cluster_camera/compressed`
2. Verify discovery server connectivity
3. Check camera node logs in Balena dashboard

## License

MIT
