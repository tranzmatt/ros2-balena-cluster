# ROS2 Jazzy Balena Cluster Nodes

Balena application for Raspberry Pi 5 nodes that connect to an external Fast DDS Discovery Server, with USB webcam support.

## Fleet Variables

Set these in the Balena Dashboard under Fleet → Variables:

| Variable | Value | Description |
|----------|-------|-------------|
| `ROS_DISCOVERY_SERVER` | `172.32.1.250:11811` | Discovery server address |
| `ROS_DOMAIN_ID` | `0` | ROS2 domain ID |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | DDS middleware |
| `ROS_LOCALHOST_ONLY` | `0` | Must be 0 for networking |
| `NODE_ROLE` | `idle` | Default role (see below) |

### Camera Variables (Optional)

| Variable | Default | Description |
|----------|---------|-------------|
| `CAMERA_DEVICE` | auto | Force specific video device index (0, 1, etc.) |
| `CAMERA_WIDTH` | `640` | Capture width in pixels |
| `CAMERA_HEIGHT` | `480` | Capture height in pixels |
| `CAMERA_FPS` | `1.0` | Frames per second to publish |
| `CAMERA_JPEG_QUALITY` | `80` | JPEG quality (1-100) |
| `SAVE_IMAGES` | `false` | Save received images to disk |
| `SAVE_PATH` | `/shared/images` | Where to save images |
| `SAVE_INTERVAL` | `10` | Save every N frames |

## Device Variables

Set per-device in Balena Dashboard under Device → Device Variables:

| Variable | Options | Description |
|----------|---------|-------------|
| `NODE_ROLE` | See below | What this node does |
| `ROS_SUPER_CLIENT` | `true`, `false` | Full cluster visibility |

### NODE_ROLE Options

**Message Roles:**
- **`talker`** - Publishes text messages to `/cluster_chat`
- **`listener`** - Listens to `/cluster_chat` messages
- **`status`** - Monitors message stats from all talkers
- **`both`** - Runs both talker and listener

**Camera Roles:**
- **`camera`** - Publishes webcam images to `/cluster_camera/compressed`
- **`camera_viewer`** - Receives and tracks images from all cameras
- **`camera_talker`** - Runs both camera and text talker

**Combined Roles:**
- **`full`** - Runs camera + talker + listener

**Demo Roles:**
- **`demo_talker`** - ROS2 demo talker node
- **`demo_listener`** - ROS2 demo listener node
- **`idle`** - Container stays alive for SSH access

## Deployment

```bash
cd balena-cluster
balena push ros2-balena-rpi5
```

## Example Setup: Camera Cluster

**Fleet-wide variables:**
```
ROS_DISCOVERY_SERVER = 172.32.1.250:11811
ROS_DOMAIN_ID = 0
NODE_ROLE = idle
CAMERA_FPS = 0.5
CAMERA_JPEG_QUALITY = 75
```

**Device A (camera node with Logitech webcam):**
```
NODE_ROLE = camera
```

**Device B (camera node with Fullhan webcam):**
```
NODE_ROLE = camera
```

**Device C (viewer/aggregator):**
```
NODE_ROLE = camera_viewer
SAVE_IMAGES = true
```

## Camera Topics

The camera node publishes to two topics:

1. **`/cluster_camera/compressed`** - All cameras publish here (for aggregation)
2. **`/<node_id>/camera/compressed`** - Node-specific topic

Message type: `sensor_msgs/msg/CompressedImage`

## Verifying Camera Functionality

### On a Pi with webcam:

```bash
balena ssh <device-uuid> ros-node

# Check if webcam is detected
v4l2-ctl --list-devices

# Test camera manually
python3 /opt/ros/cluster_camera.py
```

### On your desktop monitor:

```bash
docker exec -it ros-monitor bash

# List image topics
ros2 topic list | grep camera

# Check image rate
ros2 topic hz /cluster_camera/compressed

# Echo image metadata (not the actual bytes)
ros2 topic echo /cluster_camera/compressed --field header
```

### Save images to view them:

On any node (or the desktop), you can run the image viewer with saving enabled:

```bash
SAVE_IMAGES=true SAVE_PATH=/tmp/images python3 /opt/ros/cluster_image_viewer.py
```

## USB Webcam Compatibility

The camera node auto-detects USB webcams and filters out:
- Raspberry Pi ISP devices (`/dev/video19+`)
- Metadata-only devices
- Non-capture devices

Tested with:
- Logitech HD Webcam B910
- Fullhan USB Webcam (generic)

## Manual Control

For devices with `NODE_ROLE=idle`, SSH in and run commands:

```bash
balena ssh <device-uuid> ros-node

# List video devices
v4l2-ctl --list-devices

# Run camera manually
python3 /opt/ros/cluster_camera.py

# Run with custom settings
CAMERA_FPS=2.0 CAMERA_WIDTH=1280 python3 /opt/ros/cluster_camera.py

# Run image viewer
python3 /opt/ros/cluster_image_viewer.py

# Run text talker/listener
python3 /opt/ros/cluster_talker.py
python3 /opt/ros/cluster_listener.py
```

## Troubleshooting

### Camera not detected

```bash
# Check if device exists
ls -la /dev/video*

# Check device permissions
v4l2-ctl -d /dev/video0 --info

# Check if OpenCV can open it
python3 -c "import cv2; c=cv2.VideoCapture(0, cv2.CAP_V4L2); print('OK' if c.isOpened() else 'FAIL')"
```

If auto-detection fails but the camera is present, force the device index:
```
CAMERA_DEVICE = 0
```

### Images not reaching viewer

1. Verify discovery server is running
2. Check topic is being published: `ros2 topic list`
3. Check message rate: `ros2 topic hz /cluster_camera/compressed`

### Image quality issues

Adjust environment variables:
```
CAMERA_JPEG_QUALITY = 90  # Higher = better quality, larger files
CAMERA_WIDTH = 1280       # Higher resolution
CAMERA_HEIGHT = 720
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
│    │    ros2 topic echo /cluster_camera/compressed        │   │
│    └──────────────────────────────────────────────────────┘   │
└────────────────────────────────────────────────────────────────┘
                              │
          ┌───────────────────┼───────────────────┐
          │                   │                   │
   ┌──────┴──────┐     ┌──────┴──────┐     ┌──────┴──────┐
   │   Pi 5 #1   │     │   Pi 5 #2   │     │   Pi 5 #N   │
   │  (Balena)   │     │  (Balena)   │     │  (Balena)   │
   │  📷 Camera  │     │  📷 Camera  │     │  👁 Viewer  │
   │  NODE_ROLE  │     │  NODE_ROLE  │     │  NODE_ROLE  │
   │  = camera   │     │  = camera   │     │  = camera_  │
   │             │     │             │     │    viewer   │
   └─────────────┘     └─────────────┘     └─────────────┘
```
