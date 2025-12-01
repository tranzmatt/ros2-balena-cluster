# ROS2 Jazzy Balena Cluster Nodes

Balena application for Raspberry Pi 5 nodes that connect to an external Fast DDS Discovery Server.

## Prerequisites

- Balena account and CLI installed
- Fleet created for Raspberry Pi 5
- Discovery server running (see `../discovery-server/`)

## Deployment

### 1. Set Fleet Variables

In the Balena Dashboard, set these fleet-wide variables:

| Variable | Value |
|----------|-------|
| `ROS_DISCOVERY_SERVER` | `172.32.1.250:11811` |
| `ROS_DOMAIN_ID` | `0` |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` |
| `FASTRTPS_DEFAULT_PROFILES_FILE` | `/opt/ros/fastdds_config.xml` |
| `ROS_LOCALHOST_ONLY` | `0` |
| `ROS_SUPER_CLIENT` | `false` |

### 2. Push to Fleet

```bash
cd balena-cluster
balena push ros2-balena-rpi5
```

### 3. Add Devices

Flash BalenaOS to your Pi 5 SD cards and add them to the fleet.

## Per-Device Configuration

### Custom Node Names

Each device automatically uses its Balena device name as the ROS node name. You can override this per-device:

```bash
balena env add NODE_NAME my_custom_name --device <uuid>
```

### Super Client Mode

For diagnostic nodes that need full cluster visibility:

```bash
balena env add ROS_SUPER_CLIENT true --device <uuid>
```

## Testing

SSH into a device:

```bash
balena ssh <device-uuid> ros2-node
```

Run ROS2 commands:

```bash
# List all nodes in cluster
ros2 node list

# List topics
ros2 topic list

# Run demo publisher
ros2 run demo_nodes_cpp talker

# On another device, run subscriber
ros2 run demo_nodes_cpp listener
```

## Adding Custom Packages

### Method 1: Modify Dockerfile

Add to `Dockerfile.ros2`:

```dockerfile
# Clone your packages
RUN cd /ros2_ws/src && \
    git clone https://github.com/your/package.git

# Build workspace
RUN cd /ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install
```

### Method 2: Add Local Packages

1. Create a `packages/` directory
2. Add your ROS2 packages
3. Modify Dockerfile to copy and build them

## File Structure

```
balena-cluster/
├── docker-compose.yml           # Balena service definition
├── Dockerfile.ros2              # ROS2 Jazzy ARM64 image
├── fastdds_discovery_client.xml # Fast DDS client config
├── fastdds_super_client.xml     # Fast DDS super client config
├── entrypoint.sh                # Startup script
└── README.md                    # This file
```
