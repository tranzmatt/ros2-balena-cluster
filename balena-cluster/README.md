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
| `ROS_LOCALHOST_ONLY` | `0` |
| `ROS_SUPER_CLIENT` | `false` |

### 2. Push to Fleet

```bash
cd balena-cluster
balena push ros2-balena-rpi5
```

## Verifying the Cluster Works

The containers start with `sleep infinity` so you can exec in and run commands.

### Method 1: From Desktop Monitor (Recommended)

On your desktop where the discovery server runs:

```bash
# Exec into the monitor container
docker exec -it ros-monitor bash

# List all nodes in the cluster (should show Pi nodes)
ros2 node list

# List all topics
ros2 topic list
```

### Method 2: Cross-Node Communication Test

**On Pi #1** (via Balena SSH or web terminal):
```bash
balena ssh <device-uuid> ros-node

# Start a talker
ros2 run demo_nodes_cpp talker
```

**On Desktop Monitor:**
```bash
docker exec -it ros-monitor bash

# Listen to the Pi's talker
ros2 topic echo /chatter
```

You should see messages from the Pi appearing on your desktop!

**On Pi #2** (to verify Pi-to-Pi communication):
```bash
balena ssh <device-uuid> ros-node

# Listen to Pi #1's talker
ros2 run demo_nodes_cpp listener
```

### Method 3: Quick Cluster Health Check

From the desktop monitor, run this to see all discovered participants:

```bash
docker exec -it ros-monitor bash

# See what the discovery server knows about
ros2 daemon stop
ros2 daemon start
ros2 node list
```

## Running Your Own Nodes

Once verified, you can either:

1. **Exec in and run manually:**
   ```bash
   balena ssh <uuid> ros-node
   ros2 run my_package my_node
   ```

2. **Change the default command** in docker-compose.yml:
   ```yaml
   command: ros2 run my_package my_node
   ```

3. **Add your packages** to the Dockerfile and rebuild

## Troubleshooting

### "Connecting to Brain at: 127.0.0.1:11811"

The `ROS_DISCOVERY_SERVER` environment variable isn't being passed. Check:
1. Fleet variable is set correctly in Balena Dashboard
2. Variable name is exactly `ROS_DISCOVERY_SERVER` (case sensitive)

### Nodes not seeing each other

1. Verify discovery server is running on desktop
2. Check all devices can ping 172.32.1.250
3. Ensure UDP port 11811 is open
4. Verify all nodes have same `ROS_DOMAIN_ID`
