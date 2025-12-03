# ROS2 Jazzy Balena Cluster Nodes

Balena application for Raspberry Pi 5 nodes that connect to an external Fast DDS Discovery Server.

## Fleet Variables

Set these in the Balena Dashboard under Fleet → Variables:

| Variable | Value | Description |
|----------|-------|-------------|
| `ROS_DISCOVERY_SERVER` | `172.32.1.250:11811` | Discovery server address |
| `ROS_DOMAIN_ID` | `0` | ROS2 domain ID |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | DDS middleware |
| `ROS_LOCALHOST_ONLY` | `0` | Must be 0 for networking |
| `NODE_ROLE` | `idle` | Default role (see below) |

## Device Variables

Set per-device in Balena Dashboard under Device → Device Variables:

| Variable | Options | Description |
|----------|---------|-------------|
| `NODE_ROLE` | `talker`, `listener`, `idle` | What this node does |
| `ROS_SUPER_CLIENT` | `true`, `false` | Full cluster visibility |

### NODE_ROLE Options

- **`talker`** - Automatically runs `ros2 run demo_nodes_cpp talker`
- **`listener`** - Automatically runs `ros2 run demo_nodes_cpp listener`  
- **`idle`** - Container stays alive, SSH in to run custom commands

## Deployment

```bash
cd balena-cluster
balena push ros2-balena-rpi5
```

## Example Setup

**Fleet-wide variables:**
```
ROS_DISCOVERY_SERVER = 172.32.1.250:11811
ROS_DOMAIN_ID = 0
NODE_ROLE = idle
```

**Device A (talker):**
```
NODE_ROLE = talker
```

**Device B (listener):**
```
NODE_ROLE = listener
```

**Device C (custom):**
```
NODE_ROLE = idle
# SSH in and run whatever you want
```

## Manual Control

For devices with `NODE_ROLE=idle`, SSH in and run commands:

```bash
balena ssh <device-uuid> ros-node

# Run a talker
ros2 run demo_nodes_cpp talker

# Run a listener
ros2 run demo_nodes_cpp listener

# List nodes/topics
ros2 node list
ros2 topic list
```

## Verifying the Cluster

On your desktop monitor (with SUPER_CLIENT mode):
```bash
docker exec -it ros-monitor bash
ros2 node list
ros2 topic list
ros2 topic echo /chatter
```
