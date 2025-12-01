# ROS2 Jazzy Cluster

A complete ROS2 Jazzy cluster setup with:
- **Discovery Server** running on x64 Linux (your desktop)
- **Cluster Nodes** on Raspberry Pi 5 managed by Balena

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
   │  ROS2 Node  │     │  ROS2 Node  │     │  ROS2 Node  │
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
- `ROS_DISCOVERY_SERVER` = `172.32.1.250:11811`
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

## Repository Structure

```
ros2-cluster/
├── README.md                 # This file
├── discovery-server/         # x64 Linux discovery server + monitor
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── entrypoint.sh
│   └── README.md
└── balena-cluster/           # Raspberry Pi 5 Balena nodes
    ├── Dockerfile.ros2
    ├── docker-compose.yml
    ├── entrypoint.sh
    ├── fastdds_discovery_client.xml
    ├── fastdds_super_client.xml
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
2. Verify Pi can reach desktop: `ping 172.32.1.250` from Pi
3. Check firewall allows UDP 11811

### ros2 node list is empty

```bash
# Reset the ROS2 daemon
ros2 daemon stop
ros2 daemon start
ros2 node list
```

## License

MIT
