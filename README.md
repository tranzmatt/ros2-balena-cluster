# ROS2 Jazzy Cluster

A complete ROS2 Jazzy cluster setup with:
- **Discovery Server** running on x64 Linux
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

# Option A: Docker
docker compose up -d

# Option B: Native install (Ubuntu 24.04)
sudo ./install.sh
```

### 2. Deploy Balena Cluster Nodes

```bash
# Set fleet variables in Balena Dashboard:
# ROS_DISCOVERY_SERVER=172.32.1.250:11811
# ROS_DOMAIN_ID=0
# RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# ROS_LOCALHOST_ONLY=0

cd balena-cluster
balena push ros2-balena-rpi5
```

### 3. Verify Communication

```bash
# SSH into any Pi
balena ssh <device-uuid> ros2-node

# Check cluster connectivity
ros2 node list
ros2 topic list
```

## Repository Structure

```
ros2-cluster/
├── README.md                 # This file
├── discovery-server/         # x64 Linux discovery server
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── entrypoint.sh
│   ├── install.sh            # Native installation script
│   ├── ros2-discovery-server.service
│   └── README.md
└── balena-cluster/           # Raspberry Pi 5 Balena nodes
    ├── Dockerfile.ros2
    ├── docker-compose.yml
    ├── entrypoint.sh
    ├── fastdds_discovery_client.xml
    ├── fastdds_super_client.xml
    └── README.md
```

## Configuration Reference

### Discovery Server

| Variable | Default | Description |
|----------|---------|-------------|
| `DISCOVERY_SERVER_PORT` | `11811` | UDP port |
| `DISCOVERY_SERVER_ID` | `0` | Server ID |

### Balena Fleet Variables

| Variable | Value | Description |
|----------|-------|-------------|
| `ROS_DISCOVERY_SERVER` | `172.32.1.250:11811` | Discovery server address |
| `ROS_DOMAIN_ID` | `0` | ROS2 domain ID |
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | DDS implementation |
| `ROS_LOCALHOST_ONLY` | `0` | Must be 0 for networking |
| `ROS_SUPER_CLIENT` | `false` | Set true for diagnostics |

## Network Requirements

- All devices on same network or routable subnets
- UDP port 11811 open on discovery server
- No multicast required (discovery server eliminates this need)

## Troubleshooting

See individual README files in each subdirectory for component-specific troubleshooting.

### Common Issues

1. **Nodes not discovering**: Check discovery server is reachable (`nc -zuv 172.32.1.250 11811`)
2. **Build failures**: ARM64 builds take 15-20 minutes on first deployment
3. **Network issues**: Ensure `ROS_LOCALHOST_ONLY=0` is set

## License

MIT
