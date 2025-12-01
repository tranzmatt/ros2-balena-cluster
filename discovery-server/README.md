# ROS2 Jazzy Discovery Server

Fast DDS Discovery Server for x64 Linux with a monitor container for debugging.

## Quick Start

```bash
# Start the discovery server and monitor
docker compose up -d

# View logs
docker compose logs -f discovery-server

# Stop
docker compose down
```

## Services

### discovery-server
The main Fast DDS discovery server listening on port 11811 (UDP).

### monitor
A helper container pre-configured to connect to the discovery server. Use it to verify the cluster is working:

```bash
# Exec into the monitor container
docker exec -it ros-monitor bash

# List all nodes in the cluster
ros2 node list

# List all topics
ros2 topic list

# Echo a topic
ros2 topic echo /some_topic
```

## Native Installation (Alternative)

If you prefer running natively without Docker:

```bash
sudo ./install.sh
```

This installs a systemd service.

## Firewall

Ensure UDP port 11811 is accessible:

```bash
# UFW
sudo ufw allow 11811/udp

# firewalld
sudo firewall-cmd --add-port=11811/udp --permanent
sudo firewall-cmd --reload
```

## Verifying Operation

```bash
# Check if server is running
docker compose ps

# Check UDP port is listening
ss -uln | grep 11811

# From a remote machine, test connectivity
nc -zuv 172.32.1.250 11811
```

## Troubleshooting

### Server won't start
```bash
# Check if port is already in use
sudo lsof -i :11811

# View container logs
docker compose logs discovery-server
```

### Monitor can't see nodes
```bash
# Verify environment inside monitor
docker exec -it ros-monitor env | grep ROS

# Should show:
# ROS_DISCOVERY_SERVER=127.0.0.1:11811
# RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```
