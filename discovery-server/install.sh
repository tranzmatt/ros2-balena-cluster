#!/bin/bash
set -e

# ROS2 Jazzy Discovery Server Installation Script
# For Ubuntu 24.04 (Noble) x64

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} ROS2 Jazzy Discovery Server Installer ${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run as root (sudo ./install.sh)${NC}"
    exit 1
fi

echo -e "${GREEN}[1/4]${NC} Adding ROS2 apt repository..."
apt-get update
apt-get install -y curl gnupg lsb-release

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list

echo -e "${GREEN}[2/4]${NC} Installing ROS2 Jazzy packages..."
apt-get update
apt-get install -y ros-jazzy-ros-base ros-jazzy-rmw-fastrtps-cpp

echo -e "${GREEN}[3/4]${NC} Installing systemd service..."
cp ros2-discovery-server.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable ros2-discovery-server.service

echo -e "${GREEN}[4/4]${NC} Starting discovery server..."
systemctl start ros2-discovery-server.service

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN} Installation Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Discovery server is running on port 11811 (UDP)"
echo ""
echo "Useful commands:"
echo "  systemctl status ros2-discovery-server"
echo "  journalctl -u ros2-discovery-server -f"
echo ""

HOST_IP=$(hostname -I | awk '{print $1}')
echo -e "Clients should set: ${YELLOW}ROS_DISCOVERY_SERVER=${HOST_IP}:11811${NC}"
echo ""
