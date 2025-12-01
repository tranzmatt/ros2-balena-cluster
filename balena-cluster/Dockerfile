# File: Dockerfile
FROM ros:jazzy-ros-base

# 1. Install Fast DDS middleware + Ping for debugging
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y ros-jazzy-rmw-fastrtps-cpp iputils-ping \
       python3-pip python3-venv \
    && rm -rf /var/lib/apt/lists/*

# 2. Set Fast DDS as the default middleware
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 3. Setup Entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

WORKDIR /root/ros2_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
