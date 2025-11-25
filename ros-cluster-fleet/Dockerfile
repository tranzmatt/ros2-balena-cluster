# File: Dockerfile
FROM ros:jazzy-ros-base

# 1. Install Fast DDS tools (client side needs them too) and basics
RUN apt-get update && apt-get install -y \
    ros-jazzy-rmw-fastrtps-cpp \
    iputils-ping \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# 2. Set Fast DDS as the default middleware
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 3. Setup Entrypoint
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# 4. Set Workspace
WORKDIR /root/ros2_ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
