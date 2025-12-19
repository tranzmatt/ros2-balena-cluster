# File: Dockerfile
FROM ros:jazzy-ros-base

# Install Fast DDS middleware, CLI tools, and image support
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
       ros-jazzy-fastrtps \
       ros-jazzy-rmw-fastrtps-cpp \
       ros-jazzy-fastrtps-cmake-module \
       ros-jazzy-demo-nodes-cpp \
       ros-jazzy-demo-nodes-py \
       ros-jazzy-sensor-msgs \
       ros-jazzy-cv-bridge \
       ros-jazzy-image-transport \
       python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Set Fast DDS as the default middleware
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Copy cluster scripts (for the monitor to use)
COPY cluster_image_viewer.py /opt/ros/cluster_image_viewer.py
RUN chmod +x /opt/ros/cluster_image_viewer.py

# Source ROS automatically for interactive shells (docker exec)
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

# Entrypoint for container start
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
