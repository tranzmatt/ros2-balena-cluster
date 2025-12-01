# File: Dockerfile
FROM ros:jazzy-ros-base

# Install Fast DDS middleware and the CLI tools required to run the server
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y \
       ros-jazzy-fastrtps \
       ros-jazzy-rmw-fastrtps-cpp \
       ros-jazzy-fastrtps-cmake-module \
    && rm -rf /var/lib/apt/lists/*

# Set Fast DDS as the default middleware
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source the ROS environment automatically when the container starts
# This ensures the 'fastdds' command is found in the path
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
