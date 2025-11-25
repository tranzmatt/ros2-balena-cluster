# File: Dockerfile (Server)
FROM ros:jazzy-ros-base

# Install the middleware AND the specific tools package
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y ros-jazzy-fastrtps ros-jazzy-fastrtps-cmake-module \
    && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
