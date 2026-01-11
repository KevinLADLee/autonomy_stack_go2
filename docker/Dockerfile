# Dockerfile for Jetson Orin NX (ARM64) with ROS2 Humble
# Base image: dustynv/ros - Optimized ROS2 Humble for Jetson devices with L4T r35.3.1
# This image includes NVIDIA optimizations and is specifically designed for Jetson platforms
FROM arm64v8/ros:humble-perception-jammy

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Change ubuntu mirror to ustc
RUN sed -i -e 's@//ports.ubuntu.com/\? @//ports.ubuntu.com/ubuntu-ports @g' \
    -e 's@//ports.ubuntu.com@//mirrors.ustc.edu.cn@g' \
    /etc/apt/sources.list

# Install system dependencies
RUN apt-get update && apt-get install -y \
    cmake \ 
    git \
    libgtk2.0-dev \
    pkg-config \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libusb-dev \
    libusb-1.0-0-dev \
    python3-colcon-common-extensions \
    python-is-python3 \
    python3-pip \
    git \
    build-essential \
    cmake \
    libeigen3-dev \
    libpcl-dev \
    ros-humble-tf-transformations \
    ros-humble-joy \
    ros-humble-perception-pcl \
    ros-humble-sensor-msgs-py \
    ros-humble-tf-transformations \
    ros-humble-joy \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-cv-bridge \
    ros-humble-foxglove-bridge \
    ros-humble-ros2bag \
    ros-humble-ros2bag-storage-mcap \
    ros-humble-xacro \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    libopencv-dev \
    libpcl-dev \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install --no-cache-dir transforms3d pyyaml

# Create workspace directory
WORKDIR /workspace

# Copy the source code
COPY src/ /workspace/src/

# Copy entrypoint script early (for better caching)
COPY docker/docker-entrypoint.sh /usr/local/bin/
RUN chmod +x /usr/local/bin/docker-entrypoint.sh


# Build the workspace
# Note: This step may take 30-60 minutes on Jetson Orin NX
# Build/install/log directories stay in container (not mounted from host)
RUN touch src/utilities/goalpoint_rviz_plugin/COLCON_IGNORE \
    && touch src/utilities/teleop_rviz_plugin/COLCON_IGNORE \
    && touch src/utilities/teleop_rviz_plugin_plus/COLCON_IGNORE \
    && touch src/utilities/waypoint_rviz_plugin/COLCON_IGNORE \
    && touch src/exploration_planner/COLCON_IGNORE \
    && /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Source the workspace setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Set working directory
WORKDIR /workspace

# ENV HOST_UID=${HOST_UID:-1000}
# ENV HOST_GID=${HOST_GID:-1000}
# RUN chown -R $HOST_UID:$HOST_GID /workspace

# USER $HOST_UID:$HOST_GID

# Set entrypoint
ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]

# Default command
CMD ["/bin/bash"]

