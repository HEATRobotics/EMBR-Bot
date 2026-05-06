# Use Ubuntu 22.04 as base (same as Ubuntu Server 22.04 on Raspberry Pi)
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV LANG=en_US.UTF-8
ENV PYTHONUNBUFFERED=1

# Install basic utilities
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble and development tools (includes msg generation support)
RUN apt-get update && apt-get install -y ros-humble-ros-base ros-dev-tools python3-pip python3-colcon-common-extensions python3-rosdep python3-ament-package ros-humble-rosidl-default-generators ros-humble-rosidl-generator-py git vim nano && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install BlueZ, Bleak, and Dependencies 
RUN apt-get update && apt-get install -y \
    bluez \
    dbus \
    libreadline-dev 

RUN pip3 install --no-cache-dir bleak

# Install Python dependencies
RUN pip3 install --no-cache-dir \
    pyserial \
    pymavlink \
    dronekit \
    pymodbus \
    opencv-python-headless \
    numpy \
    pytest 

# Note: smbus and flirpy are hardware-specific, skipped in simulation

# Create workspace directory
WORKDIR /workspace

# Copy the ROS2 workspace
COPY ros2_ws /workspace/ros2_ws

# Set working directory to ros2_ws
WORKDIR /workspace/ros2_ws

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install"

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Source ROS 2\n\
source /opt/ros/humble/setup.bash\n\
\n\
# Build workspace if not already built (for volume mounts)\n\
if [ ! -f "/workspace/ros2_ws/install/setup.bash" ]; then\n\
    echo "Building ROS2 workspace (first run with volume mount)..."\n\
    cd /workspace/ros2_ws\n\
    colcon build --symlink-install\n\
    echo "Build complete!"\n\
fi\n\
\n\
# Source workspace\n\
source /workspace/ros2_ws/install/setup.bash\n\
\n\
echo "================================================="\n\
echo "EMBR-Bot Docker Container - Simulation Mode"\n\
echo "================================================="\n\
echo ""\n\
echo "ROS 2 Humble is ready!"\n\
echo "Workspace: /workspace/ros2_ws"\n\
echo "Sensor Mode: Uses config/sensors_sim.json by default"\n\
echo ""\n\
echo "Available commands:"\n\
echo "  # Simulation mode (default):"\n\
echo "  ros2 launch embr embr_launch.py config_file:=config/sensors_sim.json"\n\
echo ""\n\
echo "  # Individual nodes:"\n\
echo "  ros2 run embr getTemp --ros-args -p config_file:=config/sensors_sim.json"\n\
echo "  ros2 run embr getCube --ros-args -p config_file:=config/sensors_sim.json"\n\
echo "  ros2 run embr radio --ros-args -p config_file:=config/sensors_sim.json"\n\
echo ""\n\
echo "  # Monitoring:"\n\
echo "  ros2 topic list                       - List topics"\n\
echo "  ros2 topic echo /temperature          - Monitor temperature"\n\
echo "  ros2 topic echo /gps                  - Monitor GPS"\n\
echo ""\n\
echo "Run examples:"\n\
echo "  python3 /workspace/ros2_ws/src/embr/examples/sensor_testing_examples.py"\n\
echo ""\n\
echo "Configuration: /workspace/ros2_ws/src/embr/config/CONFIG.md"\n\
echo "Documentation: /workspace/ros2_ws/src/embr/DOC_INDEX.md"\n\
echo "================================================="\n\
echo ""\n\
\n\
# Execute command\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/entrypoint.sh"]

# Default command
CMD ["/bin/bash", "/entrypoint.sh"]

# Labels
LABEL maintainer="HEAT Robotics"
LABEL description="EMBR-Bot ROS2 Humble simulation environment"
LABEL version="1.0"
