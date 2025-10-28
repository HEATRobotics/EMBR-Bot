# ===============================================================
# 🐳 HEAT Robotics ROS2 Jazzy + Gazebo Harmonic Development Environment
# ===============================================================
FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-c"]

# 1. Install core tools
RUN apt-get update && apt-get install -y \
    sudo nano curl git locales build-essential wget \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# 2. Configure locale
RUN locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# 3. Initialize rosdep
RUN sudo rosdep init || true && rosdep update

# 4. Add Gazebo repository
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list

# 5. Install Gazebo Harmonic + ROS bridge packages
RUN apt-get update && apt-get install -y \
    gz-harmonic \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-nav2-bringup \
    ros-jazzy-turtlebot4-simulator \
    && rm -rf /var/lib/apt/lists/*

# 6. Create a clean non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME 2>/dev/null || groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1) \
    && useradd -m -s /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME 2>/dev/null || usermod -l $USERNAME -d /home/$USERNAME $(getent passwd $USER_UID | cut -d: -f1) \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# 7. Create runtime directory for GUI applications (fixes Qt/Gazebo GUI issues)
RUN mkdir -p /run/user/$USER_UID && \
    chown -R $USERNAME:$USERNAME /run/user/$USER_UID && \
    chmod 700 /run/user/$USER_UID

# 8. Switch to non-root user
USER $USERNAME

# 9. Set XDG_RUNTIME_DIR environment variable
ENV XDG_RUNTIME_DIR=/run/user/$USER_UID

# 10. Setup workspace
WORKDIR /home/$USERNAME/ros2_ws
RUN mkdir -p /home/$USERNAME/ros2_ws/src

# 11. Setup .bashrc to auto-source ROS2
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "if [ -f ~/ros2_ws/install/setup.bash ]; then source ~/ros2_ws/install/setup.bash; fi" >> ~/.bashrc && \
    echo "echo 'ROS2 Jazzy environment loaded ✓'" >> ~/.bashrc

# 12. Add entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["tail", "-f", "/dev/null"]