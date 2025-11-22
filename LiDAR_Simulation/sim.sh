# Check if container is running
if docker ps | grep -q heat_jazzy; then
    # Make sure on the right directory
    cd ~/heatsim_ws
    # Build the workspace with symlinked sources for faster development
    colcon build --symlink-install
    # Load the workspace environment so ROS2 can find packages
    source install/setup.bash
    # Launch EMBER Gazebo simulation using the EMBER Robot Launch file
    ros2 launch ember_robot gazebo_model.launch.py
fi