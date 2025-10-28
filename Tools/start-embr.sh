# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Navigate to ros2_ws (one level up from Tools, then into ros2_ws)
cd "$SCRIPT_DIR/../ros2_ws"

git pull

colcon build

source install/setup.bash

# Launch using the correct ROS2 package name
ros2 launch embr embr_launch.py
wait
