cd EMBR-Bot/ros2_ws

git pull

colcon build

source install/setup.bash

cd src/embr/launch
ros2 launch embr_launch.py
wait
