cd EMBR-Bot

git pull

colcon build

source install/setup.bash

ros2 run embr sendRf &
ros2 run embr getTemp &
wait
