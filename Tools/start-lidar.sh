cd lightwarelidar2

colcon build

source install/setup.bash


ros2 run lightwarelidar2 sf45b   --ros-args   -p port:="/dev/ttyACM1" \
-p baudrate:=115200 \
-p frame_id:="laser" \
-p maxPoints:=1000 \
-p updateRate:=10 \
-p cycleDelay:=5 \
-p lowAngleLimit:=-160 \
-p highAngleLimit:=160 \
-p publishLaserScan:=true
