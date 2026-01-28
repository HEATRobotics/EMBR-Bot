Visualizing /tf
```Bash
ros2 run tf2_tools view_frames
```

See Topics & Message Types
```Bash
ros2 topic list -t
```

See A Topic
```Bash
gz topic -e -t /topic
```

See /scan/points Point Cloud
```Bash
gz topic -e -t /scan/points | head -n 40
```
## 3D Visualization
Check ROS Version, Should Be ROS2 
```Bash
env | grep ROS
```

```Bash
ros2 run rviz2 rviz2
```
Run RViz2, since we are on ROS2

```bash
ros2 bag record -o src/ember_robot/datarecordings/rosbags/run_XX --all-topics
```
Record a Gazebo animation

```Bash
ros2 bag play src/ember_robot/datarecordings/rosbags/run_01
```
Play a recording set of data points for the simulation

## PCL From Sim Recording
#### Play the mcap file
```Bash
ros2 bag play your_file.mcap
```
#### In another terminal
```Bash
ros2 run pcl_ros pointcloud_to_pcd --topic /lidar_pointcloud --prefix frame_
```
Will dump PCD files to your current directory... **Change This?**

#### Then Move them :
```Bash
move *.pcd heatsim_ws\src\ember_robot\datarecordings\Point_Cloud_Data
```
(Look for an alternative way to do this)