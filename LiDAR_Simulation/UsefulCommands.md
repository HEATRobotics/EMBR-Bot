## TF
Visualizing /tf
```Bash
ros2 run tf2_tools view_frames
```

## ROS Topics
See all existing topics
```Bash
ros2 topic list -t
```

See specific topic designated times
```Bash
#To see the topic once
ros2 topic echo --once /topic_name

#To see the topic a limited amount of times
ros2 topic echo /topic_name | head -n 50
```

### VIA Gazebo Command
See A Topic, note only useable in containers containing GZ
```Bash
gz topic -e -t /topic
```
Or

### ROS Nodes
View Topics In A Graph
```Bash
rqt_graph
```

## Useful Tools
Data Visualization
```Bash
rviz2
```
Run RViz2, since we are on ROS2

## Data Recording
Record a Gazebo animation
```bash
ros2 bag record -o src/ember_robot/datarecordings/rosbags/run_XX --all-topics
```

Play a recording set of data points for the simulation
```Bash
ros2 bag play src/ember_robot/datarecordings/rosbags/run_01
```

### PCL From Sim Recording
#### Play the mcap file
```Bash
ros2 bag play your_file.mcap
```
#### In another terminal
```Bash
ros2 run pcl_ros pointcloud_to_pcd --topic /lidar_pointcloud --prefix frame_
```

Will dump PCD files to your current directory... which need to located in ./datarecordings via...
#### Then Move them :
```Bash
move *.pcd heatsim_ws\src\ember_robot\datarecordings\Point_Cloud_Data
```