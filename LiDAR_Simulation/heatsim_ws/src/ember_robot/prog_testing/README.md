# Algoirthm & Script Testing (Prog Testing)
```
prog_testing/
├── scripts/        # standalone scripts & algorithm prototypes
├── subscribers/    # ROS topic listeners / callbacks / data taps
├── tests/          # validation, sanity checks, expected behavior
```

## Useful Commands
**See Topics & Message Types**
```Bash
ros2 topic list -t
```

**See A Topic**
```Bash
gz topic -e -t /topic
```

## Viewing LiDAR Visually

### In First Terminal
1. ```bash setup.sh```
2. ```bash sim.sh```

### In Second Terminal
1. ```bash start.sh```
2. ```rviz2```
3. Set Topic to: ```ember_bot/sf45b_top/sf45b_gpu_lidar```
4. Add "PointCloud2"
5. Set PointCloud2 topic to: ```/scan/points```

## Viewing Thermal Camera Visually

### In First Terminal
1. ```bash setup.sh```
2. ```bash sim.sh```
3. In top right click the three vertically stacked dots
4. Search and click "Image Display"
  * Should automatically subsribe as of upload of documentation, can select topic in the drop down though if not automatically subscribed.
  * **Note:** The thermal camera is set automatically "white hot" meaning anything white is hot, objects should be set in Kalvin. Also anything to hot will cause the rest of the world to "blip" to black. Issue not resolved as for testing this has been workable for now.
