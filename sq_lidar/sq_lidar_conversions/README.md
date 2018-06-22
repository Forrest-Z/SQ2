# sq\_lidar\_conversions

 This repository uses [ROS](http://wiki.ros.org/). This converts the sensor\_msgs/Scan to the sensor\_msgs/PointCloud2 in ROS. In addition, it provides nodes to save point cloud by using odometry of robot.

## Requirements
- ROS indigo(Ubuntu 14.04) or ROS kinetic(Ubuntu 16.04)
- PCL 1.7+, 1.8+

## How to build
```
$ cd $HOME
$ cd ros_catkin_ws/src
$ git clone https://github.com/aoyan27/sq_lidar_conversions
$ cd ../
$ catkin_make
```

## How to run 
```
$ roslaunch sq_lidar_conversions sq_lidar_convertor.launch
```
