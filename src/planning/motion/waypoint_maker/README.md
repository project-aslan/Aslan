# waypoint_maker

## Overview

- This package has following nodes:

    - waypoint_loader
    - waypoint_saver
    - waypoint_marker_publisher

- 3-formats of waypoints.csv handled by waypoint_maker

    - ver1： consist of x, y, z, velocity（no velocity in the first line）

    - ver2： consist of x, y, z, yaw, velocity（no velocity in the first line）

    - ver3： category names are on the first line

## Building:

To build this package independently:
```
catkin build waypoint_maker
# OR
catkin_make --only-pkg-with-deps waypoint_maker #if you have previously built your workspace with catkin_make
```

## Nodes

### waypoint_loader

#### Getting Started

This node is responsible for loading the waypoints from a .csv file and converting them into a ROS message type.

#### Parameters

Launch parameters:

| Parameter Name | Default Value | Description |
| -------------- | ------------- | ----------- |
| disable_decision_maker | True | Disable automatic velocity decision |
| replanning_mode | False | Enable velocity replanning mode|

#### Running in Terminal

To run this node in the terminal use the following command:  
(This will also launch the waypoint_marker_publisher node)

```
roslaunch waypoint_maker waypoint_loader.launch _multi_lane_csv:="path_to_file.csv"
```

#### Subscribed Topics
`/config/waypoint_loader_output` (std_msgs/Bool)

#### Published Topics
`/lane_waypoints_array` (aslan_msgs/LaneArray)

### waypoint_saver

#### Getting Started

This node is responsible for saving the output waypoints to a CSV file. To each of the waypoints, it assigns coordinates (x,y,z) values relative to a common reference point, assigns a velocity value to each and then saves those points in a CSV file.

- When activated, subscribes `/current_pose`, `/current_velocity` (optional) and saves waypoints at specified intervals in the file specified.

#### Parameters

This package uses 5 parameters in its launch

| Parameter Name | Default Value | Description |
| -------------- | ------------- | ----------- |
| save_filename | "/tmp/saved_waypoints.csv" | Path and name of the csv file to save the waypoints |
| interval | 1 | Difference in distance between each waypoint and the following one |
| save_velocity | False | Attach velocity to waypoints or keep it to zero |

#### Running in Terminal

To run this node in the terminal use the following command:

```
roslaunch waypoint_maker waypoint_saver.launch save_filename:="your_path".csv
```

### waypoint_marker_publisher

#### Getting Started

This package is responsible for the visualization of waypoints on the map in rviz. It marks points in the map as waypoints and visualizes the velocity value of each waypoint, respectively.

#### Running in Terminal

This node can be called individually using this command
```
rosrun waypoint_maker waypoint_marker_publisher
```
OR call it alongside the waypoint_loader node using the following command
```
roslaunch waypoint_maker waypoint_loader.launch
```
#### Subscribed Topics

- `/current_pose` (geometry_msgs/PoseStamped) : default
- `/current_velocity` (geometry_msgs/TwistStamped) : default
