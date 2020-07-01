# astar_planner

## Overview

This package is based on the A* search algorithm for path planning.  
The A* algorithm is using a cost function for finding the optimal way of reaching the next target waypoint. Parameters used in the cost function:
- Estimation of the cost of reaching the target
- The actual cost of reaching the previous target

A* search is considered the best planning algorithm for finding the shortest path to a target.

This package includes following nodes:
    - obstacle_avoid
    - obstacle_sim
    - velocity_set

### Building:

To build this package individually:
```
catkin build astar_planner
# OR
catkin_make --only-pkg-with-deps astar_planner #if you have previously built your workspace with catkin_make
```

## Nodes

### obstacle_avoid (Obstacle Search)

#### Getting Started

This node is responsible for detecting obstacles, within a specific avoidance distance and updating the path of the car.

#### Running in Terminal

To run this node in the terminal use the following command:

```
roslaunch astar_planner obstacle_avoid.launch
```

### obstacle_sim

This node is responsible for visualizing obstacles on RVIZ when using the 2d Nav Goal tool. It creates a virtual obstacle in the simulator world.

#### Subscribed Topics
`/base_waypoints` (aslan_msgs/Lane)
`/closest_waypoint` (std_msgs/Int32)
`/current_pose` (geometry_msgs/PoseStamped)
`/current_velocity` (geometry_msgs/TwistStamped)
`/obstacle_waypoint` (std_msgs/Int32)
`/tf` (tf2_msgs/TFMessage)

#### Published Topics
`/safety_waypoints` (aslan_msgs/Lane)

#### Running in Terminal

To run this node in the terminal use the following command:

```
roslaunch astar_planner obstacle_sim.launch
```

### velocity_set (Velocity Request)

#### Getting Started

This node is responsible for updating the vehicle control command (linear velocity and angular acceleration) based on obstacle detection. Based on the distance of the vehicle and the obstacle in it's path, this node is responsible for accelerating, decelerating and stopping the vehicle.

#### Parameters

Launch Parameters:

| Parameter Name | Default Value | Description |
| -------------- | ------------- | ----------- |
| lidar_points_topic | points_no_ground | Input lidar topic for obstacle detection |
| radar_points_topic | radar/target_list_cartesian | Input radar topic for obstacle detection (if applicable) |
| stop_distance_stopline | 5 | Stop Distance for Stopline (m)  |
| stop_distance_obstacle | 15 | Stop Distance for Obstacle (m)  |
| radar_threshold_points | 1 | Radar Points Threshold for object detection |
| detection_range | 1.3 | Detection Range (m) |
| deceleration_range | 0 | Deceleration Range (m) |
| threshold_points | 5 | Lidar Points Threshold |
| detection_height_top | 0.1 | Detection Height Top (m) |
| detection_height_bottom | -1.5 | Detection Height Bottom (m) |


#### Running in Terminal

To run this node in the terminal use the following command:

```
roslaunch astar_planner velocity_set.launch
```

#### Subscribed Topics

- `points_lanes` (sensor_msgs/PointCloud2)
- `radar/target_list_cartesian` (sensor_msgs/PointCloud2)
- `safety_waypoints` (aslan_msgs/Lane)
- `current_velocity` (geometry_msgs/TwistStamped)
- `localizer_pose` (geometry_msgs/PoseStamped)
- `current_pose` (geometry_msgs/PoseStamped)
- `obstacle_sim_pointcloud` (sensor_msgs/PointCloud2)
- `/state/stopline_wpidx` (std_msgs/Int32)

#### Published Topics
- `detection_range` (visualization_msgs/MarkerArray)
- `obstacle` (visualization_msgs/Marker)
- `obstacle_waypoint` (std_msgs/Int32)
- `radar_emergency_stop` (std_msgs/Int32)
- `obstacle_detected_stop` (std_msgs/Bool)
- `obstacle_removed` (std_msgs/Bool)
- `final_waypoints` (aslan_msgs/Lane)
