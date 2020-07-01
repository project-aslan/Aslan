# lane_planner

## Overview

- This package has following nodes.

    - traffic_waypoints
    - lane_select

### Building:

To build this package individually:
```
catkin build lane_planner
# OR
catkin_make --only-pkg-with-deps lane_planner #if you have previously built your workspace with catkin_make
```

## Nodes

### traffic_waypoints

#### Getting Started

This node is responsible for publishing the traffic waypoints array.

#### Running in Terminal

To run this node in the terminal use the following command:

```
roslaunch lane_planner traffic_waypoints.launch
```

#### Subscribed Topics
`/lane_waypoints_array` (aslan_msgs/LaneArray)

#### Published Topics
`/traffic_waypoints_array` (aslan_msgs/LaneArray)


### lane_select

#### Getting Started

This node is responsible for extracting the route for the car to follow, by finding the closest waypoint from the vehicle's current position in the map.

#### Parameters

Launch Parameters:

| Parameter Name | Default Value | Description |
| -------------- | ------------- | ----------- |
| distance_threshold | 3 | Distance threshold to neighbor lanes |
| lane_change_interval | 10 | Lane Change Interval After Lane Merge |
| lane_change_target_minimum | 10 | Lane Change Target Minimum |
| lane_change_target_ratio | 5 | Lane Change Target Ratio (m/s) |
| vector_length_hermite_curve | 10 | Vector Length of Hermite Curve |

#### Running in Terminal

To run this node in the terminal use the following command:

```
roslaunch lane_planner lane_select.launch
```

#### Subscribed Topics
- `/traffic_waypoints_array` (waypoint_follower/LaneArray)
- `/current_pose` (geometry_msgs/PoseStamped)
- `/current_velocity` (geometry_msgs/TwistStamped)
- `/state` (std_msgs/String)
- `/config/lane_select` (runtime_manager/ConfigLaneSelect)

#### Published Topics
- `/base_waypoints` (waypoint_follower/lane)
- `/closest_waypoint` (std_msgs/Int32)
- `/change_flag` (std_msgs/Int32)
- `/lane_select_marker` (visualization_msgs/MarkerArray) (optional)