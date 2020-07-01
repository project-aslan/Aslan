# waypoint_follower

## Overview

- This package has following nodes.

    - pure_pursuit
    - twist_gate
    - twist_filter

## Building:

To build this package individually:
```
catkin build waypoint_follower
# OR
catkin_make --only-pkg-with-deps waypoint_follower #if you have previously built your workspace with catkin_make
```

## Nodes

### pure_pursuit

#### Getting Started

This node is responsible for calculating the twist command for the vehicle to follow, by fitting a curve between the vehicle and the next target waypoint, based on the lookahead distance set.

#### Parameters

Launch Parameters:

| Parameter Name | Default Value | Description |
| -------------- | ------------- | ----------- |
| is_linear_interpolation | True | Linear Interpolation mode |
| publishes_for_steering_robot | False | Publish topic for steering robot |

The speed and the lookahead distance can be set either from the input waypoints (.csv) or the pop dialog at the ASLAN GUI. The difference is:  

    - Dialog: Velocity (kph) and lookahead distance (next target waypoint to perform curve fitting) are set at a constant value
    - Waypoint: Lookahead distance and velocity are set from the waypoints input based on the lookahead ratio and are not constant

#### Running in Terminal

To run this node in the terminal use the following command:

```
roslaunch waypoint_follower pure_pursuit.launch
```

#### Subscribed Topics

- `final_waypoints` (aslan_msgs/Lane)
- `current_pose` (geometry_msgs/PoseStamped)
- `config/waypoint_follower` (aslan_msgs/ConfigWaypointFollower)
- `current_velocity` (geometry_msgs/TwistStamped)

#### Published Topics

- `twist_raw` (geometry_msgs/TwistStamped)
- `ctrl_cmd` (aslan_msgs/ControlCommandStamped)
- `next_waypoint_mark` (visualization_msgs/Marker)
- `next_target_mark` (visualization_msgs/Marker)
- `search_circle_mark` (visualization_msgs/Marker)
- `line_point_mark` (visualization_msgs/Marker)
- `trajectory_circle_mark` (visualization_msgs/Marker)
- `angular_gravity` (std_msgs/Float32)
- `deviation_of_current_position` (std_msgs/Float32)


### twist_filter (Low Pass Filtering)

#### Getting Started

This node is responsible for filtering the twist raw command, using a low pass filter, in order to produce the final twist command (/twist_cmd) that controls the vehicle.

#### Parameters

This package uses 3 parameters in its launch

| Parameter Name | Default Value | Description |
| -------------- | ------------- | ----------- |
| lateral_accel_limit | 0.8 | Lateral Acceleration limit |
| lowpass_gain_linear_x | 0 | Low pass gain linear x |
| lowpass_gain_angular_z | 0 | Low pass gain angular z |

#### Running in Terminal

To run this node in the terminal use the following command:

```
roslaunch waypoint_follower twist_filter.launch
```

#### Subscribed Topics
- `config/twist_filter` (aslan_msgs/ConfigTwistFilter)
- `obstacle_removed` (std_msgs/Bool)
- `twist_raw` (geometry_msgs/TwistStamped)

#### Published Topics
- `twist_cmd` (geometry_msgs/TwistStamped)


### twist_gate

#### Getting Started

This node is responsible for sending the control commands on the vehicle

#### Subscribed Topics
- `/remote_cmd` (aslan_msgs/RemoteCmd)
- `/twist_cmd` (geometry_msgs/TwistStamped)
- `/accel_cmd` (aslan_msgs/AccelCmd)
- `/steer_cmd` (aslan_msgs/SteerCmd)
- `/brake_cmd` (aslan_msgs/BrakeCmd)
- `/ctrl_cmd` (aslan_msgs/ControlCommandStamped)

#### Published Topics
- `/emergency_stop` (std_msgs/Bool)
- `/ctrl_mode` (std_msgs/String)
- `/vehicle_cmd` (aslan_msgs/VehicleCmd)
- `/state_cmd` (std_msgs/Int32)