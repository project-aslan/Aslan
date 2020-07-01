# lidar_localizer

## Overview

- This package has following nodes.

    - ndt_mapping
    - ndt_matching
    - ndt_matching_monitor
    - save_pcd

## Building:

To only build this package, simply do:
```
catkin_make --only-pkg-with-deps lidar_localizer # if you have previously built your workspace with catkin_make
# or 
catkin build lidar_localizer # if you have previously built with catkin build
```

## Nodes

### ndt_mapping

#### Getting Started

Normal Distribution Transform (NDT) mapping is extracting the 3D map from a recorded lidar scan of an area.

#### Parameters

Launch parameters:

| Parameter Name | Default Value | Description |
| -------------- | ------------- | ----------- |
| `use_odom` | False | Use odometry information as input, if available |
| `use_imu` | False | Use the information from the IMU as input, if available |
| `imu_topic` | /imu_raw| Topic where IMU msgs are being published |
| `leaf_size` | 1 | Voxel Grid size of the input scan (downsampling) |
| `max_iterations` | 30 | Maximum number of iterations before stopping matching |
| `max_scan_range` | 200 | Ignore points far than this value (meters) (default 200.0) |
| `min_add_scan_shift` | 1 | Minimum distance between points to be added to the final map |
| `min_scan_range` | 5 | Ignore points closer than this value (meters) |
| `resolution` | 1 | Cell Size while mapping using ND (meters) |
| `step_size` | 0.1 | Increment value between iterations while mapping |
| `trans_epsilon` | 0.01 | Value to decide convergence between iterations (meters) |

#### Running in Terminal

To run this node in the terminal use the following command:

```
roslaunch lidar_localizer ndt_mapping.launch
```

When ndt_mapping has finished processing the entire dataset, click PCD Output to save the 3D map in .pcd format.

### ndt_matching

#### Getting Started

This node  publishes the current robot/vehicle position in the /ndt_pose topic and statistics in the /ndt_stat topic, such as score, execution time.
The score is the result of PCL’s function getFitnessScore() which measures how well an input point cloud matches the reference pointcloud, in other words the alignment error between the input scan and the reference map.
This value can be used to infer the reliability of NDT.

#### Parameters

Launch parameters:

| Parameter Name | Default Value | Description |
| -------------- | ------------- | ----------- |
| `use_odom` | False | Use Odometry to try to reduce errors |
| `use_imu` | False | Use IMU to try to reduce errors |
| `imu_topic` | /imu_raw | Topic where IMU msgs are being published |
| `x` | 0 | Initial pose x - value |
| `y` | 0 | Initial pose y - value |
| `z` | 0 | Initial pose z - value |
| `roll` | 0 | Initial pose roll - value |
| `pitch` | 0 | Initial pose pitch - value |
| `yaw` | 0 | Initial pose yaw - value |
| `init_pos_gnss` | 0 | Vehicle pose value: Initial pose input or GNSS input |
| `resolution` | 1 | Cell Size while matching using ND (meters) |
| `step_size` | 0.1 | Increment value between iterations while matching |
| `trans_epsilon` | 0.01 | Value to decide convergence between iterations (meters) |
| `max_iterations` | 30 | Maximum number of iterations before stopping matching |

#### Running in Terminal

To run this node in the terminal use the following command:

```
roslaunch lidar_localizer ndt_matching
```

### ndt_matching_monitor

#### Getting Started

This node subscribes to /ndt_stat and /ndt_pose topics, and keeps a running average filter on the score value. When the filtered score is beyond some thresholds, ndt_matching_monitor will issue the /initialpose topic (same as rviz) using the last known “healty” pose to force /ndt_matching to initialize.
If a GNSS device is available, an automatic reinitialization will be triggered with it. nmea2pose node is required.
Otherwise, a halt in localization will be started. To reset the halted status, use Initial Pose tool in RVIZ.

#### Parameters

Launch parameters:

|Parameter| Type| Description|
----------|-----|--------
|`iteration_threshold_warn`| integer |Number of maximum iterations before notifying a warning. Default 10.|
|`iteration_threshold_stop`| integer |Number of maximum iterations before notifying a warning. Default 32. |
|`score_delta_threshold`| double |Difference between consecutive scores to consider a safe match. Default 14.0|
|`min_stable_samples`| integer |Minimum number of samples to start monitoring after a reset. Default 30|
|`fatal_time_threshold`| double |When no GNSS is available a prediction based on the previous samples will be employed. If reinitialization fails the algorithm will stop after n secs. Default 2|

##### Subscribed topics

|Topic|Type|Objective|
------|----|---------
|`ndt_stat`|`aslan_msgs/ndt_stat`|Obtain NDT statistics: `score`, `iterations`.|
|`ndt_pose`|`geometry_msgs/PoseStamped`|Obtain pose as calculated by `ndt_matching_node`.|
|`initialpose`|`geometry_msgs/PoseWithCovarianceStamped`|Obtain pose from RVIZ to reinitialize in case of an unrecoverable error.|
|`gnss_pose`|`geometry_msgs/Pose`|If a GNSS device is available it will be used to automatically try to reset NDT, requires `nmea2tfpose`.|

##### Published topics

|Topic|Type|Objective|
------|----|---------
|`initialpose`|`geometry_msgs/PoseWithCovarianceStamped`|Used to cause a reset or halt in `ndt_matching`.|
|`/ndt_monitor/ndt_info_text`|`jsk_rviz_plugins/OverlayText`|Publishes the text to be displayed in RVIZ with the help of `OverlayText` display type.|
|`/ndt_monitor/ndt_status`|`std_msgs/String`|Publishes the text for its later use in safety decision maker.|


#### Running in Terminal

This node can be called individually using this command
```
rosrun lidar_localizer ndt_matching_monitor
```
OR call it alongside the ndt_mapping node using the following command
```
roslaunch lidar_localizer ndt_matching_monitor.launch
```

### save_pcd

#### Getting Started

This package is responsible for saving a point cloud map at a specific path. It's being automatically run inside ndt_mapping

#### Running in Terminal

This node can be called individually using this command
```
rosrun lidar_localizer save_pcd
```
OR call it alongside the ndt_mapping node using the following command
```
rostopic pub /config/ndt_mapping_output aslan_msgs/ConfigNDTMappingOutput -1 -- '[1, 0, ""]' "<pcd_path>/<pcd_name>.pcd" 0.2
```

#### Published Topics
/config/ndt_mapping_output (aslan_msgs/ConfigNDTMappingOutput)
