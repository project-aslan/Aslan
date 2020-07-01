# Ray Ground Filter

## Getting Started

This package is responsible for filtering the output point cloud from a sensor (either lidar or radar). It is responsible for performing ground removal, by masking the point cloud received.

## Prerequisites

Launch Parameters:

| Parameter Name | Default Value | Description |
| -------------- | ------------- | ----------- |
| input_point_topic | /points_raw | Input_point_topic, ground filtering will be performed over the pointcloud in this topic |
| sensor_height | 1.8 | Height of the sensor from the ground |
| clipping_height | 0.2 | Remove Points above this height value (default 0.2 meters) |
| min_point_distance | 1.85 |  Removes Points closer than this distance from the sensor origin (default 1.85 meters) |
| radial_divider_angle | 0.08 | Angle of each Radial division on the XY Plane (default 0.08 degrees) |
| concentric_divider_distance | 0.01 | Distance of each concentric division on the XY Plane (default 0.01 meters) |
| local_max_slope | 8 | Max Slope of the ground between Points (default 8 degrees) |
| general_max_slope | 5 | Max Slope of the ground in the entire PointCloud, used when reclassification occurs (default 5 degrees) |
| min_height_threshold | 0.05 | Minimum height threshold between points (default 0.05 meters) |
| reclass_distance_threshold | 0.2 | Distance between points at which re classification will occur (default 0.2 meters) |
| no_ground_point_topic | /points_no_ground | Published topic of pointcloud after ground removal |

## Installing

To build this package individually:

```
catkin build ray_ground_filter
# OR
catkin_make --only-pkg-with-deps ray_ground_filter #if you have previously built your workspace with catkin_make
```

## Running in Terminal

To run this package in the terminal use the following command:

```
roslaunch ray_ground_filter ray_ground_filter.launch
```
