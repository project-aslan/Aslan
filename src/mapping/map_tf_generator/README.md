# Map TF Generator

## Getting Started

This package is responsible for the transformation (tf) from the point cloud map loaded, to the world of the simulator. The transformation is done by transforming each point from the pcd file using its Vector3 cartesian dimension and rotation is calculated using the quaternion using initial roll = 0, pitch = 0, and yaw = 0.

## Prerequisites

pcd_loader package is needed for loading a point cloud map that this package is going to map to the simulation world.

### Installing

To install this package you can either build the whole catkin workspace or build individually using

```
catkin build map_tf_generator
# OR
catkin_make --only-pkg-with-deps map_tf_generator #if you have previously built your workspace with catkin_make
```

## Parameters

`pcd_file` is used in the pcd_loader which is a prerequisite to run this package. pcd_file is used to determine the file path of the point cloud used in the map transformation to simulation world.

## Running in Terminal

Make sure to run the pcd_loader first before executing this package

```
roslaunch pcd_loader points_map_loader.launch pcd_file:=<path_to_pcd>/<name_of_the_point_cloud>.pcd
```

To run this package in the terminal use the following command:

```
roslaunch map_tf_generator map_tf_generate.launch
```