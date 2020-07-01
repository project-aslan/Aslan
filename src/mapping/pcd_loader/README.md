# pcd_loader

## Getting Started

This package is responsible for loading a pre-recorded point cloud (.pcd) map.

### Installing

To build this package individually:

```
catkin build pcd_loader
# OR
catkin_make --only-pkg-with-deps pcd_loader #if you have previously built your workspace with catkin_make
```

## Parameters

Launch parameters:

| Parameter Name | Default Value | Description |
| -------------- | ------------- | ----------- |
| `pcd_file` | <path_to_pcd>/<name_of_the_point_cloud>.pcd | Path to the point cloud map |

## Running in Terminal

To run this package in the terminal use the following command:

```
roslaunch pcd_loader points_map_loader.launch pcd_file:=<path_to_pcd>/<name_of_the_point_cloud>.pcd
```

