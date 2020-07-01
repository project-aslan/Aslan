# Voxel Grid Filter

## Getting Started

This package is responsible for downsampling the input point cloud data produced by the lidar using the voxel grid filtering technique.

## Parameters

Launch Parameters:

| Parameter Name | Default Value | Description |
| -------------- | ------------- | ----------- |
| points_topic | /points_raw | |
| measurement_range | 200 | Measurement Range |
| voxel_leaf_size | 2.0 | Voxel Leaf Size |

## Installing

To install this package You can either build the whole catkin workspace or build individually using

```
catkin build voxel_grid_filter
OR
catkin_make --only-pkg-with-deps voxel_grid_filter #if you have previously built your workspace with catkin_make
```

## Running in Terminal

To run this package in the terminal use the following command:

```
roslaunch voxel_grid_filter voxel_grid_filter.launch
```
