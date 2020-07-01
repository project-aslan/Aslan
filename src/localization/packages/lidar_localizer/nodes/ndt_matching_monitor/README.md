# ndt_matching_monitor

This package was originally developed for Autoware. It's responsible for simple health monitoring and reinitialization for `ndt_matching` localization node.

## Introduction
`ndt_matching` publishes the current robot/vehicle position in the `/ndt_pose` topic and statistics 
in the `/ndt_stat` topic, such as score, execution time. 

The score is the result of PCL's function `getFitnessScore()` which measures how well an input point cloud matches
 the reference pointcloud, in other words the alignment error between the input scan and the reference map.
 This value can be used to infer the reliability of NDT.

`ndt_matching_monitor` subscribes to `/ndt_stat` and `/ndt_pose` topics, and keeps a running average filter on 
the score value. When the filtered score is beyond some thresholds, `ndt_matching_monitor` will issue 
the `/initialpose` topic (same as rviz) using the last known "healty" pose to force `/ndt_matching` to initialize.


## State of work
`ndt_matching_monitor` is far from complete but is a great starting point. 
For instance, the last known "healthy" pose stated above is just the last `/ndt_pose` value.
A Kalman or particle filter might be used to improve this node. 
