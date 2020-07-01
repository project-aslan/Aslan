# Check Message Alive

High level supervisor node, monitoring the health of the system and publishing diagnostic messages for each package.

## Getting Started

This package is tracking the status of some topics and publishing an array of the status of each topic in topic `/sd_diagnostics` using a diagnostics array message. The status of the topic is one of the following:    
1- ALIVE AND PUBLISHING    
2- DIDN'T START    
3- PUBLISHED WRONG DATA     
4- ERROR     

The topics tracked are

| Topic Number | Topic Name | Message Type | Check Message Correctness |
|    :---:     | ---------- | ------------ | ------------------------- |
|  1 | pmap_stat | std_msgs::Bool | False |
|  2 | filtered_points | sensor_msgs::PointCloud2 | False |
|  3 | ndt_pose | geometry_msgs::PoseStamped | False |
|  4 | lane_waypoints_array | aslan_msgs::LaneArray | False |
|  5 | twist_cmd | geometry_msgs::TwistStamped | True |
|  6 | twist_raw | geometry_msgs::TwistStamped | False |
|  7 | points_raw | sensor_msgs::PointCloud2 | True |
|  8 | traffic_waypoints_array | aslan_msgs::LaneArray | False |
|  9 | closest_waypoint | std_msgs::Int32 | False |
| 10 | final_waypoints | aslan_msgs::Lane | False |
| 11 | current_pose | geometry_msgs::PoseStamped | False |
| 12 | safety_waypoints | aslan_msgs::Lane | False |
| 13 | tf | tf2_msgs::TFMessage | True |

For each topic a diagnostic status is created and checked using the following criteria:   
1. All topics are initialized as "DIDN'T START"   
2. A member ros::time variable is saved for each topic. This member is changed at the callback function of each topic   
3. In the diagnostics function we check the time between each topic time member and ros::time::now(), if the time more than a certain threshold then the status of this topic is set to "ERROR", otherwise it is set as "ALIVE AND PUBLISHING"   
4. For the topics `/twist_cmd`, `points_raw`, `tf` we check the validity of the message as well not only the frequency of the message. If any topic message was caught as wrong, its status changes to "PUBLISHED WRONG DATA"   

## Message Validation Check
 #### 1- twist_cmd
 This topic is responsible for sending the angular z and linear x velocity commands to the vehicle, which correspond to steering and throttle commands.   
 For this topic we are checking if the message is linear.x = 0 and angular.z = 0, and then we check the following:  
 a- If the emergency flag is raised   
 b- If we reached the end of waypoints   
 c- In any other case, we set the status of twist_cmd topic to "PUBLISHED WRONG DATA", and ROS::ERROR this case.    

 #### 2- points_raw
 This topic is publishing the lidar point cloud messages as received from the sensor.  
 For this topic we are checking if there are points published or if the received pointcloud topic is empty. In the case of an empty point cloud, we set the status of points_raw topic to "PUBLISHED WRONG DATA", and ROS::ERROR this case.

 #### 3- tf
This topic is responsible for the transformation between the frames: localizer (lidar), base_link, point cloud map, and the world. Hence we check for 3 transformations:

| frame_id | child_frame_id |
| -------- | ----- |
| base_link | velodyne |
| map | base_link |
| world | map |

If any of those transformations is wrong or missing, we set the tf topic "PUBLISHED WRONG DATA", and ROS::ERROR this case.


## Installing

To build this package individually:

```
catkin_make --only-pkg-with-deps check_message_alive
#OR
catkin build check_message_alive
```

## Running in Terminal

To run this package in the terminal use the following command:

```
rosrun check_message_alive check_message_alive
```
