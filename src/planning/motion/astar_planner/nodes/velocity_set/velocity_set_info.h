/*
 * Originally included at Autoware.ai version 1.10.0 and
 * has been modified to fit the requirements of Project ASLAN.
 *
 * Copyright (C) 2020 Project ASLAN - All rights reserved
 *
 * Original copyright notice:
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VELOCITY_SET_INFO_H
#define VELOCITY_SET_INFO_H

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <aslan_msgs/ConfigVelocitySet.h>

class VelocitySetInfo
{
 private:
  // parameters
    double stop_distance_obstacle_;   // (meter) stopping distance from obstacles
	double stop_distance_stopline_;   // (meter) stopping distance from stoplines
	int radar_points_threshold_;
	double stop_range_;               // if obstacle is in this range, stop
	int points_threshold_;            // points threshold to find obstacles for lidar
	double detection_height_top_;     // from lidar sensor
	double detection_height_bottom_;  // from lidar sensor
	double radar_detection_height_top_;     // from radar sensor
	double radar_detection_height_bottom_;  // from radar sensor
	double deceleration_obstacle_;    // (m/s^2) deceleration for obstacles
	double deceleration_stopline_;    // (m/s^2) deceleration for stopline
	double accelerate_max_;    // (m/s^2) deceleration for obstacles
	double velocity_change_limit_;    // (m/s)
	double deceleration_range_;       // if obstacle is in this range, decelerate
	double temporal_waypoints_size_;  // (meter)


  // ROS param
  double remove_points_upto_;

  pcl::PointCloud<pcl::PointXYZ> points_;
  pcl::PointCloud<pcl::PointXYZ> radar_points_;
  pcl::PointCloud<pcl::PointXYZ> obstacle_sim_points_;
  geometry_msgs::PoseStamped localizer_pose_;  // pose of sensor
  geometry_msgs::PoseStamped control_pose_;    // pose of base_link
  bool set_pose_;
  bool use_obstacle_sim_;
  int  wpidx_detectionResultByOtherNodes_; // waypoints index@finalwaypoints


public:
  VelocitySetInfo();
  ~VelocitySetInfo();

  // ROS Callback
  void InitParam(ros::NodeHandle private_nh_);
  void param_callback(const aslan_msgs::ConfigVelocitySet::ConstPtr& input);
  void pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void radar_pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

  void controlPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void localizerPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void obstacleSimCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void detectionCallback(const std_msgs::Int32 &msg);

  void clearPoints();


  int getDetectionResultByOtherNodes() const
  {
	  return wpidx_detectionResultByOtherNodes_;
  }

  double getStopRange() const
  {
    return stop_range_;
  }

  double getDecelerationRange() const
  {
    return deceleration_range_;
  }

  int getPointsThreshold() const
  {
    return points_threshold_;
  }

  int getRadarPointsThreshold() const
  {
      return radar_points_threshold_;
  }

  int getStopDistanceObstacle() const
  {
    return stop_distance_obstacle_;
  }

  int getStopDistanceStopline() const
  {
    return stop_distance_stopline_;
  }

  double getDecelerationObstacle() const
  {
    return deceleration_obstacle_;
  }

  double getDecelerationStopline() const
  {
    return deceleration_stopline_;
  }

  double getAccelerateMax() const
  {
    return accelerate_max_;
  }
  
  double getVelocityChangeLimit() const
  {
    return velocity_change_limit_;
  }

  double getTemporalWaypointsSize() const
  {
    return temporal_waypoints_size_;
  }

  pcl::PointCloud<pcl::PointXYZ> getPoints() const
  {
    return points_;
  }

  pcl::PointCloud<pcl::PointXYZ> getRadarPoints() const
  {
      return radar_points_;
  }

  geometry_msgs::PoseStamped getControlPose() const
  {
    return control_pose_;
  }

  geometry_msgs::PoseStamped getLocalizerPose() const
  {
    return localizer_pose_;
  }

  bool getSetPose() const
  {
    return set_pose_;
  }
};

#endif // VELOCITY_SET_INFO_H
