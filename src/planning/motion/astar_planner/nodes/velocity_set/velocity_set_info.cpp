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

#include "velocity_set_info.h"

using namespace std;

void joinPoints(const pcl::PointCloud<pcl::PointXYZ>& points1, pcl::PointCloud<pcl::PointXYZ>* points2)
{
  for (const auto& p : points1)
  {
    points2->push_back(p);
  }
}

VelocitySetInfo::VelocitySetInfo()
  : stop_distance_obstacle_(5),
    stop_distance_stopline_(5),
	radar_points_threshold_(1),
	stop_range_(1.3),
    points_threshold_(3),
    detection_height_top_(0.2),
    detection_height_bottom_(-1.7),
    radar_detection_height_top_(2),
    radar_detection_height_bottom_(-0.3),
    deceleration_obstacle_(0.8),
    deceleration_stopline_(0.6),
	accelerate_max_(0.6),
    velocity_change_limit_(2.77),
	deceleration_range_(0),
	temporal_waypoints_size_(100),


    set_pose_(false),
    use_obstacle_sim_(false),
    wpidx_detectionResultByOtherNodes_(-1)
{
  ros::NodeHandle private_nh_("~");
  private_nh_.param<double>("remove_points_upto", remove_points_upto_, 2.3);
}

void VelocitySetInfo::InitParam(ros::NodeHandle private_nh_)
{
  private_nh_.param<double>("radar_detection_height_top_", radar_detection_height_top_, 2);
  private_nh_.param<double>("radar_detection_height_bottom_", radar_detection_height_bottom_, -0.3);
  private_nh_.param<double>("deceleration_obstacle", deceleration_obstacle_, 0.8);
  private_nh_.param<double>("deceleration_stopline", deceleration_stopline_, 0.6);
  private_nh_.param<double>("accelerate_max", accelerate_max_, 0.6);
  private_nh_.param<double>("velocity_change_limit", velocity_change_limit_, 2.77);
  private_nh_.param<double>("temporal_waypoints_size", temporal_waypoints_size_, 100);
}

void VelocitySetInfo::param_callback(const aslan_msgs::ConfigVelocitySet::ConstPtr& input)
{
  stop_distance_obstacle_ = input->stop_distance_obstacle;
  stop_distance_stopline_ = input->stop_distance_stopline;
  stop_range_ = input->detection_range;
  deceleration_range_ = input->deceleration_range;
  points_threshold_ = input->threshold_points;
  radar_points_threshold_ = input->radar_threshold_points;
  detection_height_top_ = input->detection_height_top;
  detection_height_bottom_ = input->detection_height_bottom;
  radar_points_threshold_ = input->radar_threshold_points;

}


VelocitySetInfo::~VelocitySetInfo()
{
}

void VelocitySetInfo::clearPoints()
{
  points_.clear();
  radar_points_.clear();
}

void VelocitySetInfo::pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ> sub_points;
  pcl::fromROSMsg(*msg, sub_points);

  points_.clear();
  for (const auto &v : sub_points)
  {
    if (v.x == 0 && v.y == 0)
      continue;

    if (v.z > detection_height_top_ || v.z < detection_height_bottom_)
      continue;

    // ignore points nearby the vehicle
    if (v.x * v.x + v.y * v.y < remove_points_upto_ * remove_points_upto_)
      continue;

    points_.push_back(v);
  }

  if (use_obstacle_sim_)
  {
    joinPoints(obstacle_sim_points_, &points_);
    obstacle_sim_points_.clear();
  }
}

void VelocitySetInfo::radar_pointsCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZ> radar_points;
    pcl::fromROSMsg(*msg, radar_points);

    radar_points_.clear();
    for (const auto &v : radar_points)
    {
        if (v.x == 0 && v.y == 0)
            continue;

        if (v.z > radar_detection_height_top_ || v.z < radar_detection_height_bottom_)
            continue;

        radar_points_.push_back(v);
    }
}

void VelocitySetInfo::controlPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  control_pose_ = *msg;

  if (!set_pose_)
    set_pose_ = true;
}

void VelocitySetInfo::localizerPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  localizer_pose_ = *msg;
}

void VelocitySetInfo::obstacleSimCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::fromROSMsg(*msg, obstacle_sim_points_);

  use_obstacle_sim_ = true;
}

void VelocitySetInfo::detectionCallback(const std_msgs::Int32 &msg)
{
    wpidx_detectionResultByOtherNodes_ = msg.data;
}

