/*
 * Originally included at Autoware.ai version 1.10.0 and
 * has been modified to fit the requirements of Project ASLAN.
 *
 * Copyright (C) 2020 Project ASLAN - All rights reserved
 *
 *  Original copyright notice:
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

#ifndef WAYPOINT_LOADER_CORE_H
#define WAYPOINT_LOADER_CORE_H

// ROS includes
#include <ros/ros.h>

// C++ includes
#include <iostream>
#include <fstream>
#include <vector>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <unordered_map>

#include "aslan_msgs/LaneArray.h"
#include "velocity_replanner.h"

namespace waypoint_maker
{
const std::string MULTI_LANE_CSV = "/tmp/driving_lane.csv";

enum class FileFormat : int32_t
{
  ver1,  // x,y,z,(velocity)
  ver2,  // x,y,z,yaw,(velocity)
  ver3,  // first line consists on explanation of values

  unknown = -1,
};

typedef std::underlying_type<FileFormat>::type FileFormatInteger;

inline double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
}
inline double mps2kmph(double velocity_mps)
{
  return (velocity_mps * 60 * 60) / 1000;
}

class WaypointLoaderNode
{
public:
  WaypointLoaderNode();
  ~WaypointLoaderNode();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher & subscriber
  ros::Publisher lane_pub_;
  ros::Subscriber config_sub_;
  ros::Subscriber output_cmd_sub_;

  // variables
  std::string multi_lane_csv_;
  bool disable_decision_maker_;
  bool replanning_mode_;
  VelocityReplanner replanner_;
  std::vector<std::string> multi_file_path_;
  aslan_msgs::LaneArray output_lane_array_;

  // initializer
  void initPubSub();
  void initParameter(const aslan_msgs::ConfigWaypointLoader::ConstPtr& conf);

  // functions
  void configCallback(const aslan_msgs::ConfigWaypointLoader::ConstPtr& conf);
  void outputCommandCallback(const std_msgs::Bool::ConstPtr& output_cmd);
  void createLaneWaypoint(const std::string& file_path, aslan_msgs::Lane* lane);
  void createLaneArray(const std::vector<std::string>& paths, aslan_msgs::LaneArray* lane_array);
  void saveLaneArray(const std::vector<std::string>& paths, const aslan_msgs::LaneArray& lane_array);

  FileFormat checkFileFormat(const char* filename);
  bool verifyFileConsistency(const char* filename);
  void loadWaypointsForVer1(const char* filename, std::vector<aslan_msgs::Waypoint>* wps);
  void parseWaypointForVer1(const std::string& line, aslan_msgs::Waypoint* wp);
  void loadWaypointsForVer2(const char* filename, std::vector<aslan_msgs::Waypoint>* wps);
  void parseWaypointForVer2(const std::string& line, aslan_msgs::Waypoint* wp);
  void loadWaypointsForVer3(const char* filename, std::vector<aslan_msgs::Waypoint>* wps);
  void parseWaypointForVer3(const std::string& line, const std::vector<std::string>& contents,
                            aslan_msgs::Waypoint* wp);
};

const std::string addFileSuffix(std::string file_path, std::string suffix);
void parseColumns(const std::string& line, std::vector<std::string>* columns);
size_t countColumns(const std::string& line);
}
#endif  // WAYPOINT_LOADER_CORE_H
