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

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"


#include "libvelocity_set.h"
#include "velocity_set_info.h"
#include "velocity_set_path.h"
#include <sstream>

using namespace std;

namespace
{
constexpr int LOOP_RATE = 10;

//The distance ahead we search about the deaccelleration radius
constexpr double DECELERATION_SEARCH_DISTANCE = 20;

//The distance ahead we search about the stop radius
constexpr double STOP_SEARCH_DISTANCE = 30;

ros::Publisher emergency_stop_pub;
ros::Publisher obstacle_removed_pub;

void obstacleColorByKind(const EControl kind, std_msgs::ColorRGBA &color, const double alpha=0.5)
{
  if (kind == EControl::STOP)
  {
    color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = alpha;  // red
  }
  else if (kind == EControl::STOPLINE)
  {
    color.r = 0.0; color.g = 0.0; color.b = 1.0; color.a = alpha;  // blue
  }
  else if (kind == EControl::DECELERATE)
  {
    color.r = 1.0; color.g = 1.0; color.b = 0.0; color.a = alpha;  // yellow
  }
  else
  {
    color.r = 1.0; color.g = 1.0; color.b = 1.0; color.a = alpha;  // white
  }
}

// Display a detected obstacle
void displayObstacle(const EControl& kind, const ObstaclePoints& obstacle_points, const ros::Publisher& obstacle_pub)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  static geometry_msgs::Point prev_obstacle_point;
  if (kind == EControl::STOP || kind == EControl::STOPLINE || kind == EControl::DECELERATE)
  {
    marker.pose.position = obstacle_points.getObstaclePoint(kind);
    prev_obstacle_point = marker.pose.position;
  }
  else  // kind == OTHERS
  {
    marker.pose.position = prev_obstacle_point;
  }
  geometry_msgs::Quaternion quat;
  marker.pose.orientation = quat;

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 2.0;
  marker.lifetime = ros::Duration(0.1);
  marker.frame_locked = true;
  obstacleColorByKind(kind, marker.color, 0.7);

  obstacle_pub.publish(marker);
}

void displayDetectionRange(const aslan_msgs::Lane& lane, const int closest_waypoint,
                           const EControl& kind, const int obstacle_waypoint, const double stop_range,
                           const double deceleration_range, const ros::Publisher& detection_range_pub)
{
  // set up for marker array
  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker waypoint_marker_stop;
  visualization_msgs::Marker waypoint_marker_decelerate;
  visualization_msgs::Marker stop_line;
  waypoint_marker_stop.header.frame_id = "/map";
  waypoint_marker_stop.header.stamp = ros::Time();
  waypoint_marker_stop.id = 0;
  waypoint_marker_stop.type = visualization_msgs::Marker::SPHERE_LIST;
  waypoint_marker_stop.action = visualization_msgs::Marker::ADD;

  waypoint_marker_decelerate = waypoint_marker_stop;
  stop_line = waypoint_marker_stop;
  stop_line.type = visualization_msgs::Marker::CUBE;

  // set each namespace
  waypoint_marker_stop.ns = "Stop Detection";
  waypoint_marker_decelerate.ns = "Decelerate Detection";
  stop_line.ns = "Stop Line";

  // set scale and color
  double scale = 2 * stop_range;
  waypoint_marker_stop.scale.x = scale;
  waypoint_marker_stop.scale.y = scale;
  waypoint_marker_stop.scale.z = scale;
  waypoint_marker_stop.color.a = 0.2;
  waypoint_marker_stop.color.r = 0.0;
  waypoint_marker_stop.color.g = 1.0;
  waypoint_marker_stop.color.b = 0.0;
  waypoint_marker_stop.frame_locked = true;

  scale = 2 * (stop_range + deceleration_range);
  waypoint_marker_decelerate.scale.x = scale;
  waypoint_marker_decelerate.scale.y = scale;
  waypoint_marker_decelerate.scale.z = scale;
  waypoint_marker_decelerate.color.a = 0.15;
  waypoint_marker_decelerate.color.r = 1.0;
  waypoint_marker_decelerate.color.g = 1.0;
  waypoint_marker_decelerate.color.b = 0.0;
  waypoint_marker_decelerate.frame_locked = true;

  if (obstacle_waypoint > -1)
  {
    stop_line.pose.position = lane.waypoints[obstacle_waypoint].pose.pose.position;
    stop_line.pose.orientation = lane.waypoints[obstacle_waypoint].pose.pose.orientation;
  }
  stop_line.pose.position.z += 1.0;
  stop_line.scale.x = 0.1;
  stop_line.scale.y = 15.0;
  stop_line.scale.z = 2.0;
  stop_line.lifetime = ros::Duration(0.1);
  stop_line.frame_locked = true;
  obstacleColorByKind(kind, stop_line.color, 0.3);

  // set marker points coordinate
  for (int i = 0; i < STOP_SEARCH_DISTANCE; i++)
  {
    if (closest_waypoint < 0 || i + closest_waypoint > static_cast<int>(lane.waypoints.size()) - 1)
      break;

    geometry_msgs::Point point;
    point = lane.waypoints[closest_waypoint + i].pose.pose.position;

    waypoint_marker_stop.points.push_back(point);

    if (i > DECELERATION_SEARCH_DISTANCE)
      continue;
    waypoint_marker_decelerate.points.push_back(point);
  }

  // publish marker
  marker_array.markers.push_back(waypoint_marker_stop);
  marker_array.markers.push_back(waypoint_marker_decelerate);
  if (kind != EControl::KEEP)
    marker_array.markers.push_back(stop_line);
  detection_range_pub.publish(marker_array);
  marker_array.markers.clear();
}


int detectStopObstacle(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                       const aslan_msgs::Lane& lane, double stop_range,
                       double points_threshold, const geometry_msgs::PoseStamped& localizer_pose,
                       ObstaclePoints* obstacle_points, EObstacleType* obstacle_type,
                       const int wpidx_detection_result_by_other_nodes)
{
  int stop_obstacle_waypoint = -1;
  *obstacle_type = EObstacleType::NONE;
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + STOP_SEARCH_DISTANCE; i++)
  {
    // reach the end of waypoints
    if (i >= static_cast<int>(lane.waypoints.size()))
      break;

    // detection another nodes
    if (wpidx_detection_result_by_other_nodes >= 0 &&
        lane.waypoints.at(i).gid == wpidx_detection_result_by_other_nodes)
    {
      stop_obstacle_waypoint = i;
      *obstacle_type = EObstacleType::STOPLINE;
      obstacle_points->setStopPoint(lane.waypoints.at(i).pose.pose.position); // for vizuialization
      break;
    }

    // waypoint seen by localizer
    geometry_msgs::Point waypoint = calcRelativeCoordinate(lane.waypoints[i].pose.pose.position, localizer_pose.pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

    int stop_point_count = 0;
    for (const auto& p : points)
    {
      tf::Vector3 point_vector(p.x, p.y, 0);

      // 2D distance between waypoint and points (obstacle)
      double dt = tf::tfDistance(point_vector, tf_waypoint);
      if (dt < stop_range)
      {
        stop_point_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        obstacle_points->setStopPoint(calcAbsoluteCoordinate(point_temp, localizer_pose.pose));
      }
    }

    // there is an obstacle if the number of points exceeded the threshold
    if (stop_point_count >= points_threshold)
    {
      stop_obstacle_waypoint = i;
      *obstacle_type = EObstacleType::ON_WAYPOINTS;
      break;
    }

    obstacle_points->clearStopPoints();

    // check next waypoint...
  }

  return stop_obstacle_waypoint;
}

int detectDecelerateObstacle(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                             const aslan_msgs::Lane& lane, const double stop_range, const double deceleration_range,
                             const double points_threshold, const geometry_msgs::PoseStamped& localizer_pose,
                             ObstaclePoints* obstacle_points)
{
  int decelerate_obstacle_waypoint = -1;
  // start search from the closest waypoint
  for (int i = closest_waypoint; i < closest_waypoint + DECELERATION_SEARCH_DISTANCE; i++)
  {
    // reach the end of waypoints
    if (i >= static_cast<int>(lane.waypoints.size()))
      break;

    // waypoint seen by localizer
    geometry_msgs::Point waypoint = calcRelativeCoordinate(lane.waypoints[i].pose.pose.position, localizer_pose.pose);
    tf::Vector3 tf_waypoint = point2vector(waypoint);
    tf_waypoint.setZ(0);

    int decelerate_point_count = 0;
    for (const auto& p : points)
    {
      tf::Vector3 point_vector(p.x, p.y, 0);

      // 2D distance between waypoint and points (obstacle)
      double dt = tf::tfDistance(point_vector, tf_waypoint);
      if (dt > stop_range && dt < stop_range + deceleration_range)
      {
        decelerate_point_count++;
        geometry_msgs::Point point_temp;
        point_temp.x = p.x;
        point_temp.y = p.y;
        point_temp.z = p.z;
        obstacle_points->setDeceleratePoint(calcAbsoluteCoordinate(point_temp, localizer_pose.pose));
      }
    }

    // there is an obstacle if the number of points exceeded the threshold
    if (decelerate_point_count > points_threshold)
    {
      decelerate_obstacle_waypoint = i;
      break;
    }

    obstacle_points->clearDeceleratePoints();

    // check next waypoint...
  }

  return decelerate_obstacle_waypoint;
}

// Detect an obstacle by using pointcloud
EControl pointsDetection(const pcl::PointCloud<pcl::PointXYZ>& points, const int closest_waypoint,
                         const aslan_msgs::Lane& lane, const VelocitySetInfo& vs_info,
                         int* obstacle_waypoint, ObstaclePoints* obstacle_points, const double points_threshold)
{
  // no input for detection || no closest waypoint
  if ((points.empty() == true && vs_info.getDetectionResultByOtherNodes() == -1) || closest_waypoint < 0)
    return EControl::KEEP;

  EObstacleType obstacle_type = EObstacleType::NONE;

  int stop_obstacle_waypoint =
              detectStopObstacle(points, closest_waypoint, lane, vs_info.getStopRange(),
                                 points_threshold, vs_info.getLocalizerPose(),
                                 obstacle_points, &obstacle_type, vs_info.getDetectionResultByOtherNodes());

  // skip searching deceleration range
  if (vs_info.getDecelerationRange() < 0.01)
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    if (stop_obstacle_waypoint < 0)
      return EControl::KEEP;
    else if (obstacle_type == EObstacleType::ON_WAYPOINTS)
      return EControl::STOP;
    else if (obstacle_type == EObstacleType::STOPLINE)
      return EControl::STOPLINE;
    else
      return EControl::OTHERS;
  }
    int decelerate_obstacle_waypoint =
                detectDecelerateObstacle(points, closest_waypoint, lane, vs_info.getStopRange(), vs_info.getDecelerationRange(),
                                         points_threshold, vs_info.getLocalizerPose(), obstacle_points);

  // stop obstacle was not found
  if (stop_obstacle_waypoint < 0)
  {
    *obstacle_waypoint = decelerate_obstacle_waypoint;
    return decelerate_obstacle_waypoint < 0 ? EControl::KEEP : EControl::DECELERATE;
  }

  // stop obstacle was found but decelerate obstacle was not found
  if (decelerate_obstacle_waypoint < 0)
  {
      *obstacle_waypoint = stop_obstacle_waypoint;
    return EControl::STOP;
  }

  // about 5.0 meter
  double waypoint_interval =
      getPlaneDistance(lane.waypoints[0].pose.pose.position, lane.waypoints[1].pose.pose.position);
  int stop_decelerate_threshold = 5 / waypoint_interval;

    // both were found
  if (stop_obstacle_waypoint - decelerate_obstacle_waypoint > stop_decelerate_threshold)
  {
      *obstacle_waypoint = decelerate_obstacle_waypoint;
    return EControl::DECELERATE;
  }
  else
  {
    *obstacle_waypoint = stop_obstacle_waypoint;
    return EControl::STOP;
  }
}

EControl obstacleDetection(int closest_waypoint, const aslan_msgs::Lane& lane,
                           const VelocitySetInfo vs_info, const ros::Publisher& detection_range_pub,
                           const ros::Publisher& obstacle_pub, int* obstacle_waypoint)
{
  ObstaclePoints obstacle_points;
  std_msgs::Bool obstacle_removed_flag;

  EControl detection_result = pointsDetection(vs_info.getPoints(), closest_waypoint, lane, vs_info,
                                              obstacle_waypoint, &obstacle_points, vs_info.getPointsThreshold());

  EControl radar_detection_result = pointsDetection(vs_info.getRadarPoints(), closest_waypoint, lane, vs_info,
           obstacle_waypoint, &obstacle_points, vs_info.getRadarPointsThreshold());

  displayDetectionRange(lane, closest_waypoint, detection_result, *obstacle_waypoint, vs_info.getStopRange(),
                        vs_info.getDecelerationRange(), detection_range_pub);
  displayDetectionRange(lane, closest_waypoint, radar_detection_result, *obstacle_waypoint, vs_info.getStopRange(),
                          vs_info.getDecelerationRange(), detection_range_pub);

  static int false_count = 0;
  static EControl prev_detection = EControl::KEEP;
  static int prev_obstacle_waypoint = -1;
  std_msgs::Int32 emergency_stop;
  emergency_stop.data = -1;

  if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE || detection_result == EControl::DECELERATE)
  {
    displayObstacle(detection_result, obstacle_points, obstacle_pub);
    prev_detection = detection_result;
    false_count = 0;
    prev_obstacle_waypoint = *obstacle_waypoint;
    obstacle_removed_flag.data = false;
    obstacle_removed_pub.publish(obstacle_removed_flag);
    return detection_result;
  }

  if (radar_detection_result == EControl::STOP || radar_detection_result == EControl::STOPLINE || radar_detection_result == EControl::DECELERATE)
  {
      displayObstacle(radar_detection_result, obstacle_points, obstacle_pub);
      prev_detection = radar_detection_result;
      emergency_stop.data = 1;
      emergency_stop_pub.publish(emergency_stop);
      return radar_detection_result;
  }

  // there are no obstacles, but wait a little for safety
  if (prev_detection == EControl::STOP || prev_detection == EControl::STOPLINE || prev_detection == EControl::DECELERATE)
  {
    false_count++;

    if (false_count < LOOP_RATE / 2)
    {
      *obstacle_waypoint = prev_obstacle_waypoint;
      displayObstacle(EControl::OTHERS, obstacle_points, obstacle_pub);
      return prev_detection;
    }
  }

  // there are no obstacles, so we move forward
  *obstacle_waypoint = -1;
  false_count = 0;
  prev_detection = EControl::KEEP;

  obstacle_removed_flag.data = true;
  obstacle_removed_pub.publish(obstacle_removed_flag);
  return detection_result;
}

void changeWaypoints(const VelocitySetInfo& vs_info, const EControl& detection_result, int closest_waypoint,
                     int obstacle_waypoint, const ros::Publisher& final_waypoints_pub, VelocitySetPath* vs_path)
{

    if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE)
  {
	//The stop distance. If STOP, equales stop distance to obstacle, otherwise stop distance to stopline
    int stop_distance = (detection_result == EControl::STOP)
      ? vs_info.getStopDistanceObstacle() : vs_info.getStopDistanceStopline();
	  
	//deceleration set to stopline deceleration unless obstacle detected, then set to stopline
    double deceleration = (detection_result == EControl::STOP)
      ? vs_info.getDecelerationObstacle() : vs_info.getDecelerationStopline();
	  
	//Calculate the waypoint to stop at based on config
    int stop_waypoint =
        calcWaypointIndexReverse(vs_path->getPrevWaypoints(), obstacle_waypoint, stop_distance);
		
    vs_path->changeWaypointsForStopping(stop_waypoint, obstacle_waypoint, closest_waypoint, deceleration);
	vs_path->avoidSuddenAcceleration(vs_info.getAccelerateMax(), closest_waypoint);	
	vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), deceleration, closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
  else if (detection_result == EControl::DECELERATE)
  {
    vs_path->initializeNewWaypoints();
    vs_path->changeWaypointsForDeceleration(vs_info.getDecelerationObstacle(), closest_waypoint, obstacle_waypoint);
    vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->avoidSuddenAcceleration(vs_info.getAccelerateMax(), closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
  else
  {
    vs_path->initializeNewWaypoints();
    vs_path->avoidSuddenAcceleration(vs_info.getAccelerateMax(), closest_waypoint);
    vs_path->avoidSuddenDeceleration(vs_info.getVelocityChangeLimit(), vs_info.getDecelerationObstacle(), closest_waypoint);
    vs_path->setTemporalWaypoints(vs_info.getTemporalWaypointsSize(), closest_waypoint, vs_info.getControlPose());
    final_waypoints_pub.publish(vs_path->getTemporalWaypoints());
  }
}

}  // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "velocity_set");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  bool enablePlannerDynamicSwitch;

  std::string lidar_points_topic;
  std::string radar_points_topic;
  std::string vehicle_speed_topic;

  private_nh.param<bool>("enablePlannerDynamicSwitch", enablePlannerDynamicSwitch, false);
  private_nh.param<std::string>("lidar_points_topic", lidar_points_topic, "points_lanes");
  private_nh.param<std::string>("radar_points_topic", radar_points_topic, "radar/target_list_cartesian");
  private_nh.param<std::string>("vehicle_speed_topic", vehicle_speed_topic, "sd_current_twist");


  // class
  VelocitySetPath vs_path;
  VelocitySetInfo vs_info;

  // velocity set subscriber
  ros::Subscriber waypoints_sub = nh.subscribe("safety_waypoints", 1, &VelocitySetPath::waypointsCallback, &vs_path);
  ros::Subscriber current_vel_sub =
      nh.subscribe(vehicle_speed_topic, 1, &VelocitySetPath::currentVelocityCallback, &vs_path);

  // velocity set info subscriber
  vs_info.InitParam(private_nh);
  ros::Subscriber points_sub = nh.subscribe(lidar_points_topic, 1, &VelocitySetInfo::pointsCallback, &vs_info);
  ros::Subscriber radar_points_sub = nh.subscribe(radar_points_topic, 1, &VelocitySetInfo::radar_pointsCallback, &vs_info);

  ros::Subscriber localizer_sub = nh.subscribe("localizer_pose", 1, &VelocitySetInfo::localizerPoseCallback, &vs_info);
  ros::Subscriber control_pose_sub = nh.subscribe("current_pose", 1, &VelocitySetInfo::controlPoseCallback, &vs_info);
  ros::Subscriber obstacle_sim_points_sub = nh.subscribe("obstacle_sim_pointcloud", 1, &VelocitySetInfo::obstacleSimCallback, &vs_info);
  ros::Subscriber detectionresult_sub = nh.subscribe("/state/stopline_wpidx", 1, &VelocitySetInfo::detectionCallback, &vs_info);
  ros::Subscriber param_sub = nh.subscribe("config/velocity_set", 1, &VelocitySetInfo::param_callback, &vs_info);

  // publisher
  ros::Publisher detection_range_pub = nh.advertise<visualization_msgs::MarkerArray>("detection_range", 1);
  ros::Publisher obstacle_pub = nh.advertise<visualization_msgs::Marker>("obstacle", 1);
  ros::Publisher obstacle_waypoint_pub = nh.advertise<std_msgs::Int32>("obstacle_waypoint", 1, true);
  emergency_stop_pub = nh.advertise<std_msgs::Int32>("radar_emergency_stop", 100, true);
  ros::Publisher detected_obst_pub = nh.advertise<std_msgs::Bool>("obstacle_detected_stop", 100, true);
  obstacle_removed_pub = nh.advertise<std_msgs::Bool>("obstacle_removed", 100, true);
  ros::Publisher final_waypoints_pub = nh.advertise<aslan_msgs::Lane>("final_waypoints", 100, true);

  ros::Rate loop_rate(LOOP_RATE);
  while (ros::ok())
  {
    ros::spinOnce();

    int closest_waypoint = 0;

    if (!vs_info.getSetPose() || !vs_path.getSetPath())
    {
      loop_rate.sleep();
      continue;
    }


    int obstacle_waypoint = -1;
    EControl detection_result = obstacleDetection(closest_waypoint, vs_path.getPrevWaypoints(), vs_info,
                                                  detection_range_pub, obstacle_pub, &obstacle_waypoint);

    std_msgs::Bool obstacle_flag;

    if (detection_result == EControl::STOP || detection_result == EControl::STOPLINE || detection_result == EControl::DECELERATE){
        obstacle_flag.data = true;
        detected_obst_pub.publish(obstacle_flag);
    }else{
        obstacle_flag.data = false;
        detected_obst_pub.publish(obstacle_flag);
    }

    changeWaypoints(vs_info, detection_result, closest_waypoint,
                    obstacle_waypoint, final_waypoints_pub, &vs_path);

    vs_info.clearPoints();

    // publish obstacle waypoint index
    std_msgs::Int32 obstacle_waypoint_index;
    obstacle_waypoint_index.data = obstacle_waypoint;

    vs_path.resetFlag();
    loop_rate.sleep();
  }
  return 0;
}
