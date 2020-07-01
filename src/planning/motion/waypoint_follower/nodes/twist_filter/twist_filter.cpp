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
#include <geometry_msgs/TwistStamped.h>
#include "std_msgs/Bool.h"

#include <iostream>

#include "aslan_msgs/ConfigTwistFilter.h"

namespace {

//Publisher
ros::Publisher g_twist_pub;
double g_lateral_accel_limit = 10.0;
double g_lowpass_gain_linear_x = 0.0;
double g_lowpass_gain_angular_z = 0.0;
constexpr double RADIUS_MAX = 9e10;
constexpr double ERROR = 1e-8;

int zero_twist_counter = 0;
std_msgs::Bool obst_removed;

void removedCallback(const std_msgs::Bool::ConstPtr& msg){
    obst_removed.data = msg->data;
    if (obst_removed.data == true){
        zero_twist_counter = 0;
    }
}

void TwistCmdCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{

  double v = msg->twist.linear.x;
  double omega = msg->twist.angular.z;

  if (v == 0.0){
      zero_twist_counter = zero_twist_counter +1;
  }

  if (zero_twist_counter > 0 ){
      v = 0.0;
  }

  if(fabs(omega) < ERROR){
    g_twist_pub.publish(*msg);
    return;
  }

  double max_v = g_lateral_accel_limit / omega;

  geometry_msgs::TwistStamped tp;
  tp.header = msg->header;

  double a = v * omega;
  ROS_INFO("lateral accel = %lf", a);


  tp.twist.linear.x = fabs(a) > g_lateral_accel_limit ? max_v
                    : v;
  tp.twist.angular.z = omega;

  static double lowpass_linear_x = 0;
  static double lowpass_angular_z = 0;
  lowpass_linear_x = g_lowpass_gain_linear_x * lowpass_linear_x + (1 - g_lowpass_gain_linear_x) * tp.twist.linear.x;
  lowpass_angular_z = g_lowpass_gain_angular_z * lowpass_angular_z + (1 - g_lowpass_gain_angular_z) * tp.twist.angular.z;

  tp.twist.linear.x = lowpass_linear_x;
  tp.twist.angular.z = lowpass_angular_z;

    if (zero_twist_counter > 0 ){
        tp.twist.linear.x = 0.0;
    }

  ROS_INFO("v: %f -> %f",v,tp.twist.linear.x);
  g_twist_pub.publish(tp);

}
} // namespace


void param_callback(const aslan_msgs::ConfigTwistFilter::ConstPtr& config)
{
    g_lateral_accel_limit = config->lateral_accel_limit;
    ROS_INFO("g_lateral_accel_limit = %lf",g_lateral_accel_limit);
    g_lowpass_gain_linear_x = config->lowpass_gain_linear_x;
    ROS_INFO("lowpass_gain_linear_x = %lf",g_lowpass_gain_linear_x);
    g_lowpass_gain_angular_z = config->lowpass_gain_angular_z;
    ROS_INFO("lowpass_gain_angular_z = %lf",g_lowpass_gain_angular_z);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "twist_filter");

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    ros::Subscriber obstacle_removed_sub = nh.subscribe("obstacle_removed", 100, removedCallback);
    ros::Subscriber twist_sub = nh.subscribe("twist_raw", 1, TwistCmdCallback);
    ros::Subscriber param_sub = nh.subscribe("config/twist_filter", 10, param_callback);
    g_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist_cmd", 1000);

    ros::spin();
    return 0;
}
