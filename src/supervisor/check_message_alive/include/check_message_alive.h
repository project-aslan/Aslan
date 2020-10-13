/*
 * Copyright (C) 2020 StreetDrone Limited - All rights reserved
 * 
 * Author: Abdelrahman Barghouth
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "velocity_set_path.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <string.h>
#include <time.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <aslan_msgs/Lane.h>
#include <aslan_msgs/LaneArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_msgs/TFMessage.h>
#include <chrono>

class topic_data
{
	public:
		time_t topic_time;
		std::string topic_name;
		bool topic_alive;
		bool check_message;
		bool correct_message;

		topic_data(std::string name, bool check_message);
};

class check_message_alive
{
public:
	void pmap_stat_alive_callback(const std_msgs::Bool::ConstPtr& msg);
	void filtered_points_alive_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void ndt_pose_alive_callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void lane_array_alive_callback(const aslan_msgs::LaneArray::ConstPtr& msg);
	void twist_cmd_alive_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void twist_raw_alive_callback(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void points_raw_alive_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
	void traffic_array_alive_callback(const aslan_msgs::LaneArray::ConstPtr& msg);
	void closest_alive_callback(const std_msgs::Int32ConstPtr& msg, VelocitySetPath vs_path);
	void final_waypoints_callback(const aslan_msgs::LaneConstPtr& msg);
	void current_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg);
	void safety_waypoints_callback(const aslan_msgs::LaneConstPtr& msg);
	void tf_callback(const tf2_msgs::TFMessageConstPtr &msg);
	void radar_emergency_callback(const std_msgs::Int32::ConstPtr& msg);
	diagnostic_msgs::DiagnosticStatus get_topic_diagnostics(topic_data topic);
	diagnostic_msgs::DiagnosticArray generate_diagnostics();
	int RUN(int argc, char **argv);

	diagnostic_msgs::DiagnosticArray aslan_diagnostics_array;
	diagnostic_msgs::DiagnosticStatus aslan_diagnostics_status;
	int seq = 0;
	bool tf_msg1 = true, tf_msg2 = true, tf_msg3 = true;
	bool emergency_flag = false;
	bool end_of_waypoints = false;
	// void waypointsCallback(const aslan_msgs::LaneConstPtr& msg);

	int FIXED_ERROR_THRESH = 200; // 200 ms
};