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


diagnostic_msgs::DiagnosticArray aslan_diagnostics_array;
diagnostic_msgs::DiagnosticStatus aslan_diagnostics_status;
int seq = 0;
bool tf_msg1 = true, tf_msg2 = true, tf_msg3 = true;
bool emergency_flag = false;
bool end_of_waypoints = false;
void waypointsCallback(const aslan_msgs::LaneConstPtr& msg);


topic_data pmap_stat ("pmap_stat", false);
topic_data filtered_points ("filtered_points", false);
topic_data ndt_pose ("ndt_pose", false);
topic_data lane_waypoints_array ("lane_waypoints_array", false);
topic_data twist_cmd ("twist_cmd", true);
topic_data twist_raw ("twist_raw", false);
topic_data points_raw ("points_raw", true);
topic_data traffic_waypoints_array ("traffic_waypoints_array", false);
topic_data closest_waypoint ("closest_waypoint", false);
topic_data final_waypoints ("final_waypoints", false);
topic_data current_pose ("current_pose", false);
topic_data safety_waypoints ("safety_waypoints", false);
topic_data transf ("tf", true);



int FIXED_ERROR_THRESH = 200; // 200 ms