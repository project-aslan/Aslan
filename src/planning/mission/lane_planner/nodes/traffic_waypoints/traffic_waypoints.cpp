/*
 * Copyright (C) 2020 Project ASLAN - All rights reserved
 *
 * Author: Efimia Panagiotaki
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

// #define DEBUG

#ifdef DEBUG
#include <sstream>
#endif // DEBUG

#include <ros/console.h>
#include <ros/ros.h>

#include "aslan_msgs/ConfigLaneRule.h"
#include "aslan_msgs/LaneArray.h"

namespace {

std::string frame_id;

ros::Publisher traffic_pub;

aslan_msgs::LaneArray cached_waypoint;

#ifdef DEBUG
visualization_msgs::Marker debug_marker;
ros::Publisher marker_pub;
int marker_cnt;
#endif // DEBUG

aslan_msgs::Lane create_new_lane(const aslan_msgs::Lane& lane, const std_msgs::Header& header)
{
	aslan_msgs::Lane l = lane;
	l.header = header;

	for (aslan_msgs::Waypoint& w : l.waypoints) {
		w.pose.header = header;
		w.twist.header = header;
	}

	return l;
}

#ifdef DEBUG
std_msgs::ColorRGBA create_color(int index)
{
	std_msgs::ColorRGBA color;
	switch (index) {
	case 0:
		color.r = 0;
		color.g = 0;
		color.b = 0;
		break;
	case 1:
		color.r = 0;
		color.g = 0;
		color.b = 1;
		break;
	case 2:
		color.r = 0;
		color.g = 1;
		color.b = 0;
		break;
	case 3:
		color.r = 0;
		color.g = 1;
		color.b = 1;
		break;
	case 4:
		color.r = 1;
		color.g = 0;
		color.b = 0;
		break;
	case 5:
		color.r = 1;
		color.g = 0;
		color.b = 1;
		break;
	case 6:
		color.r = 1;
		color.g = 1;
		color.b = 0;
		break;
	default:
		color.r = 1;
		color.g = 1;
		color.b = 1;
	}
	color.a = 1;

	return color;
}
#endif // DEBUG

void create_waypoint(const aslan_msgs::LaneArray& msg) {
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id;

    cached_waypoint.lanes.clear();
    cached_waypoint.lanes.shrink_to_fit();
    for (const aslan_msgs::Lane &l : msg.lanes)
        cached_waypoint.lanes.push_back(create_new_lane(l, header));

    traffic_pub.publish(cached_waypoint);
    return;

}


} // namespace

int main(int argc, char **argv)
{
	ros::init(argc, argv, "traffic_waypoints");

	ros::NodeHandle n;

	int sub_waypoint_queue_size;
	n.param<int>("/traffic_waypoints/sub_waypoint_queue_size", sub_waypoint_queue_size, 1);
	int pub_waypoint_queue_size;
	n.param<int>("/traffic_waypoints/pub_waypoint_queue_size", pub_waypoint_queue_size, 1);
	bool pub_waypoint_latch;
	n.param<bool>("/traffic_waypoints/pub_waypoint_latch", pub_waypoint_latch, true);
#ifdef DEBUG
	int pub_marker_queue_size;
	n.param<int>("/lane_rule/pub_marker_queue_size", pub_marker_queue_size, 10);
	bool pub_marker_latch;
	n.param<bool>("/lane_rule/pub_marker_latch", pub_marker_latch, true);
#endif // DEBUG
	n.param<std::string>("/traffic_waypoints/frame_id", frame_id, "map");

	traffic_pub = n.advertise<aslan_msgs::LaneArray>("/traffic_waypoints_array", pub_waypoint_queue_size,
								pub_waypoint_latch);

	ros::Subscriber waypoint_sub = n.subscribe("/lane_waypoints_array", sub_waypoint_queue_size, create_waypoint);

	ros::spin();

	return 0;
}
