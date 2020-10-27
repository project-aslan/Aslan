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

#include "check_message_alive.h"

topic_data::topic_data(std::string name, bool check_message)
{
	topic_alive = false;
	correct_message = true;
	topic_name = name;
	check_message = check_message;
}

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

void check_message_alive::pmap_stat_alive_callback(const std_msgs::Bool::ConstPtr& msg)
{
	pmap_stat.topic_alive = true;
	pmap_stat.topic_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void check_message_alive::filtered_points_alive_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	filtered_points.topic_alive = true;
	filtered_points.topic_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void check_message_alive::ndt_pose_alive_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	ndt_pose.topic_alive = true;
	ndt_pose.topic_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void check_message_alive::lane_array_alive_callback(const aslan_msgs::LaneArray::ConstPtr& msg)
{
	lane_waypoints_array.topic_alive = true;
	lane_waypoints_array.topic_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void check_message_alive::twist_cmd_alive_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	twist_cmd.topic_alive = true;
	twist_cmd.topic_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	if(msg->twist.linear.x == 0 && msg->twist.angular.z == 0)
	{
		if(check_message_alive::emergency_flag || check_message_alive::end_of_waypoints)
		{
			twist_cmd.correct_message = true;
		}
		else
		{
			twist_cmd.correct_message = false;
		}
	}
	else
	{
		twist_cmd.correct_message = true;
	}
}

void check_message_alive::twist_raw_alive_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	twist_raw.topic_alive = true;
	twist_raw.topic_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void check_message_alive::points_raw_alive_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	points_raw.topic_alive = true;
	points_raw.topic_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	pcl::PointCloud<pcl::PointXYZ> _points;
	pcl::fromROSMsg(*msg, _points);
	if(_points.empty())
	{
		points_raw.correct_message = false;
	}
	else
	{
		points_raw.correct_message = true;
	}
}

void check_message_alive::traffic_array_alive_callback(const aslan_msgs::LaneArray::ConstPtr& msg)
{
	traffic_waypoints_array.topic_alive = true;
	traffic_waypoints_array.topic_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void check_message_alive::closest_alive_callback(const std_msgs::Int32ConstPtr& msg, VelocitySetPath vs_path)
{
	closest_waypoint.topic_alive = true;
	closest_waypoint.topic_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	aslan_msgs::Lane lane = vs_path.getPrevWaypoints();
	if(msg->data >= static_cast<int>(lane.waypoints.size()))
		check_message_alive::end_of_waypoints = true;
}

void check_message_alive::final_waypoints_callback(const aslan_msgs::LaneConstPtr& msg)
{
	final_waypoints.topic_alive = true;
	final_waypoints.topic_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void check_message_alive::current_pose_callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
	current_pose.topic_alive = true;
	current_pose.topic_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void check_message_alive::safety_waypoints_callback(const aslan_msgs::LaneConstPtr& msg)
{
	safety_waypoints.topic_alive = true;
	safety_waypoints.topic_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void check_message_alive::tf_callback(const tf2_msgs::TFMessageConstPtr &msg)
{
	transf.topic_alive = true;
	transf.topic_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

	for(int i = 0; i < msg->transforms.size(); i++)
	{
		if(msg->transforms[i].child_frame_id.compare("/base_link") == 0)
		{
			if(msg->transforms[i].header.frame_id.compare("/map") == 0)
			{
				check_message_alive::tf_msg1 = true;
			}
			else
			{
				check_message_alive::tf_msg1 = false;
			}
		}
		else if(msg->transforms[i].child_frame_id.compare("/velodyne") == 0)
		{
			if(msg->transforms[i].header.frame_id.compare("/base_link") == 0)
			{
				check_message_alive::tf_msg2 = true;
			}
			else
			{
				check_message_alive::tf_msg2 = false;
			}
		}
		else if(msg->transforms[i].child_frame_id.compare("/map") == 0)
		{
			if(msg->transforms[i].header.frame_id.compare("/world") == 0)
			{
				check_message_alive::tf_msg3 = true;
			}
			else
			{
				check_message_alive::tf_msg3 = false;
			}
		}
	}

	if(check_message_alive::tf_msg1 && check_message_alive::tf_msg2 && check_message_alive::tf_msg3)
	{
		transf.correct_message = true;
	}
	else
	{
		transf.correct_message = false;
	}

}

void check_message_alive::radar_emergency_callback(const std_msgs::Int32::ConstPtr& msg)
{
	if(msg->data == 1)
		check_message_alive::emergency_flag = true;
}

diagnostic_msgs::DiagnosticStatus check_message_alive::get_topic_diagnostics(topic_data topic)
{
	check_message_alive::aslan_diagnostics_status.name = topic.topic_name + " topic";

	if(topic.topic_alive)
	{
		// if (topic.topic_name.compare("points_raw") == 0) std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() <<std::endl;
		if(std::abs(topic.topic_time - std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count()) > check_message_alive::FIXED_ERROR_THRESH && topic.topic_name.compare("tf") != 0)
		{
			ROS_ERROR("%s is not publishing", topic.topic_name.c_str());
			check_message_alive::aslan_diagnostics_status.level = 2;
			check_message_alive::aslan_diagnostics_status.message = "ERROR";
		}
		else
		{
			if(topic.check_message && !topic.correct_message)
			{
				ROS_ERROR("%s has wrong DATA", topic.topic_name.c_str());
				check_message_alive::aslan_diagnostics_status.level = 2;
				check_message_alive::aslan_diagnostics_status.message = "PUBLISHED WRONG DATA";
			}
			else
			{
				check_message_alive::aslan_diagnostics_status.level = 0;
				check_message_alive::aslan_diagnostics_status.message = "ALIVE AND PUBLISHING";
			}
		}
	}
	else
	{
		check_message_alive::aslan_diagnostics_status.level = 0;
		check_message_alive::aslan_diagnostics_status.message = "DIDN'T START";
	}
	return check_message_alive::aslan_diagnostics_status;
}

diagnostic_msgs::DiagnosticArray check_message_alive::generate_diagnostics()
{
	check_message_alive::aslan_diagnostics_array.status.clear();
	int topic_number = 0;
	topic_data topics[] = {pmap_stat, filtered_points, ndt_pose, lane_waypoints_array, twist_cmd, twist_raw, points_raw, traffic_waypoints_array, 
	closest_waypoint, final_waypoints, current_pose, safety_waypoints, transf};
	for(const topic_data &topic : topics)
	{
		check_message_alive::aslan_diagnostics_status = get_topic_diagnostics(topic);
		check_message_alive::aslan_diagnostics_array.status.push_back(check_message_alive::aslan_diagnostics_status);
	}
	return check_message_alive::aslan_diagnostics_array;
}

int check_message_alive::RUN(int argc, char **argv)
{
    ros::init(argc, argv, "check_message_alive");

    ros::NodeHandle nh("~");
    ros::NodeHandle private_nh("~");
    VelocitySetPath vs_path;

    ros::Subscriber pmap_stat_alive_sub = nh.subscribe<std_msgs::Bool>("/pmap_stat", 100, &check_message_alive::pmap_stat_alive_callback, this);
    ros::Subscriber filtered_points_alive_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_points", 100, &check_message_alive::filtered_points_alive_callback, this);
    ros::Subscriber ndt_pose_alive_sub = nh.subscribe("/ndt_pose", 10, &check_message_alive::ndt_pose_alive_callback, this);
    ros::Subscriber lane_array_alive_sub = nh.subscribe("/lane_waypoints_array", 10, &check_message_alive::lane_array_alive_callback, this);
    ros::Subscriber twist_cmd_alive_sub = nh.subscribe("/twist_cmd", 1, &check_message_alive::twist_cmd_alive_callback, this);
    ros::Subscriber twist_raw_alive_sub = nh.subscribe("/twist_raw", 1, &check_message_alive::twist_raw_alive_callback, this);
    ros::Subscriber points_raw_alive_sub = nh.subscribe("/points_raw", 10, &check_message_alive::points_raw_alive_callback, this);
    ros::Subscriber traffic_array_alive_sub = nh.subscribe("/traffic_waypoints_array", 10, &check_message_alive::traffic_array_alive_callback, this);
    ros::Subscriber closest_waypoint_alive_sub = nh.subscribe<std_msgs::Int32>("/closest_waypoint", 10, boost::bind(&check_message_alive::closest_alive_callback, this, _1, vs_path));
    ros::Subscriber final_waypoint_alive_sub = nh.subscribe("/final_waypoints", 10, &check_message_alive::final_waypoints_callback, this);
    ros::Subscriber current_pose_alive_sub = nh.subscribe("/current_pose", 10, &check_message_alive::current_pose_callback, this);
    ros::Subscriber safety_waypoints_alive_sub = nh.subscribe("/safety_waypoints", 1, &check_message_alive::safety_waypoints_callback, this);
    ros::Subscriber waypoints_sub = nh.subscribe("/safety_waypoints", 1, &VelocitySetPath::waypointsCallback, &vs_path);
    ros::Subscriber tf_alive_sub = nh.subscribe("/tf", 10, &check_message_alive::tf_callback, this);
    ros::Subscriber radar_emergency_sub = nh.subscribe<std_msgs::Int32>("/radar_emergency_stop", 100, &check_message_alive::radar_emergency_callback, this);
    ros::Publisher aslan_diagnostics_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/aslan_diagnostics", 1);

    while(ros::ok())
    {
    	check_message_alive::aslan_diagnostics_array.header.stamp = ros::Time::now();
    	check_message_alive::aslan_diagnostics_array.header.seq = check_message_alive::seq++;
    	check_message_alive::aslan_diagnostics_array = generate_diagnostics();
    	aslan_diagnostics_pub.publish(check_message_alive::aslan_diagnostics_array);
    	ros::spinOnce();
    }
}
