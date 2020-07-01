/*
 * Copyright (C) 2020 Project ASLAN - All rights reserved
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

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>

#include <ros/ros.h>
#include <aslan_msgs/ConfigNDTMappingOutput.h>

static ros::Publisher pcd_saver_pub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_pcd");

    ros::NodeHandle nh("~");
    ros::NodeHandle private_nh("~");

    pcd_saver_pub = nh.advertise<aslan_msgs::ConfigNDTMappingOutput>("/config/ndt_mapping_output", 1000);
    aslan_msgs::ConfigNDTMappingOutput pcd_saver_msg;

    std::string filename;
    nh.getParam("filename", filename);
    ROS_INFO("Got parameter : %s", filename.c_str());
    pcd_saver_msg.filename = filename;
    pcd_saver_msg.filter_res = 0.2;

    sleep(1);
    pcd_saver_pub.publish(pcd_saver_msg);

    return 0;
}
