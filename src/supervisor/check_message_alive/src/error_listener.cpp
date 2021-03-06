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


#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include "error_listener.h"

void error_listener::diagnostics_callback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg)
{
    for(int topic = 0; topic < (msg->status).size(); topic++)
    {
        if(msg->status[topic].level == 2)
        {
            ROS_ERROR("Error in %s", msg->status[topic].name.c_str());
        }
    }
}

int error_listener::RUN(int argc, char **argv)
{
    ros::init(argc, argv, "read_diagnostics");

    ros::NodeHandle nh("~");
    ros::NodeHandle private_nh("~");
    
    ros::Subscriber aslan_diagnostics_sub = nh.subscribe<diagnostic_msgs::DiagnosticArray>("/aslan_diagnostics", 1, &error_listener::diagnostics_callback, this);
    while(ros::ok()) ros::spinOnce();
}