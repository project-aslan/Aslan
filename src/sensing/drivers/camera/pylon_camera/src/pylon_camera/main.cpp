/******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (C) 2016, Magazino GmbH. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Magazino GmbH nor the names of its
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
 *****************************************************************************/

/* Authors: debout@magazino.eu
 *          grimm@magazino.eu
 *          engelhard@magazino.eu
 */

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <pylon_camera/pylon_camera_node.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pylon_camera_node");

    pylon_camera::PylonCameraNode pylon_camera_node;

    ros::Rate r(pylon_camera_node.frameRate());

    ROS_INFO_STREAM("Start image grabbing if node connects to topic with "
        << "a frame_rate of: " << pylon_camera_node.frameRate() << " Hz");

    // Main thread and brightness-service thread
    boost::thread th(boost::bind(&ros::spin));

    while ( ros::ok() )
    {
        pylon_camera_node.spin();
        r.sleep();
    }

    ROS_INFO("Terminate PylonCameraNode");
    return EXIT_SUCCESS;
}
