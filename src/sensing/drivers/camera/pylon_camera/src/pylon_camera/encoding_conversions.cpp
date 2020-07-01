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

#include <ros/ros.h>
#include <pylon_camera/encoding_conversions.h>
#include <sensor_msgs/image_encodings.h>

namespace pylon_camera
{

namespace encoding_conversions
{

bool ros2GenAPI(const std::string& ros_enc, std::string& gen_api_enc)
{
    /*
     * http://docs.ros.org/kinetic/api/sensor_msgs/html/image__encodings_8h_source.html
     */
    if ( ros_enc == sensor_msgs::image_encodings::MONO8 )
    {
        gen_api_enc = "Mono8";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BGR8 )
    {
        gen_api_enc = "BGR8";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::RGB8 )
    {
        gen_api_enc = "RGB8";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_BGGR8 )
    {
        gen_api_enc = "BayerBG8";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_GBRG8 )
    {
        gen_api_enc = "BayerGB8";
    }
    else if ( ros_enc == sensor_msgs::image_encodings::BAYER_RGGB8 )
    {
        gen_api_enc = "BayerRG8";
    }
    /*
    else if ( ros_enc == sensor_msgs::image_encodings::YUV422 )
    {
        //  This is the UYVY version of YUV422 codec http://www.fourcc.org/yuv.php#UYVY
        //  with an 8-bit depth
        gen_api_enc = "YCbCr422_8";
    }
    */
    else
    {
        /* No gen-api pendant existant for following ROS-encodings:
         * - sensor_msgs::image_encodings::MONO16
         * - sensor_msgs::image_encodings::BGRA8
         * - sensor_msgs::image_encodings::BGR16
         * - sensor_msgs::image_encodings::BGRA16
         * - sensor_msgs::image_encodings::RGBA8
         * - sensor_msgs::image_encodings::RGB16
         * - sensor_msgs::image_encodings::RGBA16
         * - sensor_msgs::image_encodings::BAYER_BGGR16
         * - sensor_msgs::image_encodings::BAYER_GBRG16
         * - sensor_msgs::image_encodings::BAYER_GRBG16
         * - sensor_msgs::image_encodings::BAYER_GRBG8
         * - sensor_msgs::image_encodings::YUV422
         */
        return false;
    }
    return true;
}

bool genAPI2Ros(const std::string& gen_api_enc, std::string& ros_enc)
{
    if ( gen_api_enc == "Mono8" )
    {
        ros_enc = sensor_msgs::image_encodings::MONO8;
    }
    else if ( gen_api_enc == "BGR8" )
    {
        ros_enc = sensor_msgs::image_encodings::BGR8;
    }
    else if ( gen_api_enc == "RGB8" )
    {
        ros_enc = sensor_msgs::image_encodings::RGB8;
    }
    else if ( gen_api_enc == "BayerBG8" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_BGGR8;
    }
    else if ( gen_api_enc == "BayerGB8" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_GBRG8;
    }
    else if ( gen_api_enc == "BayerRG8" )
    {
        ros_enc = sensor_msgs::image_encodings::BAYER_RGGB8;
    }
    /*
    else if ( gen_api_enc == "YCbCr422_8" )
    {
        ros_enc = sensor_msgs::image_encodings::YUV422;
    }
    */
    else
    {
        /* Unsupported are:
         * - Mono10
         * - Mono10p
         * - Mono12
         * - Mono12p
         * - BayerGR10
         * - BayerGR10p
         * - BayerRG10
         * - BayerRG10p
         * - BayerGB10
         * - BayerGB10p
         * - BayerBG10
         * - BayerBG10p
         * - BayerGR12
         * - BayerGR12p
         * - BayerRG12
         * - BayerRG12p
         * - BayerGB12
         * - BayerGB12p
         * - BayerBG12
         * - BayerBG12p
         * - YCbCr422_8
         */
        return false;
    }
    return true;
}
}  // namespace encoding_conversions
}  // namespace pylon_camera
