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

#include <pylon_camera/internal/pylon_camera.h>
#include <string>
#include <vector>

namespace pylon_camera
{

enum PYLON_CAM_TYPE
{
    GIGE = 1,
    USB = 2,
    DART = 3,
    UNKNOWN = -1,
};

PylonCamera::PylonCamera()
    : device_user_id_("")
    , img_rows_(0)
    , img_cols_(0)
    , img_size_byte_(0)
    , grab_timeout_(-1.0)
    , is_ready_(false)
    , is_binary_exposure_search_running_(false)
    , max_brightness_tolerance_(2.5)
    , binary_exp_search_(nullptr)
{}

PYLON_CAM_TYPE detectPylonCamType(const Pylon::CDeviceInfo& device_info)
{
    Pylon::String_t device_class;
    if ( device_info.IsDeviceClassAvailable() )
    {
        device_class = device_info.GetDeviceClass();
        if ( device_class == "BaslerGigE" )
        {
            return GIGE;
        }
        else if ( device_class == "BaslerUsb" )
        {
            if ( device_info.IsModelNameAvailable() )
            {
                std::string model_name(device_info.GetModelName());
                if ( model_name.compare(0, 3, "acA") == 0 )
                {
                    return USB;
                }
                else if ( model_name.compare(0, 3, "daA") == 0 )
                {
                    return DART;
                }
                else
                {
                    ROS_ERROR_STREAM("Found 'BaslerUsb'-Camera, but it is neither "
                        << "a Dart, nor a USB-Camera. Up to now, other cameras are "
                        << "not supported by this pkg!");
                    return UNKNOWN;
                }
            }
            else
            {
                ROS_ERROR_STREAM("Error while detecting the pylon camera type from "
                << "its ModelName: Camera has no ModelName available!");
                return UNKNOWN;
            }
        }
        else
        {
            ROS_ERROR_STREAM("Detected Camera Type is neither 'BaslerUsb', nor "
                << "'BaslerGigE'. Up to now, other cameras not supported by "
                << "this pkg!");
            return UNKNOWN;
        }
    }
    else
    {
        ROS_ERROR_STREAM("Error while detecting the pylon camera type from "
                << "its DeviceClass: Camera has no DeviceClass available!");
        return UNKNOWN;
    }
}

PylonCamera* createFromDevice(PYLON_CAM_TYPE cam_type, Pylon::IPylonDevice* device)
{
    switch ( cam_type )
    {
        case GIGE:
            return new PylonGigECamera(device);
        case USB:
            return new PylonUSBCamera(device);
        case DART:
            return new PylonDARTCamera(device);
        case UNKNOWN:
        default:
            return nullptr;
    }
}

PylonCamera* PylonCamera::create(const std::string& device_user_id_to_open)
{
    try
    {
        // Before using any pylon methods, the pylon runtime must be initialized.
        Pylon::PylonInitialize();

        Pylon::CTlFactory& tl_factory = Pylon::CTlFactory::GetInstance();
        Pylon::DeviceInfoList_t device_list;

        // EnumerateDevices() returns the number of devices found
        if ( 0 == tl_factory.EnumerateDevices(device_list) )
        {
            Pylon::PylonTerminate();
            ROS_ERROR_ONCE("No camera present");
            return nullptr;
        }
        else
        {
            Pylon::DeviceInfoList_t::const_iterator it;
            if ( device_user_id_to_open.empty() )
            {
		for (it = device_list.begin(); it != device_list.end(); ++it)
		{
		    ROS_INFO_STREAM("Found camera with DeviceUserID "
				    << it->GetUserDefinedName() << ": "
				    << it->GetModelName());
		    PYLON_CAM_TYPE cam_type = detectPylonCamType(*it);
		    if (cam_type != UNKNOWN)
		    {
		      PylonCamera* new_cam_ptr = createFromDevice(cam_type,
					                          tl_factory.CreateDevice(*it));
		      new_cam_ptr->device_user_id_ = it->GetUserDefinedName();
		      return new_cam_ptr;
		    }
		}
		Pylon::PylonTerminate();
		ROS_ERROR_ONCE("No compatible camera present");
		return nullptr;
            }
            bool found_desired_device = false;
            for ( it = device_list.begin(); it != device_list.end(); ++it )
            {
                std::string device_user_id_found(it->GetUserDefinedName());
                if ( (0 == device_user_id_to_open.compare(device_user_id_found)) ||
                     (device_user_id_to_open.length() < device_user_id_found.length() &&
                     (0 == device_user_id_found.compare(device_user_id_found.length() -
                                                         device_user_id_to_open.length(),
                                                         device_user_id_to_open.length(),
                                                         device_user_id_to_open) )
                     )
                   )
                {
                    found_desired_device = true;
                    break;
                }
            }
            if ( found_desired_device )
            {
                ROS_INFO_STREAM("Found the desired camera with DeviceUserID "
                            << device_user_id_to_open << ": "
                            << it->GetModelName());
                PYLON_CAM_TYPE cam_type = detectPylonCamType(*it);
                return createFromDevice(cam_type,
                                        tl_factory.CreateDevice(*it));
            }
            else
            {
                ROS_ERROR_STREAM("Couldn't find the camera that matches the "
                    << "given DeviceUserID: " << device_user_id_to_open << "! "
                    << "Either the ID is wrong or the cam is not yet connected");
                return nullptr;
            }
        }
    }
    catch ( GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while opening the desired camera with "
            << "DeviceUserID: " << device_user_id_to_open << " occurred: \r\n"
            << e.GetDescription());
        return nullptr;
    }
}

const std::string& PylonCamera::deviceUserID() const
{
    return device_user_id_;
}

const size_t& PylonCamera::imageRows() const
{
    return img_rows_;
}

const size_t& PylonCamera::imageCols() const
{
    return img_cols_;
}

const size_t& PylonCamera::imageSize() const
{
    return img_size_byte_;
}

const float& PylonCamera::maxBrightnessTolerance() const
{
    return max_brightness_tolerance_;
}

const bool& PylonCamera::isReady() const
{
    return is_ready_;
}

std::size_t PylonCamera::numUserOutputs() const
{
    return user_output_selector_enums_.size();
}

const std::vector<float>& PylonCamera::sequencerExposureTimes() const
{
    return seq_exp_times_;
}

const bool& PylonCamera::isBinaryExposureSearchRunning() const
{
    return is_binary_exposure_search_running_;
}

PylonCamera::~PylonCamera()
{
    // Releases all Pylon resources.
    Pylon::PylonTerminate();
    if ( binary_exp_search_ )
    {
        delete binary_exp_search_;
        binary_exp_search_ = nullptr;
    }
}

}  // namespace pylon_camera
