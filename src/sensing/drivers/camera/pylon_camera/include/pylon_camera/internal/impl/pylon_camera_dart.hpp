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

#ifndef PYLON_CAMERA_INTERNAL_DART_H_
#define PYLON_CAMERA_INTERNAL_DART_H_

#include <string>
#include <vector>

#include <pylon_camera/internal/impl/pylon_camera_usb.hpp>

namespace pylon_camera
{

class PylonDARTCamera : public PylonUSBCamera
{
public:
    explicit PylonDARTCamera(Pylon::IPylonDevice* device);
    virtual ~PylonDARTCamera();

    virtual bool applyCamSpecificStartupSettings(const PylonCameraParameter& params);
    virtual bool setUserOutput(int output_id, bool value);
    virtual std::string typeName() const;

protected:
    virtual bool setupSequencer(const std::vector<float>& exposure_times,
                                std::vector<float>& exposure_times_set);
    virtual bool grab(Pylon::CGrabResultPtr& grab_result);
};

PylonDARTCamera::PylonDARTCamera(Pylon::IPylonDevice* device) :
    PylonUSBCamera(device)
{}

PylonDARTCamera::~PylonDARTCamera()
{}

bool PylonDARTCamera::applyCamSpecificStartupSettings(const PylonCameraParameter& parameters)
{
    if ( !PylonUSBCamera::applyCamSpecificStartupSettings(parameters) )
    {
        return false;
    }
    return true;
}

bool PylonDARTCamera::setUserOutput(int output_id, bool value)
{
    ROS_ERROR("Dart camera has no digital output.");
    return false;
}

bool PylonDARTCamera::setupSequencer(const std::vector<float>& exposure_times,
                                     std::vector<float>& exposure_times_set)
{
    ROS_ERROR("Sequencer Mode for Dart Cameras not yet implemented");
    return false;
}

bool PylonDARTCamera::grab(Pylon::CGrabResultPtr& grab_result)
{
    try
    {
        // /!\ The dart camera device does not support
        // 'waitForFrameTriggerReady'
        cam_->ExecuteSoftwareTrigger();

        cam_->RetrieveResult(grab_timeout_, grab_result,
                             Pylon::TimeoutHandling_ThrowException);
    }
    catch (const GenICam::GenericException &e)
    {
        if ( cam_->IsCameraDeviceRemoved() )
        {
            ROS_ERROR("Camera was removed");
        }
        else
        {
            ROS_ERROR_STREAM("An image grabbing exception in pylon camera occurred: "
                    << e.GetDescription());
        }
        return false;
    }
    catch ( ... )
    {
        ROS_ERROR("An unspecified image grabbing exception in pylon camera occurred");
        return false;
    }

    if ( !grab_result->GrabSucceeded() )
    {
        ROS_ERROR_STREAM("Error: " << grab_result->GetErrorCode()
                << " " << grab_result->GetErrorDescription());
        return false;
    }

    return true;
}

std::string PylonDARTCamera::typeName() const
{
    return "DART";
}

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_INTERNAL_DART_H_
