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

#ifndef PYLON_CAMERA_INTERNAL_GIGE_H_
#define PYLON_CAMERA_INTERNAL_GIGE_H_

#include <string>
#include <vector>

#include <pylon_camera/internal/pylon_camera.h>

#include <pylon/gige/BaslerGigEInstantCamera.h>

namespace pylon_camera
{

struct GigECameraTrait
{
    typedef Pylon::CBaslerGigEInstantCamera CBaslerInstantCameraT;
    typedef Basler_GigECameraParams::ExposureAutoEnums ExposureAutoEnums;
    typedef Basler_GigECameraParams::GainAutoEnums GainAutoEnums;
    typedef Basler_GigECameraParams::PixelFormatEnums PixelFormatEnums;
    typedef Basler_GigECameraParams::PixelSizeEnums PixelSizeEnums;
    typedef GenApi::IInteger AutoTargetBrightnessType;
    typedef GenApi::IInteger GainType;
    typedef int64_t AutoTargetBrightnessValueType;
    typedef Basler_GigECameraParams::ShutterModeEnums ShutterModeEnums;
    typedef Basler_GigECamera::UserOutputSelectorEnums UserOutputSelectorEnums;
    typedef Basler_GigECamera::LineSelectorEnums LineSelectorEnums;
    typedef Basler_GigECamera::LineModeEnums LineModeEnums;
    typedef Basler_GigECamera::LineSourceEnums LineSourceEnums;

    static inline AutoTargetBrightnessValueType convertBrightness(const int& value)
    {
        return value;
    }
};

typedef PylonCameraImpl<GigECameraTrait> PylonGigECamera;


template <>
bool PylonGigECamera::setAutoflash(const std::map<int, bool> flash_on_lines)
{
    // bool acc_auto_flash = false;
    for (const std::pair<int, bool> p : flash_on_lines)
    {
        try
        {
            ROS_INFO("Executing SetAutoFlash: %i -> %i", p.first, p.second);
            if (p.first == 2)
            {
                if (p.second)
                {
                    // Set to flash
                    cam_->LineSelector.SetValue(Basler_GigECameraParams::LineSelector_Line2);
                    cam_->LineMode.SetValue(Basler_GigECameraParams::LineMode_Output);
                    cam_->LineSource.SetValue(Basler_GigECameraParams::LineSource_ExposureActive);
                }
                else
                {
                    // Set to default
                    cam_->LineSelector.SetValue(Basler_GigECameraParams::LineSelector_Line2);
                    cam_->LineMode.SetValue(Basler_GigECameraParams::LineMode_Output);
                    cam_->LineSource.SetValue(Basler_GigECameraParams::LineSource_UserOutput1);
                }
                continue;
            }
            if (p.first == 3)
            {
                if (p.second)
                {
                    // Set to flash
                    cam_->LineSelector.SetValue(Basler_GigECameraParams::LineSelector_Line3);
                    cam_->LineMode.SetValue(Basler_GigECameraParams::LineMode_Output);
                    cam_->LineSource.SetValue(Basler_GigECameraParams::LineSource_ExposureActive);
                }
                else
                {
                    // Set to default
                    cam_->LineSelector.SetValue(Basler_GigECameraParams::LineSelector_Line3);
                    cam_->LineMode.SetValue(Basler_GigECameraParams::LineMode_Input);
                }
                continue;
           }
           ROS_WARN("Got request to set Flash for line %i, but only 2 and 3 are implemented!", p.first);
        }
        catch ( const GenICam::GenericException &e )
        {
            ROS_ERROR_STREAM("Error applying cam specific startup setting for GigE cameras: "
                    << e.GetDescription());
        }
    }
    return true;
}

template <>
bool PylonGigECamera::applyCamSpecificStartupSettings(const PylonCameraParameter& parameters)
{
    try
    {
        // Remove all previous settings (sequencer etc.)
        // Default Setting = Free-Running
        cam_->UserSetSelector.SetValue(Basler_GigECameraParams::UserSetSelector_Default);
        cam_->UserSetLoad.Execute();
        // UserSetSelector_Default overrides Software Trigger Mode !!
        cam_->TriggerSource.SetValue(Basler_GigECameraParams::TriggerSource_Software);
        cam_->TriggerMode.SetValue(Basler_GigECameraParams::TriggerMode_On);

        /* Thresholds for the AutoExposure Functions:
         *  - lower limit can be used to get rid of changing light conditions
         *    due to 50Hz lamps (-> 20ms cycle duration)
         *  - upper limit is to prevent motion blur
         */
        double upper_lim = std::min(parameters.auto_exp_upper_lim_,
                                    cam_->ExposureTimeAbs.GetMax());
        cam_->AutoExposureTimeAbsLowerLimit.SetValue(cam_->ExposureTimeAbs.GetMin());
        cam_->AutoExposureTimeAbsUpperLimit.SetValue(upper_lim);

        cam_->AutoGainRawLowerLimit.SetValue(cam_->GainRaw.GetMin());
        cam_->AutoGainRawUpperLimit.SetValue(cam_->GainRaw.GetMax());

        // The gain auto function and the exposure auto function can be used at the
        // same time. In this case, however, you must also set the
        // Auto Function Profile feature.
        // acA1920-40gm does not support Basler_GigECameraParams::GainSelector_AnalogAll
        // has Basler_GigECameraParams::GainSelector_All instead
        // cam_->GainSelector.SetValue(Basler_GigECameraParams::GainSelector_AnalogAll);

        if ( GenApi::IsAvailable(cam_->BinningHorizontal) &&
             GenApi::IsAvailable(cam_->BinningVertical) )
        {
            ROS_INFO_STREAM("Cam has binning range: x(hz) = ["
                    << cam_->BinningHorizontal.GetMin() << " - "
                    << cam_->BinningHorizontal.GetMax() << "], y(vt) = ["
                    << cam_->BinningVertical.GetMin() << " - "
                    << cam_->BinningVertical.GetMax() << "].");
        }
        else
        {
            ROS_INFO_STREAM("Cam does not support binning.");
        }

        ROS_INFO_STREAM("Cam has exposure time range: ["
                << cam_->ExposureTimeAbs.GetMin()
                << " - " << cam_->ExposureTimeAbs.GetMax()
                << "] measured in microseconds.");
        ROS_INFO_STREAM("Cam has gain range: ["
                << cam_->GainRaw.GetMin() << " - "
                << cam_->GainRaw.GetMax()
                << "] measured in device specific units.");

        // Check if gamma is available, print range
        if ( !GenApi::IsAvailable(cam_->Gamma) )
        {
            ROS_WARN("Cam gamma not available, will keep the default (auto).");
        }
        else
        {
            ROS_INFO_STREAM("Cam has gamma range: ["
                << cam_->Gamma.GetMin() << " - "
                << cam_->Gamma.GetMax() << "].");
        }

        ROS_INFO_STREAM("Cam has pylon auto brightness range: ["
                << cam_->AutoTargetValue.GetMin() << " - "
                << cam_->AutoTargetValue.GetMax()
                << "] which is the average pixel intensity.");

        // raise inter-package delay (GevSCPD) for solving error:
        // 'the image buffer was incompletely grabbed'
        // also in ubuntu settings -> network -> options -> MTU Size
        // from 'automatic' to 3000 if card supports it
        // Raspberry PI has MTU = 1500, max value for some cards: 9000
        cam_->GevSCPSPacketSize.SetValue(parameters.mtu_size_);
        if (parameters.auto_flash_)
        {
            std::map<int, bool> flash_on_lines;
            ROS_INFO("Flash 2: %i", parameters.auto_flash_line_2_);
            ROS_INFO("Flash 3: %i", parameters.auto_flash_line_3_);

            flash_on_lines[2] = parameters.auto_flash_line_2_;
            flash_on_lines[3] = parameters.auto_flash_line_3_;
            setAutoflash(flash_on_lines);
        }

        // http://www.baslerweb.com/media/documents/AW00064902000%20Control%20Packet%20Timing%20With%20Delays.pdf
        // inter package delay in ticks (? -> mathi said in nanosec) -> prevent lost frames
        // package size * n_cams + 5% overhead = inter package size
        // int n_cams = 1;
        // int inter_package_delay_in_ticks = n_cams * imageSize() * 1.05;
        cam_->GevSCPD.SetValue(parameters.inter_pkg_delay_);
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("Error applying cam specific startup setting for GigE cameras: "
                << e.GetDescription());
        return false;
    }
    return true;
}


template <>
bool PylonGigECamera::setupSequencer(const std::vector<float>& exposure_times,
                                     std::vector<float>& exposure_times_set)
{
    try
    {
        if ( GenApi::IsWritable(cam_->SequenceEnable) )
        {
            cam_->SequenceEnable.SetValue(false);
        }
        else
        {
            ROS_ERROR("Sequence mode not enabled.");
            return false;
        }

        cam_->SequenceAdvanceMode = Basler_GigECameraParams::SequenceAdvanceMode_Auto;
        cam_->SequenceSetTotalNumber = exposure_times.size();

        for ( std::size_t i = 0; i < exposure_times.size(); ++i )
        {
            // Set parameters for each step
            cam_->SequenceSetIndex = i;
            float reached_exposure;
            setExposure(exposure_times.at(i), reached_exposure);
            exposure_times_set.push_back(reached_exposure / 1000000.);
            cam_->SequenceSetStore.Execute();
        }

        // config finished
        cam_->SequenceEnable.SetValue(true);
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR("%s", e.GetDescription());
        return false;
    }
    return true;
}

template <>
GenApi::IFloat& PylonGigECamera::exposureTime()
{
    if ( GenApi::IsAvailable(cam_->ExposureTimeAbs) )
    {
        return cam_->ExposureTimeAbs;
    }
    else
    {
        throw std::runtime_error("Error while accessing ExposureTimeAbs in PylonGigECamera");
    }
}

template <>
GigECameraTrait::GainType& PylonGigECamera::gain()
{
    if ( GenApi::IsAvailable(cam_->GainRaw) )
    {
        return cam_->GainRaw;
    }
    else
    {
        throw std::runtime_error("Error while accessing GainRaw in PylonGigECamera");
    }
}

/**
 * @override
 * Overrides the base implementation as the Gamma object might not be available
 * for some GigE color cameras when the 'AUTO GAMMA' is activated (see setGamma()).
 *
 * @returns -1 if Gamma is set to AUTO, returns gamma value if Gamma is set to USER.
 */
template <>
float PylonGigECamera::currentGamma()
{
    if ( !GenApi::IsAvailable(cam_->Gamma) )
    {
        ROS_WARN_STREAM("Error while trying to access gamma: cam.Gamma NodeMap"
                << " is not available!");
        return -1.;
    }
    else
    {
        return static_cast<float>(gamma().GetValue());
    }
}

template <>
bool PylonGigECamera::setGamma(const float& target_gamma, float& reached_gamma)
{
    // for GigE cameras you have to enable gamma first
    if ( GenApi::IsAvailable(cam_->GammaEnable) )
    {
        cam_->GammaEnable.SetValue(true);
    }

    if ( !GenApi::IsAvailable(cam_->Gamma) )
    {
        ROS_WARN_STREAM("Error while trying to set gamma: cam.Gamma NodeMap is"
                << " not available!");
        return true;
    }

    if ( GenApi::IsAvailable(cam_->GammaSelector) )
    {
        // set gamma selector to USER, so that the gamma value has an influence
        try
        {
            cam_->GammaSelector.SetValue(Basler_GigECameraParams::GammaSelector_User);
        }
        catch ( const GenICam::GenericException &e )
        {
            ROS_ERROR_STREAM("An exception while setting gamma selector to"
                    << " USER occurred: " << e.GetDescription());
            return false;
        }
    }
    
    try
    {
        float gamma_to_set = target_gamma;
        if ( gamma().GetMin() > gamma_to_set )
        {
            gamma_to_set = gamma().GetMin();
            ROS_WARN_STREAM("Desired gamma unreachable! Setting to lower limit: "
                                  << gamma_to_set);
        }
        else if ( gamma().GetMax() < gamma_to_set )
        {
            gamma_to_set = gamma().GetMax();
            ROS_WARN_STREAM("Desired gamma unreachable! Setting to upper limit: "
                                  << gamma_to_set);
        }
        gamma().SetValue(gamma_to_set);
        reached_gamma = currentGamma();
    }
    catch ( const GenICam::GenericException &e )
    {
        ROS_ERROR_STREAM("An exception while setting target gamma to "
                << target_gamma << " occurred: " << e.GetDescription());
        return false;
    }
    return true;
}

template <>
GenApi::IFloat& PylonGigECamera::autoExposureTimeLowerLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoExposureTimeAbsLowerLimit) )
    {
        return cam_->AutoExposureTimeAbsLowerLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoExposureTimeAbsLowerLimit in PylonGigECamera");
    }
}

template <>
GenApi::IFloat& PylonGigECamera::autoExposureTimeUpperLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoExposureTimeAbsUpperLimit) )
    {
        return cam_->AutoExposureTimeAbsUpperLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoExposureTimeAbsUpperLimit in PylonGigECamera");
    }
}

template <>
GigECameraTrait::GainType& PylonGigECamera::autoGainLowerLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainRawLowerLimit) )
    {
        return cam_->AutoGainRawLowerLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainRawLowerLimit in PylonGigECamera");
    }
}

template <>
GigECameraTrait::GainType& PylonGigECamera::autoGainUpperLimit()
{
    if ( GenApi::IsAvailable(cam_->AutoGainRawUpperLimit) )
    {
        return cam_->AutoGainRawUpperLimit;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoGainRawUpperLimit in PylonGigECamera");
    }
}

template <>
GenApi::IFloat& PylonGigECamera::resultingFrameRate()
{
    if ( GenApi::IsAvailable(cam_->ResultingFrameRateAbs) )
    {
        return cam_->ResultingFrameRateAbs;
    }
    else
    {
        throw std::runtime_error("Error while accessing ResultingFrameRateAbs in PylonGigECamera");
    }
}

template <>
GigECameraTrait::AutoTargetBrightnessType& PylonGigECamera::autoTargetBrightness()
{
    if ( GenApi::IsAvailable(cam_->AutoTargetValue) )
    {
        return cam_->AutoTargetValue;
    }
    else
    {
        throw std::runtime_error("Error while accessing AutoTargetValue in PylonGigECamera");
    }
}

template <>
std::string PylonGigECamera::typeName() const
{
    return "GigE";
}

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_INTERNAL_GIGE_H_
