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

#ifndef PYLON_CAMERA_PYLON_CAMERA_PARAMETER_H
#define PYLON_CAMERA_PYLON_CAMERA_PARAMETER_H

#include <string>
#include <vector>
#include <ros/ros.h>

namespace pylon_camera
{

enum SHUTTER_MODE
{
    SM_ROLLING = 0,
    SM_GLOBAL = 1,
    SM_GLOBAL_RESET_RELEASE = 2,
    SM_DEFAULT =  -1,
};

/**
 * Parameter class for the PylonCamera
 */
class PylonCameraParameter
{
public:
    PylonCameraParameter();

    virtual ~PylonCameraParameter();

    /**
     * Read the parameters from the parameter server.
     * If invalid parameters can be detected, the interface will reset them
     * to the default values.
     * @param nh the ros::NodeHandle to use
     */
    void readFromRosParameterServer(const ros::NodeHandle& nh);

    /**
     * Getter for the device_user_id_ set from ros-parameter server
     */
    const std::string& deviceUserID() const;

    /**
     * Setter for the device_user_id_  to the class and as well
     * the ros-parameter server
     */
    void adaptDeviceUserId(const ros::NodeHandle& nh, const std::string& device_user_id);

    /**
     * Getter for the string describing the shutter mode
     */
    std::string shutterModeString() const;

    /**
     * Getter for the camera_frame_ set from ros-parameter server
     */
    const std::string& cameraFrame() const;

    /**
     * Getter for the frame_rate_ read from ros-parameter server
     */
    const double& frameRate() const;

    /**
     * Getter for the image_encoding_ read from ros-parameter server
     */
    const std::string& imageEncoding() const;

    /**
     * Setter for the frame_rate_ initially set from ros-parameter server
     * The frame rate needs to be updated with the value the camera supports
     */
    void setFrameRate(const ros::NodeHandle& nh, const double& frame_rate);

    /**
     * Getter for the camera_info_url set from ros-parameter server
     */
    const std::string& cameraInfoURL() const;

    /**
     * Setter for the camera_info_url_ if a new CameraInfo-Msgs Object is
     * provided via the SetCameraInfo-service from the CameraInfoManager
     */
    void setCameraInfoURL(const ros::NodeHandle& nh,
                          const std::string& camera_info_url);

public:
    /** Binning factor to get downsampled images. It refers here to any camera
     * setting which combines rectangular neighborhoods of pixels into larger
     * "super-pixels." It reduces the resolution of the output image to
     * (width / binning_x) x (height / binning_y).
     * The default values binning_x = binning_y = 0 are considered the same
     * as binning_x = binning_y = 1 (no subsampling).
     */
    size_t binning_x_;
    size_t binning_y_;

    /**
     * Flags which indicate if the binning factors are provided and hence
     * should be set during startup
     */
    bool binning_x_given_;
    bool binning_y_given_;

    /**
     * Factor that describes the image downsampling to speed up the exposure
     * search to find the desired brightness.
     * The smallest window height is img_rows/downsampling_factor
     */
    int downsampling_factor_exp_search_;

    // #######################################################################
    // ###################### Image Intensity Settings  ######################
    // #######################################################################
    // The following settings do *NOT* have to be set. Each camera has default
    // values which provide an automatic image adjustment
    // If one would like to adjust image brightness, it is not
    // #######################################################################

    /**
     * The exposure time in microseconds to be set after opening the camera.
     */
    double exposure_;

    /**
     * Flag which indicates if the exposure time is provided and hence should
     * be set during startup
     */
    bool exposure_given_;

    /**
     * The target gain in percent of the maximal value the camera supports
     * For USB-Cameras, the gain is in dB, for GigE-Cameras it is given in so
     * called 'device specific units'.
     */
    double gain_;

    /**
     * Flag which indicates if the gain value is provided and hence should be
     * set during startup
     */
    bool gain_given_;

    /**
     * Gamma correction of pixel intensity.
     * Adjusts the brightness of the pixel values output by the camera's sensor
     * to account for a non-linearity in the human perception of brightness or
     * of the display system (such as CRT).
     */
    double gamma_;

    /**
     * Flag which indicates if the gamma correction value is provided and
     * hence should be set during startup
     */
    bool gamma_given_;

    /**
     * The average intensity value of the images. It depends on the exposure
     * time as well as the gain setting. If 'exposure' is provided, the
     * interface will try to reach the desired brightness by only varying the
     * gain. (What may often fail, because the range of possible exposure
     * values is many times higher than the gain range).
     * If 'gain' is provided, the interface will try to reach the desired
     * brightness by only varying the exposure time. If gain AND exposure are
     * given, it is not possible to reach the brightness, because both are
     * assumed to be fix.
     */
    int brightness_;

    /**
     * Flag which indicates if the average brightness is provided and hence
     * should be set during startup
     */
    bool brightness_given_;

    /**
     * Only relevant, if 'brightness' is set as ros-parameter:
     * The brightness_continuous flag controls the auto brightness function.
     * If it is set to false, the brightness will only be reached once.
     * Hence changing light conditions lead to changing brightness values.
     * If it is set to true, the given brightness will be reached continuously,
     * trying to adapt to changing light conditions. This is only possible for
     * values in the possible auto range of the pylon API which is
     * e.g. [50 - 205] for acA2500-14um and acA1920-40gm
     */
    bool brightness_continuous_;
    /**
     * Only relevant, if 'brightness' is given as ros-parameter:
     * If the camera should try to reach and / or keep the brightness, hence
     * adapting to changing light conditions, at least one of the following
     * flags must be set. If both are set, the interface will use the profile
     * that tries to keep the  gain at minimum to reduce white noise.
     * The exposure_auto flag indicates, that the desired brightness will
     * be reached by adapting the exposure time.
     * The gain_auto flag indicates, that the desired brightness will be
     * reached by adapting the gain.
     */
    bool exposure_auto_;
    bool gain_auto_;
    // #######################################################################

    /**
     * The timeout while searching the exposure which is connected to the
     * desired brightness. For slow system this has to be increased.
     */
    double exposure_search_timeout_;

    /**
     * The exposure search can be limited with an upper bound. This is to
     * prevent very high exposure times and resulting timeouts.
     * A typical value for this upper bound is ~2000000us.
     */
    double auto_exp_upper_lim_;

    /**
     * The MTU size. Only used for GigE cameras.
     * To prevent lost frames the camera has to be configured
     * with the MTU size the network card supports. A value greater 3000
     * should be good (1500 for RaspberryPI)
     */
    int mtu_size_;

    /**
     * The inter-package delay in ticks. Only used for GigE cameras.
     * To prevent lost frames it should be greater 0.
     * For most of GigE-Cameras, a value of 1000 is reasonable.
     * For GigE-Cameras used on a RaspberryPI this value should be set to 11772
     */
    int inter_pkg_delay_;

    /**
      Shutter mode
    */
    SHUTTER_MODE shutter_mode_;

    /**
     * Flag that indicates if the camera has been calibrated and the intrinsic
     * calibration matrices are available
     */
    bool has_intrinsic_calib_;

    /**
     * Flag that indicates if the camera has a flash connected which should be on on exposure
     * Only supported for GigE cameras. Default: false
     */
    bool auto_flash_;
    /**
     * Flag that indicates if the camera, when using auto_flash == true, a flash connected on line 2 which should be on on exposure
     * Only supported for GigE cameras. Default: true
     */
    bool auto_flash_line_2_;
    /**
     * Flag that indicates if the camera has, when using auto_flash == true,  a flash connected on line 3 which should be on on exposure
     * Only supported for GigE cameras. Default: true
     */
    bool auto_flash_line_3_;

protected:
    /**
     * Validates the parameter set found on the ros parameter server.
     * If invalid parameters can be detected, the interface will reset them
     * to the default values.
     * @param nh the ros::NodeHandle to use
     */
    void validateParameterSet(const ros::NodeHandle& nh);

    /**
     * The tf frame under which the images were published
     */
    std::string camera_frame_;

    /**
     * The DeviceUserID of the camera. If empty, the first camera found in the
     * device list will be used
     */
    std::string device_user_id_;

    /**
     * The desired publisher frame rate if listening to the topics.
     * This parameter can only be set once at startup
     * Calling the GrabImages-Action can result in a higher framerate
     */
    double frame_rate_;

    /**
     * The CameraInfo URL (Uniform Resource Locator) where the optional
     * intrinsic camera calibration parameters are stored. This URL string will
     * be parsed from the CameraInfoManager:
     * http://wiki.ros.org/camera_info_manager
     */
    std::string camera_info_url_;

    /**
     * The encoding of the pixels -- channel meaning, ordering, size taken
     * from the list of strings in include/sensor_msgs/image_encodings.h
     * The supported encodings are 'mono8', 'bgr8', 'rgb8', 'bayer_bggr8',
     * 'bayer_gbrg8', 'bayer_rggb8' and 'yuv422'
     */
    std::string image_encoding_;
};

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_PYLON_CAMERA_PARAMETER_H
