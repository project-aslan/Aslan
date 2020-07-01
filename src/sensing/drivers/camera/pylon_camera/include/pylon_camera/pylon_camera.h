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

#ifndef PYLON_CAMERA_PYLON_CAMERA_H
#define PYLON_CAMERA_PYLON_CAMERA_H

#include <string>
#include <vector>
#include <map>

#include <pylon_camera/pylon_camera_parameter.h>
#include <pylon_camera/binary_exposure_search.h>
#include <sensor_msgs/RegionOfInterest.h>

namespace pylon_camera
{

// Number of channels for image encoding
#define CHANNEL_MONO8 1
#define CHANNEL_RGB8  3

/**
 * The PylonCamera base class. Create a new instance using the static create() functions.
 */
class PylonCamera
{
public:
    /**
     * Create a new PylonCamera instance. It will return the first camera that could be found.
     * @return new PylonCamera instance or NULL if no camera was found.
     */
    static PylonCamera* create();

    /**
     * Create a new PylonCamera instance based on the DeviceUserID of the camera.
     * @param device_user_id Pylon DeviceUserID. If the string is empty, the
     * first camera that could be found is returned.
     * @return new PylonCamera instance or NULL if the camera was not found.
     */
    static PylonCamera* create(const std::string& device_user_id);

    /**
     * Configures the camera according to the software trigger mode.
     * @return true if all the configuration could be set up.
     */
    virtual bool registerCameraConfiguration() = 0;

    /**
     * Opens the desired camera, the communication starts from now on.
     * @return true if the camera could be opened.
     */
    virtual bool openCamera() = 0;

    /**
     * Returns the connection state of the camera device.
     * @return true if the camera device removal from the PC has been detected.
     */
    virtual bool isCamRemoved() = 0;

    /**
     * Configure the sequencer exposure times.
     * @param exposure_times the list of exposure times.
     * @return true if all parameters could be sent to the camera.
     */
    virtual bool setupSequencer(const std::vector<float>& exposure_times) = 0;

    /**
     * Configures the camera according to the provided ros parameters.
     * This will use the device specific parameters as e.g. the mtu size for
     * GigE-Cameras
     * @param parameters The PylonCameraParameter set to use
     * @return true if all parameters could be sent to the camera.
     */
    virtual bool applyCamSpecificStartupSettings(const PylonCameraParameter& parameters) = 0;

    /**
     * Initializes the internal parameters of the PylonCamera instance.
     * @param parameters The PylonCameraParameter set to use
     * @return true if all parameters could be sent to the camera.
     */
    virtual bool startGrabbing(const PylonCameraParameter& parameters) = 0;

    /**
     * Grab a camera frame and copy the result into image
     * @param image reference to the output image.
     * @return true if the image was grabbed successfully.
     */
    virtual bool grab(std::vector<uint8_t>& image) = 0;

    /**
     * Grab a camera frame and copy the result into image
     * @param image pointer to the image buffer.
     *              Caution: Make sure the buffer is initialized correctly!
     * @return true if the image was grabbed successfully.
     */
    virtual bool grab(uint8_t* image) = 0;

    /**
     * @brief sets shutter mode for the camera (rolling or global_reset)
     * @param mode
     * @return
     */
    virtual bool setShutterMode(const pylon_camera::SHUTTER_MODE& mode) = 0;

    /**
     * Update area of interest in the camera image
     * @param target_roi the target roi
     * @param reached_roi the roi that could be set
     * @return true if the targeted roi could be reached
     */
    virtual bool setROI(const sensor_msgs::RegionOfInterest target_roi,
			sensor_msgs::RegionOfInterest& reached_roi) = 0;
    
    /**
     * Sets the target horizontal binning_x factor
     * @param target_binning_x the target horizontal binning_x factor.
     * @param reached_binning_x the reached horizontal binning_x factor.
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setBinningX(const size_t& target_binning_x,
                             size_t& reached_binning_x) = 0;
    
    /**
     * Sets the target vertical binning_y factor
     * @param target_binning_y the target vertical binning_y factor.
     * @param reached_binning_y the reached vertical binning_y factor.
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setBinningY(const size_t& target_binning_y,
                             size_t& reached_binning_y) = 0;

    /**
     * Detects the supported image pixel encodings of the camera an stores
     * them in a vector.
     * @return a list of strings describing the supported encodings in GenAPI
     *         language.
     */
    virtual std::vector<std::string> detectAvailableImageEncodings() = 0;

    /**
     * Sets the desired image pixel encoding (channel meaning, ordering, size)
     * taken from the list of strings in include/sensor_msgs/image_encodings.h
     * The supported encodings are 'mono8', 'bgr8', 'rgb8', 'bayer_bggr8',
     * 'bayer_gbrg8', 'bayer_rggb8' and 'yuv422'
     * @param target_ros_endcoding: string describing the encoding.
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setImageEncoding(const std::string& target_ros_encoding) = 0;

    /**
     * Sets the exposure time in microseconds
     * @param target_exposure the desired exposure time to set in microseconds.
     * @param reached_exposure time in microseconds
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setExposure(const float& target_exposure,
                             float& reached_exposure) = 0;

    /**
     * Sets autoflash active for the specified lines
     * @param flash_on_lines map from line e.g. 1 or 2 to a boolean to 
              activate or deactivate the autoflash for this line .
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setAutoflash(const std::map<int, bool> flash_on_lines) = 0;
    /**
     * Sets the gain in percent independent of the camera type
     * @param target_gain the target gain in percent.
     * @param reached_gain the reached gain in percent.
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setGain(const float& target_gain, float& reached_gain) = 0;

    /**
     * Sets the target gamma value
     * @param target_gamma the target gamma value.
     * @param reached_gamma the reached gamma value.
     * @return false if a communication error occurred or true otherwise.
     */
    virtual bool setGamma(const float& target_gamma, float& reached_gamma) = 0;

    /**
     * Sets the target brightness
     * Setting the exposure time to -1 enables the AutoExposureContinuous mode.
     * Setting the exposure time to  0 disables the AutoExposure function.
     * If the target exposure time is not in the range of Pylon's auto target brightness range
     * the extended brightness search is started.
     * @param target_brightness is the desired brightness. Range is [1...255].
     * @param current_brightness is the current brightness with the given settings.
     * @param exposure_auto flag which indicates if the target_brightness
     *                      should be reached adapting the exposure time
     * @param gain_auto flag which indicates if the target_brightness should be
     *                      reached adapting the gain.
     * @return true if the brightness could be reached or false otherwise.
     */
    virtual bool setBrightness(const int& target_brightness,
                               const float& current_brightness,
                               const bool& exposure_auto,
                               const bool& gain_auto) = 0;

    /**
     * @brief Detects and counts the number of user-settable-outputs the cam
     *        provides. This might be zero for some cameras. The size affects
     *        the number of 'set' ros-services the camera_node will provide.
     *        A vector which length equals the number of user-settable outputs
     *        will be generated. Hence e.g. output '1' can be accessed via
     *        user_output_selector_enums_.at(1).
     * @return the UserOutputSelector enum list
     */
    virtual std::vector<int> detectAndCountNumUserOutputs() = 0;

    /**
     * @brief setUserOutput sets the digital output
     * @param output_id
     * @param value goal value for output
     * @return true if value was set
     */
    virtual bool setUserOutput(const int& output_id, const bool& value) = 0;

    /**
     * Returns the current x offset setting.
     * @return the horizontal x offset setting.
     */
    virtual size_t currentOffsetX() = 0;

    /**
     * Returns the current y offset setting.
     * @return the horizontal y offset setting.
     */
    virtual size_t currentOffsetY() = 0;
    
    /**
     * Returns the current roi setting.
     * @return the roi setting.
     */
    virtual sensor_msgs::RegionOfInterest currentROI() = 0;
    
    /**
     * Returns the current horizontal binning_x setting.
     * @return the horizontal binning_x setting.
     */
    virtual size_t currentBinningX() = 0;

    /**
     * Returns the current vertical binning_y setting.
     * @return the vertical binning_y setting.
     */
    virtual size_t currentBinningY() = 0;

    /**
     * Get the camera image encoding according to sensor_msgs::image_encodings
     * The supported encodings are 'mono8', 'bgr8', 'rgb8', 'bayer_bggr8',
     * 'bayer_gbrg8', 'bayer_rggb8' and 'yuv422'
     * @return the current ros image pixel encoding.
     */
    virtual std::string currentROSEncoding() const = 0;

    /**
     * Get the number of bytes per pixel
     * @return number of bytes per pixel
     */
    virtual int imagePixelDepth() const = 0;

    /**
     * Returns the current exposure time in microseconds.
     * @return the exposure time in microseconds.
     */
    virtual float currentExposure() = 0;

    /**
     * Returns the current auto exposure time lower limit
     * @return the current auto exposure time lower limit
     */
    virtual float currentAutoExposureTimeLowerLimit() = 0;

    /**
     * Returns the current auto exposure time upper limit
     * @return the current auto exposure time upper limit
     */
    virtual float currentAutoExposureTimeUpperLimit() = 0;

    /**
     * Returns the current gain in percent.
     * @return the gain time percent.
     */
    virtual float currentGain() = 0;

    /**
     * Returns the current auto gain lower limit
     * @return the current auto gain lower limit
     */
    virtual float currentAutoGainLowerLimit() = 0;

    /**
     * Returns the current auto gain upper limit
     * @return the current auto gain upper limit
     */
    virtual float currentAutoGainUpperLimit() = 0;
    /**
     * Returns the current gamma value.
     * @return the gamma value.
     */
    virtual float currentGamma() = 0;

    /**
     * Checks if the camera currently tries to regulate towards a target brightness.
     * This can either be done by pylon for the range [50 - 205] or the own extendended binary search one
     * for the ranges [1 - 49] and [206 - 254].
     * @return true if the brightness-search is running
     */
    virtual bool isBrightnessSearchRunning() = 0;

    /**
     * Checks if the auto brightness function from the Pylon API is enabled.
     * @return true if AutoExposure is set to AutoExposureContinuous or AutoExposureOnce.
     */
    virtual bool isPylonAutoBrightnessFunctionRunning() = 0;

    /**
     * Getter for is_binary_exposure_search_running_
     * @return true if the extended exposure search is running
     */
    const bool& isBinaryExposureSearchRunning() const;

    /**
     * Disables all currently running brightness search methods in case that
     * the desired brightness is reached or a timeout occurred
     */
    virtual void disableAllRunningAutoBrightessFunctions() = 0;

    /**
     * Enables the continuous auto exposure mode
     */
    virtual void enableContinuousAutoExposure() = 0;

    /**
     * Enables the continuous auto gain mode
     */
    virtual void enableContinuousAutoGain() = 0;

    /**
     * Get the camera type. Currently supported cameras are USB, DART and GigE
     * @return camera type as string
     */
    virtual std::string typeName() const = 0;

    /**
     * Minimum possible increment between two possible exposure values
     * @return the minimum possible increment between two possible exposure values
     */
    virtual float exposureStep() = 0;

    /**
     * Getter for the device user id of the used camera
     * @return the device_user_id
     */
    const std::string& deviceUserID() const;

    /**
     * Getter for the image height
     * @return number of rows in the image
     */
    const size_t& imageRows() const;

    /**
     * Getter for the image width
     * @return number of columns in the image
     */
    const size_t& imageCols() const;

    /**
     * Getter for the is_ready_ flag. This is set in case that the
     * grab-result-pointer of the first acquisition contains valid data.
     * Hence this is the current state of the interface
     * @return true if the interface is ready
     */
    const bool& isReady() const;

    /**
     * Returns the number of digital user outputs, which can be set by the
     * camera. Might be zero for some cameras. The size affects the number of
     * 'set' ros-services the camera node will provide
     * @return number of digital user outputs
     */
    std::size_t numUserOutputs() const;

    /**
     * Returns the image size in bytes
     * @return the image size in bytes
     */
    const size_t& imageSize() const;

    /**
     * Get the maximum achievable frame rate
     * @return float
     */
    virtual float maxPossibleFramerate() = 0;

    /**
     * Checks if the camera has the auto exposure feature.
     * @return true if the camera supports auto exposure.
     */
    const bool& hasAutoExposure() const;

    /**
     * Max allowed delta between target and reached brightness
     * @return the allowed tolerance.
     */
    const float& maxBrightnessTolerance() const;

    /**
     * Getter for the sequencer exposure times.
     * @return the list of exposure times
     */
    const std::vector<float>& sequencerExposureTimes() const;

    virtual ~PylonCamera();
protected:
    /**
     * Protected default constructor.
     */
    PylonCamera();

    /**
     * Enables the extended brightness search.
     * @param brightness target brightness
     * @return true after reaching the target brightness.
     */
    virtual bool setExtendedBrightness(const int& target_brightness,
                                       const float& current_brightness) = 0;

    /**
     * Parameters for the extended brightness search
     */
    BinaryExposureSearch* binary_exp_search_;

    /**
     * The DeviceUserID of the found camera
     */
    std::string device_user_id_;

    /**
     * Number of image rows.
     */
    size_t img_rows_;

    /**
     * Number of image columns.
     */
    size_t img_cols_;

    /**
     * The size of the image in number of bytes.
     */
    size_t img_size_byte_;

    /**
     * The max time a single grab is allowed to take. This value should always
     * be greater then the max possible exposure time of the camera
     */
    float grab_timeout_;

    /**
     * Flag which is set in case that the grab-result-pointer of the first
     * acquisition contains valid data
     */
    bool is_ready_;

    /**
     * True if the extended binary exposure search is running.
     */
    bool is_binary_exposure_search_running_;

    /**
     * Max allowed delta between target and reached brightness
     */
    const float max_brightness_tolerance_;

    /**
     * Exposure times to use when in sequencer mode.
     */
    std::vector<float> seq_exp_times_;

    /**
     * Vector containing all available user outputs.
     */
    std::vector<int> user_output_selector_enums_;

    /**
     * Vector that contains the available image_encodings the camera supports.
     * The strings describe the GenAPI encoding.
     */
    std::vector<std::string> available_image_encodings_;
};

}  // namespace pylon_camera

#endif  // PYLON_CAMERA_PYLON_CAMERA_H
