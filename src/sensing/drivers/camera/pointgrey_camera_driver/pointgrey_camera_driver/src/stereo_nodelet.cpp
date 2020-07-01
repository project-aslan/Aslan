/*
This code was developed by the National Robotics Engineering Center (NREC), part of the Robotics Institute at Carnegie Mellon University.
Its development was funded by DARPA under the LS3 program and submitted for public release on June 7th, 2012.
Release was granted on August, 21st 2012 with Distribution Statement "A" (Approved for Public Release, Distribution Unlimited).

This software is released under a BSD license:

Copyright (c) 2012, Carnegie Mellon University. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
Neither the name of the Carnegie Mellon University nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/



/**
   @file stereo_nodelet.cpp
   @author Chad Rockey
   @date March 9, 2011
   @brief ROS nodelet for the Point Grey Stereo Cameras

   @attention Copyright (C) 2012
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

// ROS and associated nodelet interface and PLUGINLIB declaration header
#include "ros/ros.h"
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "pointgrey_camera_driver/PointGreyCamera.h" // The actual standalone library for the PointGreys

#include <image_transport/image_transport.h> // ROS library that allows sending compressed images
#include <camera_info_manager/camera_info_manager.h> // ROS library that publishes CameraInfo topics
#include <sensor_msgs/CameraInfo.h> // ROS message header for CameraInfo
#include <std_msgs/Float64.h>

#include <wfov_camera_msgs/WFOVImage.h>
#include <image_exposure_msgs/ExposureSequence.h> // Message type for configuring gain and white balance.

#include <diagnostic_updater/diagnostic_updater.h> // Headers for publishing diagnostic messages.
#include <diagnostic_updater/publisher.h>

#include <boost/thread.hpp> // Needed for the nodelet to launch the reading thread.

#include <dynamic_reconfigure/server.h> // Needed for the dynamic_reconfigure gui service to run

namespace pointgrey_camera_driver
{

class PointGreyStereoCameraNodelet: public nodelet::Nodelet
{
public:
  PointGreyStereoCameraNodelet() {}

  ~PointGreyStereoCameraNodelet()
  {
    pubThread_->interrupt();
    pubThread_->join();
    cleanUp();
  }

private:
  /*!
  * \brief Function that allows reconfiguration of the camera.
  *
  * This function serves as a callback for the dynamic reconfigure service.  It simply passes the configuration object to the driver to allow the camera to reconfigure.
  * \param config  camera_library::CameraConfig object passed by reference.  Values will be changed to those the driver is currently using.
  * \param level driver_base reconfiguration level.  See driver_base/SensorLevels.h for more information.
  */
  void paramCallback(pointgrey_camera_driver::PointGreyConfig &config, uint32_t level)
  {

    // Stereo is only active in this mode (16 bits, 8 for each image)
    config.video_mode = "format7_mode3";

    try
    {
      NODELET_DEBUG("Dynamic reconfigure callback with level: %d", level);
      pg_.setNewConfiguration(config, level);

      // Store needed parameters for the metadata message
      gain_ = config.gain;
      wb_blue_ = config.white_balance_blue;
      wb_red_ = config.white_balance_red;


      // Store CameraInfo binning information
      if(config.video_mode == "640x480_mono8" || config.video_mode == "format7_mode1")
      {
        binning_x_ = 2;
        binning_y_ = 2;
      }
      else if(config.video_mode == "format7_mode2")
      {
        binning_x_ = 0;
        binning_y_ = 2;
      }
      else
      {
        binning_x_ = 0;
        binning_y_ = 0;
      }

      // Store CameraInfo RegionOfInterest information
      if(config.video_mode == "format7_mode0" || config.video_mode == "format7_mode1" || config.video_mode == "format7_mode2")
      {
        roi_x_offset_ = config.format7_x_offset;
        roi_y_offset_ = config.format7_y_offset;
        roi_width_ = config.format7_roi_width;
        roi_height_ = config.format7_roi_height;
        do_rectify_ = true; // Set to true if an ROI is used.
      }
      else
      {
        // Zeros mean the full resolution was captured.
        roi_x_offset_ = 0;
        roi_y_offset_ = 0;
        roi_height_ = 0;
        roi_width_ = 0;
        do_rectify_ = false; // Set to false if the whole image is captured.
      }
    }
    catch(std::runtime_error& e)
    {
      NODELET_ERROR("Reconfigure Callback failed with error: %s", e.what());
    }
  }

  /*!
  * \brief Serves as a psuedo constructor for nodelets.
  *
  * This function needs to do the MINIMUM amount of work to get the nodelet running.  Nodelets should not call blocking functions here for a significant period of time.
  */
  void onInit()
  {
    // Get nodeHandles
    ros::NodeHandle &nh = getMTNodeHandle();
    ros::NodeHandle &pnh = getMTPrivateNodeHandle();
    std::string firstcam;
    pnh.param<std::string>("first_namespace", firstcam, "left");
    ros::NodeHandle lnh(getMTNodeHandle(), firstcam);
    std::string secondcam;
    pnh.param<std::string>("second_namespace", secondcam, "right");
    ros::NodeHandle rnh(getMTNodeHandle(), secondcam);

    // Get a serial number through ros
    int serialParam;
    pnh.param<int>("serial", serialParam, 0);
    uint32_t serial = (uint32_t)serialParam;
    // Get the location of our camera config yamls
    std::string camera_info_url;
    pnh.param<std::string>("camera_info_url", camera_info_url, "");
    std::string second_info_url;
    pnh.param<std::string>("second_info_url", second_info_url, "");
    // Get the desired frame_id, set to 'camera' if not found
    pnh.param<std::string>("frame_id", frame_id_, "camera");
    pnh.param<std::string>("second_frame_id", second_frame_id_, frame_id_); // Default to left frame_id per stereo API
    // Set the timeout for grabbing images from the network
    double timeout;
    pnh.param("timeout", timeout, 1.0);

    // Try connecting to the camera
    volatile bool connected = false;
    while(!connected && ros::ok())
    {
      try
      {
        NODELET_INFO("Connecting to camera serial: %u", serial);
        pg_.setDesiredCamera(serial);
        NODELET_DEBUG("Actually connecting to camera.");
        pg_.connect();
        connected = true;
        NODELET_DEBUG("Setting timeout to: %f.", timeout);
        pg_.setTimeout(timeout);
      }
      catch(std::runtime_error& e)
      {
        NODELET_ERROR("%s", e.what());
        ros::Duration(1.0).sleep(); // sleep for one second each time
      }
    }

    // Start up the dynamic_reconfigure service, note that this needs to stick around after this function ends
    srv_ = boost::make_shared <dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig> > (pnh);
    dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig>::CallbackType f =  boost::bind(&pointgrey_camera_driver::PointGreyStereoCameraNodelet::paramCallback, this, _1, _2);
    srv_->setCallback(f);

    // Start the camera info manager and attempt to load any configurations
    std::stringstream cinfo_name;
    cinfo_name << serial;
    cinfo_.reset(new camera_info_manager::CameraInfoManager(lnh, cinfo_name.str(), camera_info_url));
    rcinfo_.reset(new camera_info_manager::CameraInfoManager(rnh, cinfo_name.str(), second_info_url));
    /*if (cinfo_->validateURL(camera_info_url)){ // Load CameraInfo (if any) from the URL.  Will ROS_ERROR if bad.
      cinfo_->loadCameraInfo(camera_info_url);
    }*/
    ci_.reset(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci_->header.frame_id = frame_id_;
    rci_.reset(new sensor_msgs::CameraInfo(rcinfo_->getCameraInfo()));
    rci_->header.frame_id = second_frame_id_;

    // Publish topics using ImageTransport through camera_info_manager (gives cool things like compression)
    it_.reset(new image_transport::ImageTransport(lnh));
    it_pub_ = it_->advertiseCamera("image_raw", 5);
    rit_.reset(new image_transport::ImageTransport(rnh));
    rit_pub_ = rit_->advertiseCamera("image_raw", 5);

    // Set up diagnostics
    updater_.setHardwareID("pointgrey_camera " + serial);

    ///< @todo Move this to diagnostics
    temp_pub_ = nh.advertise<std_msgs::Float64>("temp", 5);

    // Subscribe to gain and white balance changes
    sub_ = nh.subscribe("image_exposure_sequence", 10, &pointgrey_camera_driver::PointGreyStereoCameraNodelet::gainWBCallback, this);

    volatile bool started = false;
    while(!started)
    {
      try
      {
        NODELET_DEBUG("Starting camera capture.");
        pg_.start();
        started = true;
        // Start the thread to loop through and publish messages
        pubThread_ = boost::shared_ptr< boost::thread > (new boost::thread(boost::bind(&pointgrey_camera_driver::PointGreyStereoCameraNodelet::devicePoll, this)));
      }
      catch(std::runtime_error& e)
      {
        NODELET_ERROR("%s", e.what());
        ros::Duration(1.0).sleep(); // sleep for one second each time
      }
    }
  }

  /*!
  * \brief Cleans up the memory and disconnects the camera.
  *
  * This function is called from the deconstructor since pg_.stop() and pg_.disconnect() could throw exceptions.
  */
  void cleanUp()
  {
    try
    {
      NODELET_DEBUG("Stopping camera capture.");
      pg_.stop();
      NODELET_DEBUG("Disconnecting from camera.");
      pg_.disconnect();
    }
    catch(std::runtime_error& e)
    {
      NODELET_ERROR("%s", e.what());
    }
  }

  /*!
  * \brief Function for the boost::thread to grabImages and publish them.
  *
  * This function continues until the thread is interupted.  Responsible for getting sensor_msgs::Image and publishing them.
  */
  void devicePoll()
  {
    while(!boost::this_thread::interruption_requested())   // Block until we need to stop this thread.
    {
      try
      {
        sensor_msgs::ImagePtr image(new sensor_msgs::Image);
        sensor_msgs::ImagePtr second_image(new sensor_msgs::Image);
        pg_.grabStereoImage(*image, frame_id_, *second_image, second_frame_id_);

        ros::Time time = ros::Time::now();
        image->header.stamp = time;
        second_image->header.stamp = time;
        ci_->header.stamp = time;
        rci_->header.stamp = time;
        ci_->binning_x = binning_x_;
        rci_->binning_x = binning_x_;
        ci_->binning_y = binning_y_;
        rci_->binning_y = binning_y_;
        ci_->roi.x_offset = roi_x_offset_;
        rci_->roi.x_offset = roi_x_offset_;
        ci_->roi.y_offset = roi_y_offset_;
        rci_->roi.y_offset = roi_y_offset_;
        ci_->roi.height = roi_height_;
        rci_->roi.height = roi_height_;
        ci_->roi.width = roi_width_;
        rci_->roi.width = roi_width_;
        ci_->roi.do_rectify = do_rectify_;
        rci_->roi.do_rectify = do_rectify_;

        it_pub_.publish(image, ci_);
        rit_pub_.publish(second_image, rci_);
        std_msgs::Float64 temp;
        temp.data = pg_.getCameraTemperature();
        temp_pub_.publish(temp);
      }
      catch(CameraTimeoutException& e)
      {
        NODELET_WARN("%s", e.what());
      }
      catch(std::runtime_error& e)
      {
        NODELET_ERROR("%s", e.what());
        try
        {
          // Something terrible has happened, so let's just disconnect and reconnect to see if we can recover.
          pg_.disconnect();
          ros::Duration(1.0).sleep(); // sleep for one second each time
          pg_.connect();
          pg_.start();
        }
        catch(std::runtime_error& e2)
        {
          NODELET_ERROR("%s", e2.what());
        }
      }
      // Update diagnostics
      updater_.update();
    }
  }

  void gainWBCallback(const image_exposure_msgs::ExposureSequence &msg)
  {
    try
    {
      NODELET_DEBUG("Gain callback:  Setting gain to %f and white balances to %u, %u", msg.gain, msg.white_balance_blue, msg.white_balance_red);
      gain_ = msg.gain;
      pg_.setGain(gain_);
      wb_blue_ = msg.white_balance_blue;
      wb_red_ = msg.white_balance_red;
      pg_.setBRWhiteBalance(false, wb_blue_, wb_red_);
    }
    catch(std::runtime_error& e)
    {
      NODELET_ERROR("gainWBCallback failed with error: %s", e.what());
    }
  }

  boost::shared_ptr<dynamic_reconfigure::Server<pointgrey_camera_driver::PointGreyConfig> > srv_; ///< Needed to initialize and keep the dynamic_reconfigure::Server in scope.
  boost::shared_ptr<image_transport::ImageTransport> it_; ///< Needed to initialize and keep the ImageTransport in scope.
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_; ///< Needed to initialize and keep the CameraInfoManager in scope.
  image_transport::CameraPublisher it_pub_; ///< CameraInfoManager ROS publisher
  ros::Publisher temp_pub_; ///< Publisher for current camera temperature @todo Put this in diagnostics instead.
  ros::Subscriber sub_; ///< Subscriber for gain and white balance changes.

  diagnostic_updater::Updater updater_; ///< Handles publishing diagnostics messages.
  double min_freq_;
  double max_freq_;

  PointGreyCamera pg_; ///< Instance of the PointGreyCamera library, used to interface with the hardware.
  sensor_msgs::CameraInfoPtr ci_; ///< Camera Info message.
  std::string frame_id_; ///< Frame id for the camera messages, defaults to 'camera'
  boost::shared_ptr<boost::thread> pubThread_; ///< The thread that reads and publishes the images.

  double gain_;
  uint16_t wb_blue_;
  uint16_t wb_red_;

  // For stereo cameras
  std::string second_frame_id_; ///< Frame id used for the second camera.
  boost::shared_ptr<image_transport::ImageTransport> rit_; ///< Needed to initialize and keep the ImageTransport in scope.
  boost::shared_ptr<camera_info_manager::CameraInfoManager> rcinfo_; ///< Needed to initialize and keep the CameraInfoManager in scope.
  image_transport::CameraPublisher rit_pub_; ///< CameraInfoManager ROS publisher
  sensor_msgs::CameraInfoPtr rci_; ///< Camera Info message.

  // Parameters for cameraInfo
  size_t binning_x_; ///< Camera Info pixel binning along the image x axis.
  size_t binning_y_; ///< Camera Info pixel binning along the image y axis.
  size_t roi_x_offset_; ///< Camera Info ROI x offset
  size_t roi_y_offset_; ///< Camera Info ROI y offset
  size_t roi_height_; ///< Camera Info ROI height
  size_t roi_width_; ///< Camera Info ROI width
  bool do_rectify_; ///< Whether or not to rectify as if part of an image.  Set to false if whole image, and true if in ROI mode.
};

PLUGINLIB_EXPORT_CLASS(pointgrey_camera_driver::PointGreyStereoCameraNodelet, nodelet::Nodelet)  // Needed for Nodelet declaration
}
