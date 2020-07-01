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



/*-*-C++-*-*/
/**
   @file PointGreyCamera.cpp
   @author Chad Rockey
   @date July 11, 2011
   @brief Interface to Point Grey cameras

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

#include "pointgrey_camera_driver/PointGreyCamera.h"

#include <iostream>
#include <sstream>
#include <flycapture/FlyCapture2Defs.h>

using namespace FlyCapture2;

PointGreyCamera::PointGreyCamera():
  busMgr_(), cam_()
{
  serial_ = 0;
  captureRunning_ = false;
}

PointGreyCamera::~PointGreyCamera()
{
}

bool PointGreyCamera::setNewConfiguration(pointgrey_camera_driver::PointGreyConfig &config, const uint32_t &level)
{
  if(!cam_.IsConnected())
  {
    PointGreyCamera::connect();
  }

  // Activate mutex to prevent us from grabbing images during this time
  boost::mutex::scoped_lock scopedLock(mutex_);

  // return true if we can set values as desired.
  bool retVal = true;

  // Check video mode
  VideoMode vMode; // video mode desired
  Mode fmt7Mode; // fmt7Mode to set
  retVal &= PointGreyCamera::getVideoModeFromString(config.video_mode, vMode, fmt7Mode);

  // Only change video mode if we have to.
  // dynamic_reconfigure will report anything other than LEVEL_RECONFIGURE_RUNNING if we need to change videomode.
  if(level != PointGreyCamera::LEVEL_RECONFIGURE_RUNNING)
  {
    bool wasRunning = PointGreyCamera::stop(); // Check if camera is running, and if it is, stop it.
    if(vMode == VIDEOMODE_FORMAT7)
    {
      PixelFormat fmt7PixFmt;
      PointGreyCamera::getFormat7PixelFormatFromString(config.format7_color_coding, fmt7PixFmt);
      // Oh no, these all need to be converted into uints, so my pass by reference trick doesn't work
      uint16_t uwidth = (uint16_t)config.format7_roi_width;
      uint16_t uheight = (uint16_t)config.format7_roi_height;
      uint16_t uoffsetx = (uint16_t)config.format7_x_offset;
      uint16_t uoffsety = (uint16_t)config.format7_y_offset;
      retVal &= PointGreyCamera::setFormat7(fmt7Mode, fmt7PixFmt, uwidth, uheight, uoffsetx, uoffsety);
      config.format7_roi_width = uwidth;
      config.format7_roi_height = uheight;
      config.format7_x_offset = uoffsetx;
      config.format7_y_offset = uoffsety;
    }
    else
    {
      // Need to set just videoMode
      PointGreyCamera::setVideoMode(vMode);
    }
    // Restart the camera if it was running
    if(wasRunning)
    {
      PointGreyCamera::start();
    }
  }

  // Set frame rate
  retVal &= PointGreyCamera::setProperty(FRAME_RATE, false, config.frame_rate);

  // Set exposure
  retVal &= PointGreyCamera::setProperty(AUTO_EXPOSURE, config.auto_exposure, config.exposure);

  // Set sharpness
  retVal &= PointGreyCamera::setProperty(SHARPNESS, config.auto_sharpness, config.sharpness);

  // Set saturation
  retVal &= PointGreyCamera::setProperty(SATURATION, config.auto_saturation, config.saturation);

  // Set shutter time
  double shutter = 1000.0 * config.shutter_speed; // Needs to be in milliseconds
  retVal &= PointGreyCamera::setProperty(SHUTTER, config.auto_shutter, shutter);
  config.shutter_speed = shutter / 1000.0; // Needs to be in seconds

  // Set gain
  retVal &= PointGreyCamera::setProperty(GAIN, config.auto_gain, config.gain);

  // Set pan
  unsigned int pan = config.pan;
  unsigned int not_used = 0;
  retVal &= PointGreyCamera::setProperty(PAN, false, pan, not_used);
  config.pan = pan;

  // Set tilt
  unsigned int tilt = config.tilt;
  retVal &= PointGreyCamera::setProperty(TILT, false, tilt, not_used);
  config.tilt = tilt;

  // Set brightness
  retVal &= PointGreyCamera::setProperty(BRIGHTNESS, false, config.brightness);

  // Set gamma
  retVal &= PointGreyCamera::setProperty(GAMMA, false, config.gamma);

  // Set white balance
  uint16_t blue = config.white_balance_blue;
  uint16_t red = config.white_balance_red;
  retVal &= PointGreyCamera::setWhiteBalance(config.auto_white_balance, blue, red);
  config.white_balance_blue = blue;
  config.white_balance_red = red;

  // Set trigger
  switch (config.trigger_polarity)
  {
    case pointgrey_camera_driver::PointGrey_Low:
    case pointgrey_camera_driver::PointGrey_High:
      {
      bool temp = config.trigger_polarity;
      retVal &= PointGreyCamera::setExternalTrigger(config.enable_trigger, config.trigger_mode, config.trigger_source, config.trigger_parameter, config.trigger_delay, temp);
      config.strobe1_polarity = temp;
      }
      break;
    default:
      retVal &= false;
  }

  // Set strobe
  switch (config.strobe1_polarity)
  {
    case pointgrey_camera_driver::PointGrey_Low:
    case pointgrey_camera_driver::PointGrey_High:
      {
      bool temp = config.strobe1_polarity;
      retVal &= PointGreyCamera::setExternalStrobe(config.enable_strobe1, pointgrey_camera_driver::PointGrey_GPIO1, config.strobe1_duration, config.strobe1_delay, temp);
      config.strobe1_polarity = temp;
      }
      break;
    default:
      retVal &= false;
  }


  switch (config.strobe2_polarity)
  {
    case pointgrey_camera_driver::PointGrey_Low:
    case pointgrey_camera_driver::PointGrey_High:
      {
      bool temp = config.strobe2_polarity;
      retVal &= PointGreyCamera::setExternalStrobe(config.enable_strobe2, pointgrey_camera_driver::PointGrey_GPIO2, config.strobe2_duration, config.strobe2_delay, temp);
      config.strobe2_polarity = temp;
      }
      break;
    default:
      retVal &= false;
  }


  return retVal;
}

void PointGreyCamera::setGain(double &gain)
{
  PointGreyCamera::setProperty(GAIN, false, gain);
}

void PointGreyCamera::setBRWhiteBalance(bool auto_white_balance, uint16_t &blue, uint16_t &red)
{
  PointGreyCamera::setWhiteBalance(auto_white_balance, blue, red);
}

void PointGreyCamera::setVideoMode(FlyCapture2::VideoMode &videoMode)
{
  // Just set max frame rate, the actual double parameter will do the fine adjustments
  FrameRate frameRate = FRAMERATE_7_5; // Most reliable, so set as default.
  if(videoMode == VIDEOMODE_640x480Y8)
  {
    frameRate = FRAMERATE_30;
  }
  else if(videoMode == VIDEOMODE_1280x960Y8)
  {
    frameRate = FRAMERATE_15;
  }
  else if(videoMode == VIDEOMODE_1280x960Y16)
  {
    frameRate = FRAMERATE_7_5;
  }
  else if(videoMode == VIDEOMODE_FORMAT7)
  {
    frameRate = FRAMERATE_FORMAT7;
  }
  Error error = cam_.SetVideoModeAndFrameRate(videoMode, frameRate);
  PointGreyCamera::handleError("PointGreyCamera::setVideoMode Could not set video mode", error);
}

bool PointGreyCamera::setFormat7(FlyCapture2::Mode &fmt7Mode, FlyCapture2::PixelFormat &fmt7PixFmt, uint16_t &roi_width, uint16_t &roi_height, uint16_t &roi_offset_x, uint16_t &roi_offset_y)
{
  // return true if we can set values as desired.
  bool retVal = true;
  // Error for checking if functions went okay
  Error error;

  // Get Format7 information
  Format7Info fmt7Info;
  bool supported;
  fmt7Info.mode = fmt7Mode;
  error = cam_.GetFormat7Info(&fmt7Info, &supported);
  PointGreyCamera::handleError("PointGreyCamera::setFormat7 Could not get Format 7 information", error);
  if(!supported)
  {
    throw std::runtime_error("PointGreyCamera::setFormat7 Format 7 mode not supported on this camera.");
  }

  // Make Format7 Configuration
  Format7ImageSettings fmt7ImageSettings;
  fmt7ImageSettings.mode = fmt7Mode;
  fmt7ImageSettings.pixelFormat = fmt7PixFmt;

  // Check Width
  roi_width = roi_width / fmt7Info.imageHStepSize * fmt7Info.imageHStepSize; // Locks the width into an appropriate multiple using an integer divide
  if(roi_width == 0)
  {
    fmt7ImageSettings.width = fmt7Info.maxWidth;
  }
  else if(roi_width > fmt7Info.maxWidth)
  {
    roi_width = fmt7Info.maxWidth;
    fmt7ImageSettings.width = fmt7Info.maxWidth;
    retVal &= false;
  }
  else
  {
    fmt7ImageSettings.width = roi_width;
  }

  // Check Height
  roi_height = roi_height / fmt7Info.imageVStepSize * fmt7Info.imageVStepSize; // Locks the height into an appropriate multiple using an integer divide
  if(roi_height == 0)
  {
    fmt7ImageSettings.height = fmt7Info.maxHeight;
  }
  else if(roi_height > fmt7Info.maxHeight)
  {
    roi_height = fmt7Info.maxHeight;
    fmt7ImageSettings.height = fmt7Info.maxHeight;
    retVal &= false;
  }
  else
  {
    fmt7ImageSettings.height = roi_height;
  }

  // Check OffsetX
  roi_offset_x = roi_offset_x / fmt7Info.offsetHStepSize * fmt7Info.offsetHStepSize;  // Locks the X offset into an appropriate multiple using an integer divide
  if(roi_offset_x > (fmt7Info.maxWidth - fmt7ImageSettings.width))
  {
    roi_offset_x = fmt7Info.maxWidth - fmt7ImageSettings.width;
    retVal &= false;
  }
  fmt7ImageSettings.offsetX  = roi_offset_x;

  // Check OffsetY
  roi_offset_y = roi_offset_y / fmt7Info.offsetVStepSize * fmt7Info.offsetVStepSize;  // Locks the X offset into an appropriate multiple using an integer divide
  if(roi_offset_y > fmt7Info.maxHeight - fmt7ImageSettings.height)
  {
    roi_offset_y = fmt7Info.maxHeight - fmt7ImageSettings.height;
    retVal &= false;
  }
  fmt7ImageSettings.offsetY  = roi_offset_y;

  // Validate the settings to make sure that they are valid
  Format7PacketInfo fmt7PacketInfo;
  bool valid;
  error = cam_.ValidateFormat7Settings(&fmt7ImageSettings, &valid, &fmt7PacketInfo);
  PointGreyCamera::handleError("PointGreyCamera::setFormat7 Error validating Format 7 settings", error);
  if(!valid)
  {
    throw std::runtime_error("PointGreyCamera::setFormat7 Format 7 Settings Not Valid.");
  }

  // Stop the camera to allow settings to change.
  error = cam_.SetFormat7Configuration(&fmt7ImageSettings, fmt7PacketInfo.recommendedBytesPerPacket);
  PointGreyCamera::handleError("PointGreyCamera::setFormat7 Could not send Format7 configuration to the camera", error);

  // Get camera info to check if camera is running in color or mono mode
  CameraInfo cInfo;
  error = cam_.GetCameraInfo(&cInfo);
  PointGreyCamera::handleError("PointGreyCamera::setFormat7  Failed to get camera info.", error);
  isColor_ = cInfo.isColorCamera;

  return retVal;
}

bool PointGreyCamera::getVideoModeFromString(std::string &vmode, FlyCapture2::VideoMode &vmode_out, FlyCapture2::Mode &fmt7Mode)
{
  // return true if we can set values as desired.
  bool retVal = true;

  // Get camera info to check if color or black and white chameleon
  CameraInfo cInfo;
  Error error = cam_.GetCameraInfo(&cInfo);
  PointGreyCamera::handleError("PointGreyCamera::getVideoModeFromString  Failed to get camera info.", error);

  if(vmode.compare("640x480_mono8") == 0)
  {
    vmode_out = VIDEOMODE_640x480Y8;
  }
  else if(vmode.compare("640x480_mono16") == 0)
  {
    vmode_out = VIDEOMODE_640x480Y16;
  }
  else if(vmode.compare("1280x960_mono8") == 0)
  {
    vmode_out = VIDEOMODE_1280x960Y8;
    if(cInfo.isColorCamera)   // Is color camera, set the output differently
    {
      vmode = "1280x960_bayer8";
      retVal &= false;
    }
  }
  else if(vmode.compare("1280x960_bayer8") == 0)
  {
    vmode_out = VIDEOMODE_1280x960Y8;
    if(!cInfo.isColorCamera)   // Is black and white camera, set the output differently
    {
      vmode = "1280x960_mono8";
      retVal &= false;
    }
  }
  else if(vmode.compare("1280x960_mono16") == 0)
  {
    vmode_out = VIDEOMODE_1280x960Y16;
  }
  else if(vmode.compare("format7_mode0") == 0)
  {
    fmt7Mode = MODE_0;
    vmode_out = VIDEOMODE_FORMAT7;
  }
  else if(vmode.compare("format7_mode1") == 0)
  {
    fmt7Mode = MODE_1;
    vmode_out = VIDEOMODE_FORMAT7;
  }
  else if(vmode.compare("format7_mode2") == 0)
  {
    fmt7Mode = MODE_2;
    vmode_out = VIDEOMODE_FORMAT7;
  }
  else if(vmode.compare("format7_mode3") == 0)
  {
    fmt7Mode = MODE_3;
    vmode_out = VIDEOMODE_FORMAT7;
  }
  else if(vmode.compare("format7_mode4") == 0)
  {
    fmt7Mode = MODE_4;
    vmode_out = VIDEOMODE_FORMAT7;
  }
  else if(vmode.compare("format7_mode5") == 0)
  {
    fmt7Mode = MODE_5;
    vmode_out = VIDEOMODE_FORMAT7;
  }
  else if(vmode.compare("format7_mode7") == 0)
  {
    fmt7Mode = MODE_7;
    vmode_out = VIDEOMODE_FORMAT7;
  }
  else    // Something not supported was asked of us, drop down into the most compatible mode
  {
    vmode = "640x480_mono8";
    vmode_out = VIDEOMODE_640x480Y8;
    retVal &= false;
  }

  return retVal;
}

bool PointGreyCamera::getFormat7PixelFormatFromString(std::string &sformat, FlyCapture2::PixelFormat &fmt7PixFmt)
{
  // return true if we can set values as desired.
  bool retVal = true;

  // Get camera info to check if color or black and white camera
  CameraInfo cInfo;
  Error error = cam_.GetCameraInfo(&cInfo);
  PointGreyCamera::handleError("PointGreyCamera::getFormat7PixelFormatFromString  Failed to get camera info.", error);

  if(cInfo.isColorCamera)
  {
    if(sformat.compare("raw8") == 0)
    {
      fmt7PixFmt = PIXEL_FORMAT_RAW8;
    }
    else if(sformat.compare("raw16") == 0)
    {
      fmt7PixFmt = PIXEL_FORMAT_RAW16;
    }
    else if(sformat.compare("mono8") == 0)
    {
      fmt7PixFmt = PIXEL_FORMAT_MONO8;
    }
    else if(sformat.compare("mono16") == 0)
    {
      fmt7PixFmt = PIXEL_FORMAT_MONO16;
    }
    else if(sformat.compare("rgb8") == 0){
      fmt7PixFmt = PIXEL_FORMAT_RGB;
    }
    else
    {
      sformat = "raw8";
      fmt7PixFmt = PIXEL_FORMAT_RAW8;
      retVal &= false;
    }
  }
  else     // Is black and white
  {
    if(sformat.compare("mono8") == 0)
    {
      fmt7PixFmt = PIXEL_FORMAT_MONO8;
    }
    else if(sformat.compare("mono16") == 0)
    {
      fmt7PixFmt = PIXEL_FORMAT_MONO16;
    }
    else
    {
      sformat = "mono8";
      fmt7PixFmt = PIXEL_FORMAT_MONO8;
      retVal &= false;
    }
  }

  return retVal;
}

bool PointGreyCamera::setProperty(const FlyCapture2::PropertyType &type, const bool &autoSet, unsigned int &valueA, unsigned int &valueB)
{
  // return true if we can set values as desired.
  bool retVal = true;

  PropertyInfo pInfo;
  pInfo.type = type;
  Error error = cam_.GetPropertyInfo(&pInfo);
  PointGreyCamera::handleError("PointGreyCamera::setProperty Could not get property info.", error);

  if(pInfo.present)
  {
    Property prop;
    prop.type = type;
    prop.autoManualMode = (autoSet && pInfo.autoSupported);
    prop.absControl = false;
    prop.onOff = pInfo.onOffSupported;

    if(valueA < pInfo.min)
    {
      valueA = pInfo.min;
      retVal &= false;
    }
    else if(valueA > pInfo.max)
    {
      valueA = pInfo.max;
      retVal &= false;
    }
    if(valueB < pInfo.min)
    {
      valueB = pInfo.min;
      retVal &= false;
    }
    else if(valueB > pInfo.max)
    {
      valueB = pInfo.max;
      retVal &= false;
    }
    prop.valueA = valueA;
    prop.valueB = valueB;
    error = cam_.SetProperty(&prop);
    PointGreyCamera::handleError("PointGreyCamera::setProperty  Failed to set property ", error); /** @todo say which property? */

    // Read back setting to confirm
    error = cam_.GetProperty(&prop);
    PointGreyCamera::handleError("PointGreyCamera::setProperty  Failed to confirm property ", error); /** @todo say which property? */
    if(!prop.autoManualMode)
    {
      valueA = prop.valueA;
      valueB = prop.valueB;
    }
  }
  else     // Not supported
  {
    valueA = 0;
    valueB = 0;
  }

  return retVal;
}

bool PointGreyCamera::setProperty(const FlyCapture2::PropertyType &type, const bool &autoSet, double &value)
{
  // return true if we can set values as desired.
  bool retVal = true;

  PropertyInfo pInfo;
  pInfo.type = type;
  Error error = cam_.GetPropertyInfo(&pInfo);
  PointGreyCamera::handleError("PointGreyCamera::setProperty Could not get property info.", error);

  if(pInfo.present)
  {
    Property prop;
    prop.type = type;
    prop.autoManualMode = (autoSet && pInfo.autoSupported);
    prop.absControl = pInfo.absValSupported;
    prop.onOff = pInfo.onOffSupported;

    if(value < pInfo.absMin)
    {
      value = pInfo.absMin;
      retVal &= false;
    }
    else if(value > pInfo.absMax)
    {
      value = pInfo.absMax;
      retVal &= false;
    }
    prop.absValue = value;
    error = cam_.SetProperty(&prop);
    PointGreyCamera::handleError("PointGreyCamera::setProperty  Failed to set property ", error); /** @todo say which property? */

    // Read back setting to confirm
    error = cam_.GetProperty(&prop);
    PointGreyCamera::handleError("PointGreyCamera::setProperty  Failed to confirm property ", error); /** @todo say which property? */
    if(!prop.autoManualMode)
    {
      value = prop.absValue;
    }
  }
  else     // Not supported
  {
    value = 0.0;
  }
  return retVal;
}

bool PointGreyCamera::setWhiteBalance(bool &auto_white_balance, uint16_t &blue, uint16_t &red)
{
  // Get camera info to check if color or black and white chameleon
  CameraInfo cInfo;
  Error error = cam_.GetCameraInfo(&cInfo);
  handleError("PointGreyCamera::setWhiteBalance  Failed to get camera info.", error);

  if(!cInfo.isColorCamera)
  {
    // Not a color camera, does not support auto white balance
    auto_white_balance = false;
    red = 0;
    blue = 0;
    return false;
  }

  unsigned white_balance_addr = 0x80c;
  unsigned enable = 1 << 31;
  unsigned value = 1 << 25;

  if (auto_white_balance) {
    PropertyInfo prop_info;
    prop_info.type = WHITE_BALANCE;
    error = cam_.GetPropertyInfo(&prop_info);
    handleError("PointGreyCamera::setWhiteBalance  Failed to get property info.", error);
    if (!prop_info.autoSupported) {
      // This is typically because a color camera is in mono mode, so we set
      // the red and blue to some reasonable value for later use
      auto_white_balance = false;
      blue = 800;
      red = 550;
      return false;
    }
    // Auto white balance is supported
    error = cam_.WriteRegister(white_balance_addr, enable);
    handleError("PointGreyCamera::setWhiteBalance  Failed to write to register.", error);
    // Auto mode
    value |= 1 << 24;
  } else {
    // Manual mode
    value |= 0 << 24;
  }
  // Blue is bits 8-19 (0 is MSB), red is 20-31.
  value |= blue << 12 | red;
  error = cam_.WriteRegister(white_balance_addr, value);
  handleError("PointGreyCamera::setWhiteBalance  Failed to write to register.", error);
  return true;
}

void PointGreyCamera::setTimeout(const double &timeout)
{
  FC2Config pConfig;
  Error error = cam_.GetConfiguration(&pConfig);
  PointGreyCamera::handleError("PointGreyCamera::setTimeout Could not get camera configuration", error);
  pConfig.grabTimeout = (int)(1000.0 * timeout); // Needs to be in ms
  if(pConfig.grabTimeout < 0.00001)
  {
    pConfig.grabTimeout = -1; // Default - no timeout
  }
  error = cam_.SetConfiguration(&pConfig);
  PointGreyCamera::handleError("PointGreyCamera::setTimeout Could not set camera configuration", error);
}

float PointGreyCamera::getCameraTemperature()
{
  Property tProp;
  tProp.type = TEMPERATURE;
  Error error = cam_.GetProperty(&tProp);
  PointGreyCamera::handleError("PointGreyCamera::getCameraTemperature Could not get property.", error);
  return tProp.valueA / 10.0f - 273.15f;  // It returns values of 10 * K
}

float PointGreyCamera::getCameraFrameRate()
{
  Property fProp;
  fProp.type = FRAME_RATE;
  Error error = cam_.GetProperty(&fProp);
  PointGreyCamera::handleError("PointGreyCamera::getCameraFrameRate Could not get property.", error);
  std::cout << "Frame Rate is: " << fProp.absValue << std::endl;
  return fProp.absValue;
}

static int sourceNumberFromGpioName(const std::string s)
{
  if(s.compare("gpio0") == 0)
  {
    return 0;
  }
  else if(s.compare("gpio1") == 0)
  {
    return 1;
  }
  else if(s.compare("gpio2") == 0)
  {
    return 2;
  }
  else if(s.compare("gpio3") == 0)
  {
    return 3;
  }
  else
  {
    // Unrecognized pin
    return -1;
  }
}

bool PointGreyCamera::setExternalStrobe(bool &enable, const std::string &dest, double &duration, double &delay, bool &polarityHigh)
{
  // return true if we can set values as desired.
  bool retVal = true;

  // Check strobe source
  int pin;
  pin = sourceNumberFromGpioName(dest);
  if (pin < 0)
  {
    // Unrecognized source
    return false;
  }
  // Check for external trigger support
  StrobeInfo strobeInfo;
  strobeInfo.source = pin;
  Error error = cam_.GetStrobeInfo(&strobeInfo);
  PointGreyCamera::handleError("PointGreyCamera::setExternalStrobe Could not check external strobe support.", error);
  if(strobeInfo.present != true)
  {
    // Camera doesn't support external strobes on this pin, so set enable_strobe to false
    enable = false;
    return false;
  }

  StrobeControl strobeControl;
  strobeControl.source = pin;
  error = cam_.GetStrobe(&strobeControl);
  PointGreyCamera::handleError("PointGreyCamera::setExternalStrobe Could not get strobe control.", error);
  strobeControl.duration = duration;
  strobeControl.delay = delay;
  strobeControl.onOff = enable;
  strobeControl.polarity = polarityHigh;

  error = cam_.SetStrobe(&strobeControl);
  PointGreyCamera::handleError("PointGreyCamera::setExternalStrobe Could not set strobe control.", error);
  error = cam_.GetStrobe(&strobeControl);
  PointGreyCamera::handleError("PointGreyCamera::setExternalStrobe Could not get strobe control.", error);
  delay = strobeControl.delay;
  enable = strobeControl.onOff;
  polarityHigh = strobeControl.polarity;

  return retVal;
}

bool PointGreyCamera::setExternalTrigger(bool &enable, std::string &mode, std::string &source, int32_t &parameter, double &delay, bool &polarityHigh)
{
  // return true if we can set values as desired.
  bool retVal = true;
  // Check for external trigger support
  TriggerModeInfo triggerModeInfo;
  Error error = cam_.GetTriggerModeInfo(&triggerModeInfo);
  PointGreyCamera::handleError("PointGreyCamera::setExternalTrigger Could not check external trigger support.", error);
  if(triggerModeInfo.present != true)
  {
    // Camera doesn't support external triggering, so set enable_trigger to false
    enable = false;
    return false;
  }

  TriggerMode triggerMode;
  error = cam_.GetTriggerMode(&triggerMode);
  PointGreyCamera::handleError("PointGreyCamera::setExternalTrigger Could not get trigger mode.", error);
  triggerMode.onOff = enable;

  // Set trigger mode
  std::string tmode = mode;
  if(tmode.compare("mode0") == 0)
  {
    triggerMode.mode = 0;
  }
  else if(tmode.compare("mode1") == 0)
  {
    triggerMode.mode = 1;
  }
  else if(tmode.compare("mode3") == 0)
  {
    triggerMode.mode = 3;
  }
  else if(tmode.compare("mode14") == 0)
  {
    triggerMode.mode = 14;
  }
  else
  {
    // Unrecognized mode
    triggerMode.mode = 0;
    mode = "mode0";
    retVal &= false;
  }

  // Parameter is used for mode3 (return one out of every N frames).  So if N is two, it returns every other frame.
  triggerMode.parameter = parameter;

  // Set trigger source
  std::string tsource = source;
  int pin = sourceNumberFromGpioName(tsource);
  if (pin < 0)
  {
    // Unrecognized source
    triggerMode.source = 0;
    source = "gpio0";
    retVal &= false;
  }
  else
  {
    triggerMode.source = pin;
  }

  triggerMode.polarity = polarityHigh;

  error = cam_.SetTriggerMode(&triggerMode);
  PointGreyCamera::handleError("PointGreyCamera::setExternalTrigger Could not set trigger mode.", error);
  error = cam_.GetTriggerMode(&triggerMode);
  PointGreyCamera::handleError("PointGreyCamera::setExternalTrigger Could not get trigger mode.", error);
  enable = triggerMode.onOff;
  std::stringstream buff;
  buff << "mode" << triggerMode.mode;
  mode = buff.str();

  /** @todo, check delay min and max values */

  // Set trigger delay
  TriggerDelay triggerDelay;
  triggerDelay.type = TRIGGER_DELAY;
  triggerDelay.absControl = true;
  triggerDelay.absValue = delay;
  triggerDelay.onOff = true;
  error = cam_.SetTriggerDelay(&triggerDelay);
  PointGreyCamera::handleError("PointGreyCamera::setExternalTrigger Could not set trigger delay.", error);
  error = cam_.GetTriggerDelay(&triggerDelay);
  PointGreyCamera::handleError("PointGreyCamera::setExternalTrigger Could not get trigger delay.", error);
  delay = triggerDelay.absValue;

  return retVal;
}

void PointGreyCamera::setGigEParameters(bool auto_packet_size, unsigned int packet_size, unsigned int packet_delay)
{
  auto_packet_size_ = auto_packet_size;
  packet_size_ = packet_size;
  packet_delay_ = packet_delay;
}

void PointGreyCamera::setupGigEPacketSize(PGRGuid & guid)
{
  GigECamera cam;
  Error error;
  error = cam.Connect(&guid);
  PointGreyCamera::handleError("PointGreyCamera::connect could not connect as GigE camera", error);
  unsigned int packet_size;
  error = cam.DiscoverGigEPacketSize(&packet_size);
  PointGreyCamera::handleError("PointGreyCamera::connect could not discover GigE packet_size", error);
  GigEProperty prop;
  prop.propType = PACKET_SIZE;
  error = cam.GetGigEProperty(&prop);
  PointGreyCamera::handleError("PointGreyCamera::connect could not get GigE packet_size", error);
  prop.value = packet_size;
  error = cam.SetGigEProperty(&prop);
  PointGreyCamera::handleError("PointGreyCamera::connect could not set GigE packet_size", error);
}

void PointGreyCamera::setupGigEPacketSize(PGRGuid & guid, unsigned int packet_size)
{
  GigECamera cam;
  Error error;
  error = cam.Connect(&guid);
  PointGreyCamera::handleError("PointGreyCamera::connect could not connect as GigE camera", error);
  GigEProperty prop;
  prop.propType = PACKET_SIZE;
  prop.value = packet_size;
  error = cam.SetGigEProperty(&prop);
  PointGreyCamera::handleError("PointGreyCamera::connect could not set GigE packet_size", error);
}

void PointGreyCamera::setupGigEPacketDelay(PGRGuid & guid, unsigned int packet_delay)
{
  GigECamera cam;
  Error error;
  error = cam.Connect(&guid);
  PointGreyCamera::handleError("PointGreyCamera::connect could not connect as GigE camera", error);
  GigEProperty prop;
  prop.propType = PACKET_DELAY;
  prop.value = packet_delay;
  error = cam.SetGigEProperty(&prop);
  PointGreyCamera::handleError("PointGreyCamera::connect could not set GigE packet_delay", error);
}

void PointGreyCamera::connect()
{
  if(!cam_.IsConnected())
  {
    Error error;
    PGRGuid guid;  // GUIDS are NOT persistent accross executions, do not store them.
    if(serial_ != 0)  // If we have a specific camera to connect to.
    {
      error = busMgr_.GetCameraFromSerialNumber(serial_, &guid);
      std::stringstream serial_string;
      serial_string << serial_;
      std::string msg = "PointGreyCamera::connect Could not find camera with serial number: " + serial_string.str() + ". Is that camera plugged in?";
      PointGreyCamera::handleError(msg, error);
    }
    else     // Connect to any camera (the first)
    {
      error  = busMgr_.GetCameraFromIndex(0, &guid);
      PointGreyCamera::handleError("PointGreyCamera::connect Failed to get first connected camera", error);
    }

    FlyCapture2::InterfaceType ifType;
    error = busMgr_.GetInterfaceTypeFromGuid(&guid, &ifType);
    PointGreyCamera::handleError("PointGreyCamera::connect Failed to get interface style of camera", error);
    if (ifType == FlyCapture2::INTERFACE_GIGE)
    {
		// Set packet size:
        if (auto_packet_size_)
            setupGigEPacketSize(guid);
        else
            setupGigEPacketSize(guid, packet_size_);

        // Set packet delay:
        setupGigEPacketDelay(guid, packet_delay_);

        // Enable packet resend
        GigECamera cam;
        Error error;
        error = cam.Connect(&guid);
        PointGreyCamera::handleError("PointGreyCamera::connect could not connect as GigE camera", error);
        GigEConfig gigeconfig;
        error = cam.GetGigEConfig(&gigeconfig);
        PointGreyCamera::handleError("PointGreyCamera::GetGigEConfig could not get GigE setting", error);
        gigeconfig.enablePacketResend = true;
        error = cam.SetGigEConfig(&gigeconfig);
        PointGreyCamera::handleError("PointGreyCamera::SetGigEConfig could not set GigE settings (packet resend)", error);
    }

    error = cam_.Connect(&guid);
    PointGreyCamera::handleError("PointGreyCamera::connect Failed to connect to camera", error);

    // Get camera info to check if camera is running in color or mono mode
    CameraInfo cInfo;
    error = cam_.GetCameraInfo(&cInfo);
    PointGreyCamera::handleError("PointGreyCamera::connect  Failed to get camera info.", error);
    isColor_ = cInfo.isColorCamera;

    // Enable metadata
    EmbeddedImageInfo info;
    info.timestamp.onOff = true;
    info.gain.onOff = true;
    info.shutter.onOff = true;
    info.brightness.onOff = true;
    info.exposure.onOff = true;
    info.whiteBalance.onOff = true;
    info.frameCounter.onOff = true;
    info.ROIPosition.onOff = true;
    error = cam_.SetEmbeddedImageInfo(&info);
    PointGreyCamera::handleError("PointGreyCamera::connect Could not enable metadata", error);
  }
}

void PointGreyCamera::disconnect()
{
  boost::mutex::scoped_lock scopedLock(mutex_);
  captureRunning_ = false;
  if(cam_.IsConnected())
  {
    Error error = cam_.Disconnect();
    PointGreyCamera::handleError("PointGreyCamera::disconnect Failed to disconnect camera", error);
  }
}

void PointGreyCamera::start()
{
  if(cam_.IsConnected() && !captureRunning_)
  {
    // Start capturing images
    Error error = cam_.StartCapture();
    PointGreyCamera::handleError("PointGreyCamera::start Failed to start capture", error);
    captureRunning_ = true;
  }
}

bool PointGreyCamera::stop()
{
  if(cam_.IsConnected() && captureRunning_)
  {
    // Stop capturing images
    captureRunning_ = false;
    Error error = cam_.StopCapture();
    PointGreyCamera::handleError("PointGreyCamera::stop Failed to stop capture", error);
    return true;
  }
  return false;
}

void PointGreyCamera::grabImage(sensor_msgs::Image &image, const std::string &frame_id)
{
  boost::mutex::scoped_lock scopedLock(mutex_);
  if(cam_.IsConnected() && captureRunning_)
  {
    // Make a FlyCapture2::Image to hold the buffer returned by the camera.
    Image rawImage;
    // Retrieve an image
    Error error = cam_.RetrieveBuffer(&rawImage);
    PointGreyCamera::handleError("PointGreyCamera::grabImage Failed to retrieve buffer", error);
    metadata_ = rawImage.GetMetadata();

    // Set header timestamp as embedded for now
    TimeStamp embeddedTime = rawImage.GetTimeStamp();
    image.header.stamp.sec = embeddedTime.seconds;
    image.header.stamp.nsec = 1000 * embeddedTime.microSeconds;

    // Check the bits per pixel.
    uint8_t bitsPerPixel = rawImage.GetBitsPerPixel();

    // Set the image encoding
    std::string imageEncoding = sensor_msgs::image_encodings::MONO8;
    BayerTileFormat bayer_format = rawImage.GetBayerTileFormat();
    if(isColor_ && bayer_format != NONE)
    {
      if(bitsPerPixel == 16)
      {
        switch(bayer_format)
        {
          case RGGB:
            imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB16;
            break;
          case GRBG:
            imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG16;
            break;
          case GBRG:
            imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG16;
            break;
          case BGGR:
            imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR16;
            break;
        }
      }
      else
      {
        switch(bayer_format)
        {
        case RGGB:
          imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB8;
          break;
        case GRBG:
          imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG8;
          break;
        case GBRG:
          imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG8;
          break;
        case BGGR:
          imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR8;
          break;
        }
      }
    }
    else     // Mono camera or in pixel binned mode.
    {
      if(bitsPerPixel == 16)
      {
        imageEncoding = sensor_msgs::image_encodings::MONO16;
      }
      else if(bitsPerPixel==24)
      {
        imageEncoding = sensor_msgs::image_encodings::RGB8;
      }
      else
      {
        imageEncoding = sensor_msgs::image_encodings::MONO8;
      }
    }

    fillImage(image, imageEncoding, rawImage.GetRows(), rawImage.GetCols(), rawImage.GetStride(), rawImage.GetData());
    image.header.frame_id = frame_id;
  }
  else if(cam_.IsConnected())
  {
    throw CameraNotRunningException("PointGreyCamera::grabImage: Camera is currently not running.  Please start the capture.");
  }
  else
  {
    throw std::runtime_error("PointGreyCamera::grabImage not connected!");
  }
}

void PointGreyCamera::grabStereoImage(sensor_msgs::Image &image, const std::string &frame_id, sensor_msgs::Image &second_image, const std::string &second_frame_id)
{
  boost::mutex::scoped_lock scopedLock(mutex_);
  if(cam_.IsConnected() && captureRunning_)
  {
    // Make a FlyCapture2::Image to hold the buffer returned by the camera.
    Image rawImage;
    // Retrieve an image
    Error error = cam_.RetrieveBuffer(&rawImage);
    PointGreyCamera::handleError("PointGreyCamera::grabStereoImage Failed to retrieve buffer", error);
    metadata_ = rawImage.GetMetadata();

    // Set header timestamp as embedded for now
    TimeStamp embeddedTime = rawImage.GetTimeStamp();
    image.header.stamp.sec = embeddedTime.seconds;
    image.header.stamp.nsec = 1000 * embeddedTime.microSeconds;

    // GetBitsPerPixel returns 16, but that seems to mean "2 8 bit pixels,
    // one for each image". Therefore, we don't use it
    //uint8_t bitsPerPixel = rawImage.GetBitsPerPixel();

    // Set the image encoding
    std::string imageEncoding = sensor_msgs::image_encodings::MONO8;
    BayerTileFormat bayer_format = rawImage.GetBayerTileFormat();

    if(isColor_ && bayer_format != NONE)
    {
        switch(bayer_format)
        {
        case RGGB:
          imageEncoding = sensor_msgs::image_encodings::BAYER_RGGB8;
          break;
        case GRBG:
          imageEncoding = sensor_msgs::image_encodings::BAYER_GRBG8;
          break;
        case GBRG:
          imageEncoding = sensor_msgs::image_encodings::BAYER_GBRG8;
          break;
        case BGGR:
          imageEncoding = sensor_msgs::image_encodings::BAYER_BGGR8;
          break;
        }
    }
    else     // Mono camera
    {
      imageEncoding = sensor_msgs::image_encodings::MONO8;
    }

    // Set up the output images
    image.encoding = imageEncoding;
    second_image.encoding = imageEncoding;
    image.height = rawImage.GetRows();
    second_image.height = image.height;
    image.width = rawImage.GetCols();
    second_image.width = image.width;
    image.step = rawImage.GetStride() / 2;
    second_image.step = image.step;
    image.is_bigendian = 0;
    second_image.is_bigendian = 0;
    size_t st0 = (image.height * image.step);
    image.data.resize(st0);
    second_image.data.resize(st0);
    image.header.frame_id = frame_id;
    second_image.header.frame_id = second_frame_id;

    // Get the image data
    const uint8_t* raw_data = rawImage.GetData();

    // Step through the raw data and set each image in turn
    for(size_t i = 0; i < rawImage.GetRows(); i++)  // Rows
    {
      for(size_t j = 0; j < rawImage.GetCols(); j++)  // Columns that need to have the 16 bits split into 2 8 bit groups
      {
        size_t index = i * image.step + j;
        size_t raw_index = 2 * index;
        image.data[index] = raw_data[raw_index];
        second_image.data[index] = raw_data[raw_index + 1];
      }
    }

    //fillImage(image, imageEncoding, rawImage.GetRows(), rawImage.GetCols(), rawImage.GetStride(), rawImage.GetData());
  }
  else if(cam_.IsConnected())
  {
    throw CameraNotRunningException("PointGreyCamera::grabStereoImage: Camera is currently not running.  Please start the capture.");
  }
  else
  {
    throw std::runtime_error("PointGreyCamera::grabStereoImage not connected!");
  }
}

uint PointGreyCamera::getGain()
{
  return metadata_.embeddedGain >> 20;
}

uint PointGreyCamera::getShutter()
{
  return metadata_.embeddedShutter >> 20;
}

uint PointGreyCamera::getBrightness()
{
  return metadata_.embeddedTimeStamp >> 20;
}

uint PointGreyCamera::getExposure()
{
  return metadata_.embeddedBrightness >> 20;
}

uint PointGreyCamera::getWhiteBalance()
{
  return metadata_.embeddedExposure >> 8;
}

uint PointGreyCamera::getROIPosition()
{
  return metadata_.embeddedROIPosition >> 24;
}

void PointGreyCamera::setDesiredCamera(const uint32_t &id)
{
  serial_ = id;
}

std::vector<uint32_t> PointGreyCamera::getAttachedCameras()
{
  std::vector<uint32_t> cameras;
  unsigned int num_cameras;
  Error error = busMgr_.GetNumOfCameras(&num_cameras);
  PointGreyCamera::handleError("PointGreyCamera::getAttachedCameras: Could not get number of cameras", error);
  for(unsigned int i = 0; i < num_cameras; i++)
  {
    unsigned int this_serial;
    error = busMgr_.GetCameraSerialNumberFromIndex(i, &this_serial);
    PointGreyCamera::handleError("PointGreyCamera::getAttachedCameras: Could not get get serial number from index", error);
    cameras.push_back(this_serial);
  }
  return cameras;
}

void PointGreyCamera::handleError(const std::string &prefix, const FlyCapture2::Error &error)
{
  if(error == PGRERROR_TIMEOUT)
  {
    throw CameraTimeoutException("PointGreyCamera: Failed to retrieve buffer within timeout.");
  }
  else if(error == PGRERROR_IMAGE_CONSISTENCY_ERROR)
  {
    throw CameraImageConsistencyError("PointGreyCamera: Image consistency error.");
  }
  else if(error != PGRERROR_OK)     // If there is actually an error (PGRERROR_OK means the function worked as intended...)
  {
    std::string start(" | FlyCapture2::ErrorType ");
    std::stringstream out;
    out << error.GetType();
    std::string desc(error.GetDescription());
    throw std::runtime_error(prefix + start + out.str() + " " + desc);
  }
}
