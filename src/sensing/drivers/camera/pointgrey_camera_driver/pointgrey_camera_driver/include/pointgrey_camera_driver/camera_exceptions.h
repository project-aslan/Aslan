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
   @file camera_exceptions.h
   @author Chad Rockey
   @date July 20, 2011
   @brief Interface to Point Grey cameras

   @attention Copyright (C) 2011
   @attention National Robotics Engineering Center
   @attention Carnegie Mellon University
*/

#ifndef _CAMERAEXCEPTIONS_H_
#define _CAMERAEXCEPTIONS_H_

#include <stdexcept>

class CameraTimeoutException: public std::runtime_error
{
public:
  CameraTimeoutException(): runtime_error("Image not found within timeout.") {}
  CameraTimeoutException(std::string msg): runtime_error(msg.c_str()) {}
};

class CameraNotRunningException: public std::runtime_error
{
public:
  CameraNotRunningException(): runtime_error("Camera is currently not running.  Please start the capture.") {}
  CameraNotRunningException(std::string msg): runtime_error(msg.c_str()) {}
};

class CameraImageNotReadyException: public std::runtime_error
{
public:
  CameraImageNotReadyException(): runtime_error("Image is currently not ready.") {}
  CameraImageNotReadyException(std::string msg): runtime_error(msg.c_str()) {}
};

class CameraImageConsistencyError: public std::runtime_error
{
public:
  CameraImageConsistencyError(): runtime_error("Image consistency issue.") {}
  CameraImageConsistencyError(std::string msg): runtime_error(msg.c_str()) {}
};

#endif
