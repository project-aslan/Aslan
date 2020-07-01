/*
 * Copyright (C) 2020 StreetDrone Limited - All rights reserved
 * 
 * Author: Fionán O'Sullivan
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
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
 *
 */

  
#include <can_msgs/Frame.h>
#include "sd_typedefs.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"

//Constants

//Variances calcualted from standard deviation in OXTS Manual
#define Orientation_X_Variance (2.74155703e-7)
#define Orientation_Y_Variance (2.74155703e-7)
#define Orientation_Z_Variance (0.00000304617)
#define Rate_X_Variance (3.04617332e-8)
#define Rate_Y_Variance (3.04617332e-8)
#define Rate_Z_Variance (3.04617332e-8)
#define Accel_X_Variance (2.4059025e-10)
#define Accel_Y_Variance (2.4059025e-10)
#define Accel_Z_Variance (2.4059025e-10)
#define Variance_Unkown (0) //Variance for PEAK not currently known

#define  DEG_to_RAD  (0.0174533)		//Conversion constant from deg to rad



using namespace std;

namespace sd{
	//Structs
	//Used for storing Quaternion
	struct Quaternion
	{
		double w, x, y, z;
	};

	//Functions

	//Converts from Euler to Quaternion format. 
	Quaternion ToQuaternion(double, double, double); // yaw (Z), pitch (Y), roll (X)



	void ParseRxCANDataOXTSCan(can_msgs::Frame&,
													double&,
													double&, double&, 
													double&,  double&,  double&,
													double&, double&, double&,
													double&, double&, double&);
														
		
	void ParseRxCANDataPEAKCan(can_msgs::Frame&,
													double&,
													double&, double&, 
													double&,  double&,  double&,
													double&, double&, double&,
													double&, double&, double&);
														
		
	void PackImuMessage(bool, sensor_msgs::Imu&, double, double, double, double, double, double, double, double, double);


}
