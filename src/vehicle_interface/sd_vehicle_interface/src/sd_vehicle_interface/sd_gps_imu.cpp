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

  
 #include "sd_vehicle_interface/sd_gps_imu.h"
 using namespace std;

namespace sd{


	//Functions

	//Converts from Euler to Quaternion format. 
	Quaternion ToQuaternion(double yaw, double pitch, double roll) // yaw (Z), pitch (Y), roll (X)
	{
		// Abbreviations for the various angular functions
		double cy = cos(yaw * 0.5);
		double sy = sin(yaw * 0.5);
		double cp = cos(pitch * 0.5);
		double sp = sin(pitch * 0.5);
		double cr = cos(roll * 0.5);
		double sr = sin(roll * 0.5);

		Quaternion q;
		q.w = cy * cp * cr + sy * sp * sr;
		q.x = cy * cp * sr - sy * sp * cr;
		q.y = sy * cp * sr + cy * sp * cr;
		q.z = sy * cp * cr - cy * sp * sr;

		return q;
	}

	void ParseRxCANDataOXTSCan(	can_msgs::Frame& ReceivedFrameCAN,
														double& CurrentLinearVelocity_Mps,
														double& GPS_longitude, double& GPS_latitude, 
														double& IMU_Angle_X,  double& IMU_Angle_Y,  double& IMU_Angle_Z,
														double& IMU_Rate_X, double& IMU_Rate_Y, double& IMU_Rate_Z,
														double& IMU_Accel_X, double& IMU_Accel_Y, double& IMU_Accel_Z)
														
		//This function parses the received can traffic from the OXTS unit. Please see OXTS documentation
		{
			CAN_frame_t ReceivedFrameUnion;
			ReceivedFrameUnion.byte[0] = ReceivedFrameCAN.data[0];
			ReceivedFrameUnion.byte[1] = ReceivedFrameCAN.data[1];
			ReceivedFrameUnion.byte[2] = ReceivedFrameCAN.data[2];
			ReceivedFrameUnion.byte[3] = ReceivedFrameCAN.data[3];
			ReceivedFrameUnion.byte[4] = ReceivedFrameCAN.data[4];
			ReceivedFrameUnion.byte[5] = ReceivedFrameCAN.data[5];
			ReceivedFrameUnion.byte[6] = ReceivedFrameCAN.data[6];
			ReceivedFrameUnion.byte[7] = ReceivedFrameCAN.data[7];
			
			
			switch (ReceivedFrameCAN.id){
			 case 1539: //0x603
				{
				CurrentLinearVelocity_Mps = ReceivedFrameUnion.word[3]*0.01;

					break;
				}
			case 1537: //0x601
				{
				GPS_longitude = ReceivedFrameUnion.dword[1]*0.0000001;
				GPS_latitude = ReceivedFrameUnion.dword[0]*0.0000001;
					
					break;
				}
			case 1541: //0x605
				{
				IMU_Accel_X = ReceivedFrameUnion.word[0]*0.01;
				IMU_Accel_Y = ReceivedFrameUnion.word[1]*0.01;
				IMU_Accel_Z = ReceivedFrameUnion.word[2]*0.01;
				 
					break;
				}
			case 1543: //0x607
				{
				IMU_Angle_X = ReceivedFrameUnion.word[0]*0.01;
				IMU_Angle_Y = ReceivedFrameUnion.word[1]*0.01;
				IMU_Angle_Z = ReceivedFrameUnion.word[2]*0.01;
				 
					break;
				}
			case 1544: //0x608
				{
				IMU_Rate_X = ReceivedFrameUnion.word[0]*0.01;
				IMU_Rate_Y = ReceivedFrameUnion.word[1]*0.01;
				IMU_Rate_Z = ReceivedFrameUnion.word[2]*0.01;
				 
					break;
				}

			}
		}
		
		
	void ParseRxCANDataPEAKCan(	can_msgs::Frame& ReceivedFrameCAN,
														double& CurrentLinearVelocity_Mps,
														double& GPS_longitude, double& GPS_latitude, 
														double& IMU_Angle_X,  double& IMU_Angle_Y,  double& IMU_Angle_Z,
														double& IMU_Rate_X, double& IMU_Rate_Y, double& IMU_Rate_Z,
														double& IMU_Accel_X, double& IMU_Accel_Y, double& IMU_Accel_Z)
														
		//This function parses the received can traffic from the OXTS unit. Please see OXTS documentation
		{
			CAN_frame_t ReceivedFrameUnion;
			ReceivedFrameUnion.byte[0] = ReceivedFrameCAN.data[0];
			ReceivedFrameUnion.byte[1] = ReceivedFrameCAN.data[1];
			ReceivedFrameUnion.byte[2] = ReceivedFrameCAN.data[2];
			ReceivedFrameUnion.byte[3] = ReceivedFrameCAN.data[3];
			ReceivedFrameUnion.byte[4] = ReceivedFrameCAN.data[4];
			ReceivedFrameUnion.byte[5] = ReceivedFrameCAN.data[5];
			ReceivedFrameUnion.byte[6] = ReceivedFrameCAN.data[6];
			ReceivedFrameUnion.byte[7] = ReceivedFrameCAN.data[7];
			
			
		switch (ReceivedFrameCAN.id){
			case 1536: //0x600
				{				
					IMU_Accel_X = ReceivedFrameUnion.word[1]*0.038344002; //*3.91 = mG, then conversion to m/s^2
					IMU_Accel_Y = ReceivedFrameUnion.word[1]*0.038344002;
					IMU_Accel_Z = ReceivedFrameUnion.word[2]*0.038344002;

					break;
				}			
			case 1552: //0x610
				{
					float_bits_converter IMU_Rate_X_fbc;
					float_bits_converter IMU_Rate_Y_fbc;				
						
					IMU_Rate_X_fbc.integer_can = ReceivedFrameUnion.dword[0];
					IMU_Rate_Y_fbc.integer_can = ReceivedFrameUnion.dword[1];
					IMU_Rate_X = IMU_Rate_X_fbc.float_can;
					IMU_Rate_Y = IMU_Rate_X_fbc.float_can;
					
					IMU_Angle_X += IMU_Rate_X*0.05; //Angle achieved by integrating rate over time (50 ms rate)
					IMU_Angle_Y += IMU_Rate_Y*0.05; //Angle achieved by integrating rate over time (50 ms rate)

					break;
				}			
			case 1553: //0x611
				{
					float_bits_converter IMU_Rate_Z_fbc;				

					IMU_Rate_Z_fbc.integer_can = ReceivedFrameUnion.dword[0];
					IMU_Rate_Z = IMU_Rate_Z_fbc.float_can;
					IMU_Angle_Z += IMU_Rate_Z*0.05; //Angle achieved by integrating rate over time (50 ms rate)
					break;
				}	
				
			 case 1569: //0x621
				{
					float_bits_converter CurrentLinearVelocity_Mps_fbc;
					float_bits_converter Course_Deg_fbc;
					
					CurrentLinearVelocity_Mps_fbc.integer_can = ReceivedFrameUnion.dword[1];
					CurrentLinearVelocity_Mps = CurrentLinearVelocity_Mps_fbc.float_can*KPH_to_MPS;
					
					Course_Deg_fbc.integer_can = ReceivedFrameUnion.dword[0];
					IMU_Angle_Z = Course_Deg_fbc.float_can; //This is a better measurement than integrated rate over time. 
					
				
					break;
				}
			case 1571: //0x623
				{
					float_bits_converter GPS_Latitude_Minutes_fbc;
					GPS_Latitude_Minutes_fbc.integer_can =  ReceivedFrameUnion.dword[0];

					GPS_latitude = (GPS_Latitude_Minutes_fbc.float_can/60) + ReceivedFrameUnion.word[2];

					//As per PEAK CAN GPS .dbc, 83 represents south.
					if (ReceivedFrameUnion.byte[6] == 83){
						GPS_latitude *= -1;
					}
				}
			case 1570: //0x623
				{
					float_bits_converter GPS_Longitude_Minutes_fbc;
					GPS_Longitude_Minutes_fbc.integer_can =  ReceivedFrameUnion.dword[0];

					GPS_latitude = (GPS_Longitude_Minutes_fbc.float_can/60) + ReceivedFrameUnion.word[2];

					//As per PEAK CAN GPS .dbc, 87 represents west.
					if (ReceivedFrameUnion.byte[6] == 87){
						GPS_latitude *= -1;
					}
				}
			}
		}
	
		
	void PackImuMessage(bool IMUVarianceKnown_B, sensor_msgs::Imu& current_IMU, double IMU_Angle_X, double  IMU_Angle_Y, double  IMU_Angle_Z, double  IMU_Rate_X, double  IMU_Rate_Y, double  IMU_Rate_Z, double  IMU_Accel_X, double  IMU_Accel_Y, double  IMU_Accel_Z)
	{
		geometry_msgs::Quaternion  Orientation_Quaternion;
		geometry_msgs::Vector3 AngularVelocity3D;
		geometry_msgs::Vector3 LinearAccel3D;
		
		Quaternion ConvertedQuaternion = ToQuaternion(IMU_Angle_X*DEG_to_RAD, IMU_Angle_Y*DEG_to_RAD, IMU_Angle_Z*DEG_to_RAD); //Covert angles to quaternion, uses rad/s
		Orientation_Quaternion.x = ConvertedQuaternion.x;
		Orientation_Quaternion.y = ConvertedQuaternion.y;
		Orientation_Quaternion.z = ConvertedQuaternion.z;
		Orientation_Quaternion.w = ConvertedQuaternion.w;
			
		AngularVelocity3D.x = IMU_Rate_X*DEG_to_RAD;
		AngularVelocity3D.y = IMU_Rate_Y*DEG_to_RAD;
		AngularVelocity3D.z = IMU_Rate_Z*DEG_to_RAD;
			
		LinearAccel3D.x = IMU_Accel_X*DEG_to_RAD;
		LinearAccel3D.y = IMU_Accel_Y*DEG_to_RAD;
		LinearAccel3D.z = IMU_Accel_Z*DEG_to_RAD;
		
		if(IMUVarianceKnown_B){
		
		current_IMU.orientation = Orientation_Quaternion;
		current_IMU.orientation_covariance = {Orientation_X_Variance, 0.0, 0.0,
																0.0, Orientation_Y_Variance, 0.0,
																0.0, 0.0,Orientation_Z_Variance};
																																	
		current_IMU.angular_velocity = AngularVelocity3D;
		current_IMU.angular_velocity_covariance = {Rate_X_Variance, 0.0, 0.0,
																			0.0, Rate_Y_Variance, 0.0,
																			0.0, 0.0, Rate_Z_Variance};
																			
		current_IMU.linear_acceleration = LinearAccel3D;
		current_IMU.linear_acceleration_covariance = {Accel_X_Variance, 0.0, 0.0,
																			0.0, Accel_Y_Variance, 0.0,
																			0.0, 0.0,Accel_X_Variance};
		}else{
			
				
		current_IMU.orientation = Orientation_Quaternion;
		current_IMU.orientation_covariance = {Variance_Unkown, 0.0, 0.0,
																0.0, Variance_Unkown, 0.0,
																0.0, 0.0,Variance_Unkown};
																																	
		current_IMU.angular_velocity = AngularVelocity3D;
		current_IMU.angular_velocity_covariance = {Variance_Unkown, 0.0, 0.0,
																			0.0, Variance_Unkown, 0.0,
																			0.0, 0.0, Variance_Unkown};
																			
		current_IMU.linear_acceleration = LinearAccel3D;
		current_IMU.linear_acceleration_covariance = {Variance_Unkown, 0.0, 0.0,
																			0.0, Variance_Unkown, 0.0,
																			0.0, 0.0,Variance_Unkown};
		}
	
	}

}
