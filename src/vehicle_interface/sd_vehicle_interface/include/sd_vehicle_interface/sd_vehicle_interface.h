/*
 * Copyright (C) 2020 StreetDrone Limited - All rights reserved
 * 
 * Author: Fionán O'Sullivan
 * (co)Author: Abdelrahman Barghout
 *
 * Based on original work of: Efimia Panagiotaki
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
#include "aslan_msgs/SDControl.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
//*****CONSTANTS*****

//Constants
#define  RAD_to_DEG  (57.2958)			//Conversion constant from rad to deg
#define  DEG_to_RAD  (0.0174533)		//Conversion constant from deg to rad

//Ros Frequencies
#define ROS_LOOP (200) 		//The rate at which we publish data

//Control Frequencies 
#define CONTROL_LOOP (20) //The number of cycles that are counted for ever control loop cycle. 200/20 = 5Hz.
 
 //*****VARIABLES*****
class sd_vehicle_interface
{
public:
double CurrentTwistLinearCANImu_Mps = 0.0;		//Current Twist Linear in Mps, as read from the CAN bus from the GPS/IMU
double CurrentTwistLinearSD_Mps_Final = 0.0;	//The speed used within the control functions. Specified at launch wether this is based on vehicle can, IMU, or localisation.  
double CurrentTwistLinearCANSD_Mps = 0.0; 		//Current Twist Linear in Mps, as read from the CAN bus from the StreetDrone XCU
double CurrentTwistLinearNDT_Mps = 0.0;			//Current Twist Linear in Mps, as reported by NDT localisation
double GPS_Latitude = 0.0;								//latitude, as read from the CAN bus
double GPS_Longitude = 0.0;								//latitude, as read from the CAN bus
double IMU_Angle_X =0;
double IMU_Angle_Y =0;
double IMU_Angle_Z =0;
double IMU_Rate_X =0;
double IMU_Rate_Y =0;
double IMU_Rate_Z = 0;
double IMU_Accel_X =0;
double IMU_Accel_Y =0;
double IMU_Accel_Z = 0;
double TargetTwistLinear_Mps; 							//Target Twist linear in m/s
double TargetTwistAngular_Degps;						//Target Twist angular in deg/s
uint8_t AliveCounter_Z = 0;								//Alive Counter, increments every cycle
bool AutomationArmed_B = false;						//Boolean, true if safety driver turns mode switch to autonomous mode
bool AutomationGranted_B = false; 					//Boolean, true if vehicle grants autonomous mode request
bool IMUVarianceKnown_B = false; 					//Boolean, true if the inout GPS has known variance (OXTS YES, PEAK NO)


//Speed Control
int8_t FinalDBWTorqueRequest_Pc = 0; 				//The Final Drive-By-Wire torque request, expressed from -100% (full brake) to 100% (full throttle)
int8_t FinalDBWSteerRequest_Pc = 0; 				//Final steer request, +/- 100 is full lock left and right
int P_Contribution_Pc = 0; 									//The torque contributed by proportional gain
int I_Contribution_Pc = 0; 									//The torque contributed  by integral gain
int D_Contribution_Pc = 0; 									//The torque contributed by derivative gain
int FF_Contribution_Pc = 0; 								//The torque contributed by feedforward gain

//Ros variables
can_msgs::Frame ReceivedFrameCANRx;  		//Create the can frame that will be store received can messages
can_msgs::Frame CustomerControlCANTx; 		//Create the can frame that will be outputted onto the canbus
can_msgs::Frame ControllerFeedbackCANTx; 	//Create the frame that will supply feedback data to engineers tuning controller

ros::Publisher sent_msgs_pub;
ros::Publisher current_twist_pub;
ros::Publisher current_GPS_pub;
ros::Publisher current_IMU_pub;
ros::Publisher sd_control_pub;


std::string _sd_vehicle;
std::string _sd_gps_imu;
std::string _sd_speed_source; 
bool _sd_simulation_mode;

std::string twizy_string = "twizy";

std::string oxts_string = "oxts";
std::string peak_string = "peak";
std::string no_imu_string = "none";

std::string vehicle_can_speed_string = "vehicle_can_speed"; 
std::string imu_speed_string = "imu_speed"; 
std::string ndt_speed_string = "ndt_speed"; 

void ReceivedFrameCANRx_callback(const can_msgs::Frame::ConstPtr& msg); // Callback to listen to Can msgs
void TwistCommand_callback(const geometry_msgs::TwistStampedConstPtr &msg); // Callback to listen to twist msg and convert it to deg/sec twist
void CurrentVelocity_callback(const geometry_msgs::TwistStampedConstPtr &msg);// Callback to listen to NDT speed and convert it from mps to kph
double speedSource(std::string speed_source); // Function to decide if speed is determined by CAN, NDT, or IMU
sensor_msgs::NavSatFix GPS_setup(sensor_msgs::NavSatFix current_GPS);// Updating GPS signal
void Twist_setup(geometry_msgs::TwistStamped *current_Twist);// Updating twist signal
std::string AutonomousControl_setup();// Check if autonomous control is ready
std::string AutonomousControl_action(aslan_msgs::SDControl *SD_Current_Control); // Perform autonomous Control for car
std::string SpeedControl(aslan_msgs::SDControl *SD_Current_Control); // Control speed based on vehicle and other attributes
std::string ChooseVehicle();// Choose whether vehicle is twizy or env200
void publish_states(geometry_msgs::TwistStamped current_Twist, sensor_msgs::NavSatFix current_GPS, sensor_msgs::Imu current_IMU);// Publishing twist, GPS, and IMU msgs
int RUN(int argc, char **argv);
};
