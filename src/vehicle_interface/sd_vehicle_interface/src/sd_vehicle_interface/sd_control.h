/*
 * Copyright (C) 2020 StreetDrone Limited - All rights reserved
 * 
 * Author: Fionán O'Sullivan
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

using namespace std;

#include <cmath>
#include <stdint.h>

namespace speedcontroller{


	static double PreviousLinearVelocityError_Mps = 0;		//The linear velocity error from the last cycle

	//Calibrations
	//The Proportional, Integral and Derivative gains of the linear velocity feedback loop are set here. 
	//It is possible to set different gains for occasions where the vehicle is braking to a stop, braking and accellerating

	//*****TWIZY CALIBRATIONS*****
	//Gains for increasing speed
	#define Kp_Speed_Twizy                    (23)
	#define Ki_Speed_Twizy                    (1)
	#define Kd_Speed_Twizy                    (0)

	//Speed Control Anti-fusinees Band (+/- band of target where we maintain torque)
	#define ANTI_FUSSINESS_TWIZY (0.1)

	//Gains for braking to a stop only
	#define Kp_Speed_FullStop_Braking_Twizy            (65)
	#define Ki_Speed_FullStop_Braking_Twizy             (0)
	#define Kd_Speed_FullStop_Braking_Twizy            (0)

	//Gains for reducing speed, not to a stop
	#define Kp_Speed_Retd_Twizy          (40)
	#define Ki_Speed_Retd_Twizy           (1)
	#define Kd_Speed_Retd_Twizy          (0)


	//Maximum and minimum torque requests
	#define MAX_TORQUE_TWIZY  (100)
	#define MIN_TORQUE_TWIZY    (-100)
	#define MAX_ABS_I_CONTRIBUTION_TWIZY  (15)
	#define I_GAIN_ERROR_BAND_TWIZY (2) //The absolute error about a setpoint where I gain is allowed to accumulate. 


	//Other control calibratables
	#define BRAKE_HOLD_TORQUE_TWIZY (-10) //Initial torque needed to get the vehicle moving

	//*****
	//*****ENV200 CALIBRATIONS
		//Gains for increasing speed
	#define Kp_Speed_Env200                    (7)
	#define Ki_Speed_Env200                    (0.6)
	#define Kd_Speed_Env200                    (0)

	//Speed Control Anti-fusinees Band (+/- band of target where we maintain torque)
	#define ANTI_FUSSINESS_ENV200 (0.1)

	//Gains for braking to a stop only
	#define Kp_Speed_FullStop_Braking_Env200            (20)
	#define Ki_Speed_FullStop_Braking_Env200            (0)
	#define Kd_Speed_FullStop_Braking_Env200            (0)

	//Gains for reducing speed, not to a stop
	#define Kp_Speed_Retd_Env200          (15)
	#define Ki_Speed_Retd_Env200           (1)
	#define Kd_Speed_Retd_Env200          (0)


	//Maximum and minimum torque requests
	#define MAX_TORQUE_ENV200  (35)
	#define MIN_TORQUE_ENV200    (-100)
	#define MAX_ABS_I_CONTRIBUTION_ENV200  (7)
	#define I_GAIN_ERROR_BAND_ENV200 (2) //The absolute error about a setpoint where I gain is allowed to accumulate. 

	//Other control calibratables
	#define BRAKE_HOLD_TORQUE_ENV200 (-50) //Initial torque needed to get the vehicle moving
	//******
	//*****SHARED CALIBRATIONS
	//Max and min steer angle as percentage
	#define MAX_STEER_ANG   (100)
	#define MIN_STEER_ANG    (-100)
	
	//Yaw Map Calibrations
	#define V_XAXIS    (10) //If extending map, increase these constants to match
	
	#define YAW_YAXIS  (14)
	#define YAW_STEPS (5) //At which discrete steps is each point on the yaw axis, defult 5 deg/s steps
	
	//******

	//The below table represents a feedforward control yaw to steering table, based on input yaw and velocity
	//The variables below are fully tunable, and inbetween values of speed and yaw shall interpolate between map entries

	extern uint8_t steer_map[YAW_YAXIS][V_XAXIS];

	//FeedForward Calibration
	//Map is used to provide feedforward torque (% from 0-100) for a given speed. An additional axis should be considered for gradient
	//Note, the FF gain at exactly 0 will be BRAKE_HOLD_TORQUE
	extern int8_t feedforward_torque_map_twizy[V_XAXIS];
	extern int8_t feedforward_torque_map_env200[V_XAXIS];


	//This function calculates the steer angle that will achieve a given angular velocity at a given speed
    int8_t CalculateSteerRequest(double, double);
	/*Inputs
	double TargetAngularVelocity_Dps: The target Angular Velocity in Degrees per Second
	double CurrentTwistLinear_Mps: The current linear velocity in Metres per Second
	Outputs
	int8_t FinalDBWSteerRequest_Pc: The steer angle request, expressed as a percentage from full lock left/right (+/- 100%). 0 being centred steering. */
	
	//This function calculates the torque using PID and feedforward control
	int8_t CalculateTorqueRequestTwizy(double, double, int&, int&, int&, int&);
    /*Inputs
	string _sd_vehicle: The StreetDrone vehicle under control, 'twizy' or 'env200'
	double TargetLinearVelocity_Mps: The target linear velocity in Metres per Second
	double CurrentTwistLinear_Mps: The current linear velocity in Metres per Second
	double TargetAngularVelocity_Dps: The target Angular Velocity in Degrees per Second
	int P_Contribution_Pc: The contribution to final torque given by the P term, used for user feedback for tuning
	int I_Contribution_Pc: The contribution to final torque given by the I term, used for user feedback for tuning
	int D_Contribution_Pc: The contribution to final torque given by the D term, used for user feedback for tuning
	int FF_Contribution_Pc: The contribution to final torque given by the feedforward calculation, used for user feedback for tuning

	Outputs
	int8_t FinalDBWTorqueRequest_Pc: The torque request, expressed as a percentage of full braking vs full throttle (+/- 100%). 0 being no torque request.*/
	
		//This function calculates the torque using PID and feedforward control
	int8_t CalculateTorqueRequestEnv200(double, double, int&, int&, int&, int&);
    /*Inputs
	string _sd_vehicle: The StreetDrone vehicle under control, 'twizy' or 'env200'
	double TargetLinearVelocity_Mps: The target linear velocity in Metres per Second
	double CurrentTwistLinear_Mps: The current linear velocity in Metres per Second
	double TargetAngularVelocity_Dps: The target Angular Velocity in Degrees per Second
	int P_Contribution_Pc: The contribution to final torque given by the P term, used for user feedback for tuning
	int I_Contribution_Pc: The contribution to final torque given by the I term, used for user feedback for tuning
	int D_Contribution_Pc: The contribution to final torque given by the D term, used for user feedback for tuning
	int FF_Contribution_Pc: The contribution to final torque given by the feedforward calculation, used for user feedback for tuning

	Outputs
	int8_t FinalDBWTorqueRequest_Pc: The torque request, expressed as a percentage of full braking vs full throttle (+/- 100%). 0 being no torque request.*/
	
}
