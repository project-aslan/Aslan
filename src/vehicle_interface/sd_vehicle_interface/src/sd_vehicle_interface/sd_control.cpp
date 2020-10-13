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

#include "sd_vehicle_interface/sd_control.h"

namespace speedcontroller{
	
	uint8_t steer_map[YAW_YAXIS][V_XAXIS] =	
	{//0m/s	<1m/s	<2m/s	<3m/s	<4m/s	<5m/s	<6m/s	<7m/s	<8m/s	<9m/s
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0},	// 0 deg/s
	{40,	30,	20,	18,	11,	10,	7,	7,	6,	3},	// <5 deg/s
	{60,	50,	37,	30,	21,	13,	14,	12,	11,	7},	// <10 deg/s
	{95,	65,	49,	43,	34,	28,	21,	16,	14,	12},	// <15 deg/s
	{100,	97,	90,	77,	50,	35,	28,	23,	21,	17},	// <20 deg/s
	{100,	100,	100,	90,	55,	39,	35,	28,	25,	17},	// <25 deg/s
	{100,	100,	100,	100,	60,	50,	40,	30,	28,	17},	// <30 deg/s
	{100,	100,	100,	100,	65,	60,	45,	35,	32,	17},	// <35 deg/s
	{100,	100,	100,	100,	90,	70,	58,	42,	38,	17},	// <40 deg/s	
	{100,	100,	100,	100,	95,	85,	65,	48,	33,	17},	// <45 deg/s
	{100,	100,	100,	100,	100,	90,	70,	55,	45,	17},	// <50 deg/s
	{100,	100,	100,	100,	100,	95,	80,	60,	50,	19},	// <55 deg/s
	{100,	100,	100,	100,	100,	100,	90,	65,	53,	20},	// <60 deg/s
	{100,	100,	100,	100,	100,	100,	100,	70,	55,	45},	// >60 deg/s //Map should be extended if tighter turns needed above 6m/s
	};

	//FeedForward Calibration
	//Map is used to provide feedforward torque (% from 0-100) for a given speed. An additional axis should be considered for gradient
	//Note, the FF gain at exactly 0 will be BRAKE_HOLD_TORQUE
	int8_t feedforward_torque_map_twizy[V_XAXIS]  = 	
	//>0m/s	<1m/s	<2m/s	<3m/s	<4m/s	<5m/s	<6m/s	<7m/s	<8m/s	<9m/s
	{33,	33,	33,	34,	35,	38,	39,	40,	41,	42};
	
	int8_t feedforward_torque_map_env200[V_XAXIS]  = 	
	//>0m/s	<1m/s	<2m/s	<3m/s	<4m/s	<5m/s	<6m/s	<7m/s	<8m/s	<9m/s
	{-5,	0,	2,	6,	7,	8,	10,	12,	13,	15};


	//This function calculates the steer angle that will achieve a given angular velocity at a given speed
    int8_t CalculateSteerRequest(double TargetAngularVelocity_Dps, double CurrentTwistLinear_Mps){
		
		//For velocitys inbetween match points e.g 1.5 m/s, both the steer angles from the 1m/s and 2m/s
		//column are calculated, the final steer linearly interpolated between them. 
		double SteeringAngleTable_Pc_Speed_1; 
		double SteeringAngleTable_Pc_Speed_2;
		int CalculatedSteeringAngle_Pc; //Final Output steer 


		//Yaw index and remainder used to interpolate between different yaw values of the steer table
		int yaw_index = floor(abs((TargetAngularVelocity_Dps)/YAW_STEPS));
		double yaw_index_remainder=abs((TargetAngularVelocity_Dps)/YAW_STEPS) - yaw_index;
		
		//Speed index and remainder used to interpolate between different speed values of the steer table
		int speed_index = floor(CurrentTwistLinear_Mps);
		double speed_index_remainder=abs(CurrentTwistLinear_Mps) - speed_index;
					
		//handles inputs outside calibrated map size (high speeds and high yaw request)
		if(speed_index > V_XAXIS -2){
			speed_index = V_XAXIS -2;
		}
		if(yaw_index > YAW_YAXIS -2){
			yaw_index = YAW_YAXIS -2;
		}

		//yaw interpolation handled below
		SteeringAngleTable_Pc_Speed_1 = (1-yaw_index_remainder)*steer_map[yaw_index][speed_index] + yaw_index_remainder*steer_map[yaw_index+1][speed_index];
		SteeringAngleTable_Pc_Speed_2 = (1-yaw_index_remainder)*steer_map[yaw_index][speed_index+1] + yaw_index_remainder*steer_map[yaw_index+1][speed_index+1];
		
		//Interpolate for speed
		CalculatedSteeringAngle_Pc = (1-speed_index_remainder)*SteeringAngleTable_Pc_Speed_1 +  speed_index_remainder*(SteeringAngleTable_Pc_Speed_2);
		
		//Flip sign convention (Positive Yaw request achieved by negative steering Angle in StreetDrone protocol)
		if(TargetAngularVelocity_Dps <= 0){
			CalculatedSteeringAngle_Pc = CalculatedSteeringAngle_Pc*-1;
		}

		//If there is not angular velocity request, steering angle is set to 0
		if (0 == TargetAngularVelocity_Dps){
			CalculatedSteeringAngle_Pc = 0;
			
		//If there is no linear velocity request, steering angle is set to 0
		} else if (CurrentTwistLinear_Mps <= 0){
			CalculatedSteeringAngle_Pc = 0;
		}

			//Saturate to Min/Max value
		if (CalculatedSteeringAngle_Pc > MAX_STEER_ANG){
				CalculatedSteeringAngle_Pc = MAX_STEER_ANG;
		} else if(CalculatedSteeringAngle_Pc < MIN_STEER_ANG){
			CalculatedSteeringAngle_Pc = MIN_STEER_ANG;
		}

		return CalculatedSteeringAngle_Pc;
    }

	int8_t CalculateTorqueRequestTwizy(double TargetLinearVelocity_Mps, double CurrentLinearVelocity_Mps, int& P_Contribution_Pc, int& I_Contribution_Pc, int& D_Contribution_Pc, int& FF_Contribution_Pc){
        
		//Calculate PID Errors
		static double LinearVelocityError_Mps;
		static double LinearVelocityIntegratedError;
		static double LinearVelocityDerivativeError;
		int FinalDBWTorqueRequest_Pc; //Variable for final torque value
		
		//Speed index and remainder used to interpolate between different speed values of the steer table
		int speed_index = floor(TargetLinearVelocity_Mps);
		double speed_index_remainder=abs(TargetLinearVelocity_Mps) - speed_index;
		
		if(speed_index > V_XAXIS -2){
			speed_index = V_XAXIS -2;
		}

						
		//Reverse currently not suppported. If a negative speed request is received, we clamp it to 0 (standstill)
		if (TargetLinearVelocity_Mps <= 0) {
			TargetLinearVelocity_Mps = 0;
			FF_Contribution_Pc = 0;
		}else{
			//Calculate Feedforward contribution
			FF_Contribution_Pc  = (1-speed_index_remainder)*feedforward_torque_map_twizy[speed_index] + speed_index_remainder*feedforward_torque_map_twizy[speed_index+1];
		}
					
		LinearVelocityError_Mps = TargetLinearVelocity_Mps - CurrentLinearVelocity_Mps; 				//The error between current speed and target speed
		LinearVelocityDerivativeError = LinearVelocityError_Mps - PreviousLinearVelocityError_Mps; 	//The difference between current error and error on last cycle
		LinearVelocityIntegratedError = LinearVelocityIntegratedError + LinearVelocityError_Mps; 			//The accumulated error over time
		
		if ((TargetLinearVelocity_Mps == 0 && CurrentLinearVelocity_Mps == 0)){
				
			P_Contribution_Pc = 0;
			I_Contribution_Pc = 0;
			D_Contribution_Pc =0;
			FF_Contribution_Pc = BRAKE_HOLD_TORQUE_TWIZY; //If we currently have  a speed target of 0 (standstill), and are not moving, lightly hold the vehicle still by gently braking
			
		}  else if (abs(LinearVelocityError_Mps) < ANTI_FUSSINESS_TWIZY){
			
			//PID remain the same. I gain = 0. This resets the I gain so we don't have to "unwind"
			LinearVelocityIntegratedError = 0;
			I_Contribution_Pc = 0;
			//P, D and FF fails maintain last value
			
		} else if ((TargetLinearVelocity_Mps == 0 && CurrentLinearVelocity_Mps > 0)){ //Use braking gains if we wish to slow down to a standstill (Emergency stop or final stop). 
		   
			P_Contribution_Pc = LinearVelocityError_Mps * Kp_Speed_FullStop_Braking_Twizy;
			I_Contribution_Pc = LinearVelocityIntegratedError * Ki_Speed_FullStop_Braking_Twizy;
			D_Contribution_Pc = LinearVelocityDerivativeError * Kd_Speed_FullStop_Braking_Twizy;
			
		}else if (LinearVelocityError_Mps < - ANTI_FUSSINESS_TWIZY){ //When we are going too fast, we reduce speed with a different set of gains. (this allows us to account for vehicle overrun/coasting)
			
			P_Contribution_Pc = LinearVelocityError_Mps * Kp_Speed_Retd_Twizy;
			I_Contribution_Pc = LinearVelocityIntegratedError * Ki_Speed_Retd_Twizy;
			D_Contribution_Pc = LinearVelocityDerivativeError * Kd_Speed_Retd_Twizy;
			
		} else { //else use the calculated errors
		
			P_Contribution_Pc = LinearVelocityError_Mps * Kp_Speed_Twizy;
			I_Contribution_Pc = LinearVelocityIntegratedError * Ki_Speed_Twizy;
			D_Contribution_Pc = LinearVelocityDerivativeError * Kd_Speed_Twizy;
		}
				
		//I gain Anti windup Strategy
		if (I_Contribution_Pc > MAX_ABS_I_CONTRIBUTION_TWIZY){  //I Gain saturation
			I_Contribution_Pc = MAX_ABS_I_CONTRIBUTION_TWIZY;
		} else if (I_Contribution_Pc < -MAX_ABS_I_CONTRIBUTION_TWIZY){
			I_Contribution_Pc = - MAX_ABS_I_CONTRIBUTION_TWIZY;
		}
		
		if(CurrentLinearVelocity_Mps < ANTI_FUSSINESS_TWIZY){ //Prevents I gain winding up when sitting still with handbrake on
		I_Contribution_Pc = 0;
		}
		
		if(abs(LinearVelocityError_Mps) > I_GAIN_ERROR_BAND_TWIZY){ //I Gain should only influence the system in a band about the setpoint. 
			LinearVelocityIntegratedError = 0;
			I_Contribution_Pc = 0;
		}
		
		
		FinalDBWTorqueRequest_Pc = P_Contribution_Pc + I_Contribution_Pc + D_Contribution_Pc + FF_Contribution_Pc;

			
		//Impose limits on the torque if greater than or less than maximum and minimum values
		if (FinalDBWTorqueRequest_Pc > MAX_TORQUE_TWIZY){
			FinalDBWTorqueRequest_Pc = MAX_TORQUE_TWIZY;
		} else if(FinalDBWTorqueRequest_Pc < MIN_TORQUE_TWIZY){
			FinalDBWTorqueRequest_Pc = MIN_TORQUE_TWIZY;
		}
		
		//Remember previous variables for next cycle
		PreviousLinearVelocityError_Mps = LinearVelocityError_Mps;
		
		return (int8_t)FinalDBWTorqueRequest_Pc;

    }
	
		int8_t CalculateTorqueRequestEnv200(double TargetLinearVelocity_Mps, double CurrentLinearVelocity_Mps, int& P_Contribution_Pc, int& I_Contribution_Pc, int& D_Contribution_Pc, int& FF_Contribution_Pc){
        
		//Calculate PID Errors
		static double LinearVelocityError_Mps;
		static double LinearVelocityIntegratedError;
		static double LinearVelocityDerivativeError;
		int FinalDBWTorqueRequest_Pc; //Variable for final torque value
		
		//Speed index and remainder used to interpolate between different speed values of the steer table
		int speed_index = floor(TargetLinearVelocity_Mps);
		double speed_index_remainder=abs(TargetLinearVelocity_Mps) - speed_index;
		
		if(speed_index > V_XAXIS -2){
			speed_index = V_XAXIS -2;
		}

						
		//Reverse currently not suppported. If a negative speed request is received, we clamp it to 0 (standstill)
		if (TargetLinearVelocity_Mps <= 0) {
			TargetLinearVelocity_Mps = 0;
			FF_Contribution_Pc = 0;
		}else{
			//Calculate Feedforward contribution
			FF_Contribution_Pc  = (1-speed_index_remainder)*feedforward_torque_map_env200[speed_index] + speed_index_remainder*feedforward_torque_map_env200[speed_index+1];
		}
					
		LinearVelocityError_Mps = TargetLinearVelocity_Mps - CurrentLinearVelocity_Mps; 				//The error between current speed and target speed
		LinearVelocityDerivativeError = LinearVelocityError_Mps - PreviousLinearVelocityError_Mps; 	//The difference between current error and error on last cycle
		LinearVelocityIntegratedError = LinearVelocityIntegratedError + LinearVelocityError_Mps; 			//The accumulated error over time
		
		if ((TargetLinearVelocity_Mps == 0 && CurrentLinearVelocity_Mps == 0)){
				
			P_Contribution_Pc = 0;
			I_Contribution_Pc = 0;
			D_Contribution_Pc =0;
			FF_Contribution_Pc = BRAKE_HOLD_TORQUE_ENV200; //If we currently have  a speed target of 0 (standstill), and are not moving, lightly hold the vehicle still by gently braking
			
		}  else if (abs(LinearVelocityError_Mps) < ANTI_FUSSINESS_ENV200){
			
			//PID remain the same. I gain = 0. This resets the I gain so we don't have to "unwind"
			LinearVelocityIntegratedError = 0;
			I_Contribution_Pc = 0;
			//P, D and FF fails maintain last value
			
		} else if ((TargetLinearVelocity_Mps == 0 && CurrentLinearVelocity_Mps > 0)){ //Use braking gains if we wish to slow down to a standstill (Emergency stop or final stop). 
		   
			P_Contribution_Pc = LinearVelocityError_Mps * Kp_Speed_FullStop_Braking_Env200;
			I_Contribution_Pc = LinearVelocityIntegratedError * Ki_Speed_FullStop_Braking_Env200;
			D_Contribution_Pc = LinearVelocityDerivativeError * Kd_Speed_FullStop_Braking_Env200;
			
		}else if (LinearVelocityError_Mps < - ANTI_FUSSINESS_ENV200){ //When we are going too fast, we reduce speed with a different set of gains. (this allows us to account for vehicle overrun/coasting)
			
			P_Contribution_Pc = LinearVelocityError_Mps * Kp_Speed_Retd_Env200;
			I_Contribution_Pc = LinearVelocityIntegratedError * Ki_Speed_Retd_Env200;
			D_Contribution_Pc = LinearVelocityDerivativeError * Kd_Speed_Retd_Env200;
			
		} else { //else use the calculated errors
		
			P_Contribution_Pc = LinearVelocityError_Mps * Kp_Speed_Env200;
			I_Contribution_Pc = LinearVelocityIntegratedError * Ki_Speed_Env200;
			D_Contribution_Pc = LinearVelocityDerivativeError * Kd_Speed_Env200;
		}
				
		//I gain Anti windup Strategy
		if (I_Contribution_Pc > MAX_ABS_I_CONTRIBUTION_ENV200){  //I Gain saturation
			I_Contribution_Pc = MAX_ABS_I_CONTRIBUTION_ENV200;
		} else if (I_Contribution_Pc < -MAX_ABS_I_CONTRIBUTION_ENV200){
			I_Contribution_Pc = - MAX_ABS_I_CONTRIBUTION_ENV200;
		}
		
		if(CurrentLinearVelocity_Mps < ANTI_FUSSINESS_ENV200){ //Prevents I gain winding up when sitting still with handbrake on
		I_Contribution_Pc = 0;
		}
		
		if(abs(LinearVelocityError_Mps) > I_GAIN_ERROR_BAND_ENV200){ //I Gain should only influence the system in a band about the setpoint. 
			LinearVelocityIntegratedError = 0;
			I_Contribution_Pc = 0;
		}
		
		
		FinalDBWTorqueRequest_Pc = P_Contribution_Pc + I_Contribution_Pc + D_Contribution_Pc + FF_Contribution_Pc;

			
		//Impose limits on the torque if greater than or less than maximum and minimum values
		if (FinalDBWTorqueRequest_Pc > MAX_TORQUE_ENV200){
			FinalDBWTorqueRequest_Pc = MAX_TORQUE_ENV200;
		} else if(FinalDBWTorqueRequest_Pc < MIN_TORQUE_ENV200){
			FinalDBWTorqueRequest_Pc = MIN_TORQUE_ENV200;
		}
		
		//Remember previous variables for next cycle
		PreviousLinearVelocityError_Mps = LinearVelocityError_Mps;
		
		return (int8_t)FinalDBWTorqueRequest_Pc;

    }

}
