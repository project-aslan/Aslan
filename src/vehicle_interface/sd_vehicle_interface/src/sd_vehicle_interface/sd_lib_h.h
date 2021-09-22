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


#include <can_msgs/Frame.h>

namespace sd{
	//**************************************************
	/*				SD RX FUNCTIONS				*/
	//**************************************************
	
	
	/*ParseRxCanDataSDCan
	This function parses an input can frame, checks it's ID and updates appropriate variables with freshest data*/
    void ParseRxCANDataSDCan(can_msgs::Frame&, double& , bool& , bool&);
	/*Inputs
	can_msgs::Frame& ReceivedFrameCAN : Raw socketcan frame in format of can_msgs/Frame.h, this is the RX can stream
	double& CurrentLinearVelocity_Mps The function shall update this variable with the latest Linear Velocity as read from the CAN bus in Mps
	bool& AutomationGranted_B,  This function shall set this variable to TRUE if the latest received can data condirms vehicle is in Automated Mode
	bool& AutomationArmed_B  This function shall set this variable to TRIE if the latest received CAN data confirms that the vehicle is armed for autonomous mode*/
	
	//**************************************************
	/*				SD TX FUNCTIONS				*/
	//**************************************************
	
	/*
	InitSDInterfaceControl & InitSDInterfaceFeedback
	These functions  assign the appriiate CAN IDs for the Control and Feedback frames of the StreetDrone CAN protocol
	They should be run on a socketcan frame in format of can_msgs/Frame.h
	Perform these function on initialisation before main loop
	The Feedback message is optional*/
    void InitSDInterfaceControl(can_msgs::Frame&);
	void InitSDInterfaceFeedback(can_msgs::Frame&);
	/*Inputs
	can_msgs::Frame& CustomerControlCANTx/CustomerFeedbackCANTx to be initialised*/
	
	//Request Autonomous Control of the Vehicle
    void RequestAutonomousControl(can_msgs::Frame&,  uint8_t);
	/*inputs
	can_msgs::Frame& CustomerControlCANTx :The SD Interface Control Message after initialisation
	uint8_t AliveCounter_Z : An Alive counter. Increment this variable by 1 each loop. Loop must run at minimum 200Hz. Protects again stale CAN data*/

	//ResetControlCanData
	//This function resets all but the alive counter to 0
	//This will handback control to the safety driver
    void ResetControlCanData(can_msgs::Frame&, uint8_t);
	void UpdateControlAlive(can_msgs::Frame&, uint8_t);

	/*Inputs
	can_msgs::Frame& CustomerControlCANTx :The SD Interface Control Message after initialisation
	uint8_t AliveCounter_Z : An Alive counter. Increment this variable by 1 each loop. Loop must run at minimum 200Hz. Protects again stale CAN data*/

	//PopControlCANData
	//Populates the Control Tx message to the vehicle
    void PopControlCANData(can_msgs::Frame&, int8_t, int8_t, uint8_t);
	/*Inputs
	can_msgs::Frame& CustomerControlCANTx  The SD Interface Control Message after initialisation
	int8_t FinalDBWTorqueRequest_Pc:The Torque percentage requested of the vehicle
	int8_t FinalDBWSteerRequest_Pc: The Steer Percentage requested of the vehicle
	uint8_t AliveCounter_Z  : An Alive counter. Increment this variable by 1 each loop. Loop must run at minimum 200Hz. Protects again stale CAN data*/
	
	//Populates the Feedback CAN message (Optional)
    void PopFeedbackCANData(can_msgs::Frame&, int, int, int, int, double, double);
	/*Inputs
	void can_msgs::Frame& ControllerFeedbackCANTx
	int P_Contribution_Pc :The Contribution to final torque by the P term
	int I_Contribution_Pc :The Contribution to final torque by the I term
	int D_Contribution_Pc :The Contribution to final torque by the D term
	int FF_Contribution_Pc :The Contribution to final torque by Feedorward control
	double TargetLinearVelocity_Mps :The Target speed (feedback only)
	double TargetAngularVelocity_Degps: The Target Angular velocity (Feedback Only)*/

}

