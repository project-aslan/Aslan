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
using namespace std;


#include <ros/ros.h>
#include "aslan_msgs/SDControl.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3.h"
#include <can_msgs/Frame.h>
#include <iostream>
#include "sd_vehicle_interface/sd_vehicle_interface.h"
#include "sd_vehicle_interface/sd_lib_h.h"
#include "sd_vehicle_interface/sd_gps_imu.h"
#include "sd_vehicle_interface/sd_control.h"

//Callback Functions
void sd_vehicle_interface::ReceivedFrameCANRx_callback(const can_msgs::Frame::ConstPtr& msg)
{
    //Populates into sd_vehicle_interface::ReceivedFrameCANRx the latest can message
    sd_vehicle_interface::ReceivedFrameCANRx = *msg.get();
    sd::ParseRxCANDataSDCan(sd_vehicle_interface::ReceivedFrameCANRx, sd_vehicle_interface::CurrentTwistLinearCANSD_Mps, sd_vehicle_interface::AutomationArmed_B, sd_vehicle_interface::AutomationGranted_B);

    if( sd_vehicle_interface::oxts_string == sd_vehicle_interface::_sd_gps_imu){

        sd_vehicle_interface::IMUVarianceKnown_B = true; //variance/Covariance known for OXTS
        //Use the OXTS parsing function
        sd::ParseRxCANDataOXTSCan(sd_vehicle_interface::ReceivedFrameCANRx, sd_vehicle_interface::CurrentTwistLinearCANImu_Mps, sd_vehicle_interface::GPS_Longitude, sd_vehicle_interface::GPS_Latitude, sd_vehicle_interface::IMU_Angle_X, sd_vehicle_interface::IMU_Angle_Y, sd_vehicle_interface::IMU_Angle_Z, sd_vehicle_interface::IMU_Rate_X, sd_vehicle_interface::IMU_Rate_Y, sd_vehicle_interface::IMU_Rate_Z, sd_vehicle_interface::IMU_Accel_X, sd_vehicle_interface::IMU_Accel_Y, sd_vehicle_interface::IMU_Accel_Z);
    }else if( sd_vehicle_interface::peak_string== sd_vehicle_interface::_sd_gps_imu){
        //Use the PEAK parsing Function
        sd_vehicle_interface::IMUVarianceKnown_B = false; //variance/Covariance not known for PEAK
        sd::ParseRxCANDataPEAKCan(sd_vehicle_interface::ReceivedFrameCANRx, sd_vehicle_interface::CurrentTwistLinearCANImu_Mps, sd_vehicle_interface::GPS_Longitude, sd_vehicle_interface::GPS_Latitude, sd_vehicle_interface::IMU_Angle_X, sd_vehicle_interface::IMU_Angle_Y, sd_vehicle_interface::IMU_Angle_Z, sd_vehicle_interface::IMU_Rate_X, sd_vehicle_interface::IMU_Rate_Y, sd_vehicle_interface::IMU_Rate_Z, sd_vehicle_interface::IMU_Accel_X, sd_vehicle_interface::IMU_Accel_Y, sd_vehicle_interface::IMU_Accel_Z);
    }else if( sd_vehicle_interface::no_imu_string== sd_vehicle_interface::_sd_gps_imu){
        //Do nothing
    }else{
        ROS_WARN("SD_Vehicle_Interface parameter for sd_gps_imu is not valid");
    }
}


void sd_vehicle_interface::TwistCommand_callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    //Populate a twist angular and twist linear message with the received message from Ros topic and convert to deg/s
    sd_vehicle_interface::TargetTwistAngular_Degps= (msg->twist.angular.z) * RAD_to_DEG;
    sd_vehicle_interface::TargetTwistLinear_Mps = msg->twist.linear.x;
}

void sd_vehicle_interface::CurrentVelocity_callback(const geometry_msgs::TwistStampedConstPtr &msg)
{
    //Current Velocity Reported from NDT
    sd_vehicle_interface::CurrentTwistLinearNDT_Mps = msg->twist.linear.x; //mps to kph
}

double sd_vehicle_interface::speedSource(string speed_source)
{
    if ( sd_vehicle_interface::ndt_speed_string == speed_source) return sd_vehicle_interface::CurrentTwistLinearNDT_Mps;
    else if ( sd_vehicle_interface::imu_speed_string == speed_source) return sd_vehicle_interface::CurrentTwistLinearCANImu_Mps; //Use the IMU speed source
    else if ( sd_vehicle_interface::vehicle_can_speed_string == speed_source) return sd_vehicle_interface::CurrentTwistLinearCANSD_Mps;
    else ROS_WARN("SD_Vehicle_Interface parameter for sd_speed_source is not valid");
    return 0;
}

sensor_msgs::NavSatFix sd_vehicle_interface::GPS_setup(sensor_msgs::NavSatFix current_GPS)
{
    current_GPS.longitude = sd_vehicle_interface::GPS_Longitude;
    current_GPS.latitude = sd_vehicle_interface::GPS_Latitude;
    return current_GPS;
}

void sd_vehicle_interface::Twist_setup(geometry_msgs::TwistStamped *current_Twist)
{
    current_Twist->twist.angular.z = sd_vehicle_interface::IMU_Rate_Z * DEG_to_RAD;
    current_Twist->twist.linear.z = sd_vehicle_interface::CurrentTwistLinearSD_Mps_Final;
}

std::string sd_vehicle_interface::AutonomousControl_setup()
{
    sd::UpdateControlAlive(sd_vehicle_interface::CustomerControlCANTx, sd_vehicle_interface::AliveCounter_Z); //Otherwise, populate the can frame with 0's

    if (0 ==(sd_vehicle_interface::AliveCounter_Z % CONTROL_LOOP)){ //We only run as per calibrated frequency

        if (sd_vehicle_interface::AutomationArmed_B){
            sd::RequestAutonomousControl(sd_vehicle_interface::CustomerControlCANTx, sd_vehicle_interface::AliveCounter_Z); //If the safety driver has armed the vehicle for autonomous, request autonomous control of torque and steer
            return "Autonomous";
        }else{
            sd::ResetControlCanData(sd_vehicle_interface::CustomerControlCANTx, sd_vehicle_interface::AliveCounter_Z); //Otherwise, populate the can frame with 0's
            return "Manual";
        }
    }
    return "FREQUENCY_ERROR";
}

std::string sd_vehicle_interface::AutonomousControl_action(aslan_msgs::SDControl *SD_Current_Control)
{
    if (sd_vehicle_interface::AutomationGranted_B ||  sd_vehicle_interface::_sd_simulation_mode){
        sd_vehicle_interface::SpeedControl(SD_Current_Control);
        //Populate the Can frames with calculated data
        sd::PopControlCANData(sd_vehicle_interface::CustomerControlCANTx, sd_vehicle_interface::FinalDBWTorqueRequest_Pc, sd_vehicle_interface::FinalDBWSteerRequest_Pc, sd_vehicle_interface::AliveCounter_Z);
        sd::PopFeedbackCANData(sd_vehicle_interface::ControllerFeedbackCANTx, sd_vehicle_interface::P_Contribution_Pc, sd_vehicle_interface::I_Contribution_Pc, sd_vehicle_interface::D_Contribution_Pc, sd_vehicle_interface::FF_Contribution_Pc, sd_vehicle_interface::TargetTwistLinear_Mps, sd_vehicle_interface::TargetTwistAngular_Degps);
        return "Autonomous";
    }
    return "Manual";
}

std::string sd_vehicle_interface::SpeedControl(aslan_msgs::SDControl *SD_Current_Control)
{
    if (0 ==(sd_vehicle_interface::AliveCounter_Z % CONTROL_LOOP)){ //We only run as per calibrated frequency

        //Calculate Steer and torque values, as well as controll feedback (PID and FeedForward Contributions to Torque Controller)
        sd_vehicle_interface::FinalDBWSteerRequest_Pc   = speedcontroller::CalculateSteerRequest  (sd_vehicle_interface::TargetTwistAngular_Degps, sd_vehicle_interface::CurrentTwistLinearSD_Mps_Final);
        sd_vehicle_interface::ChooseVehicle();
        // cout << sd_vehicle_interface::_sd_vehicle <<" TwistAngular " <<  setw(8) << sd_vehicle_interface::TargetTwistAngular_Degps << " Steer " <<  setw(8) << (int)sd_vehicle_interface::FinalDBWSteerRequest_Pc << endl;
        //cout <<  sd_vehicle_interface::_sd_vehicle << " TwistLinear " <<  setw(8) <<sd_vehicle_interface::TargetTwistLinear_Mps << " Current_V "<<  setw(4)  << sd_vehicle_interface::CurrentTwistLinearCANSD_Mps << " Torque "<<  setw(2)  << (int)sd_vehicle_interface::FinalDBWTorqueRequest_Pc << " P " <<  setw(2) << sd_vehicle_interface::P_Contribution_Pc << " I " <<  setw(2) << sd_vehicle_interface::I_Contribution_Pc << " D " <<  setw(2) << sd_vehicle_interface::D_Contribution_Pc << " FF " <<  setw(2) << sd_vehicle_interface::FF_Contribution_Pc << endl;
        SD_Current_Control->steer = sd_vehicle_interface::FinalDBWSteerRequest_Pc;
        SD_Current_Control->torque = sd_vehicle_interface::FinalDBWTorqueRequest_Pc;
        sd_vehicle_interface::sd_control_pub.publish(*SD_Current_Control);
        return "SPEED_DONE";
    }
    return "FREQUENCY_ERROR";
}

std::string sd_vehicle_interface::ChooseVehicle()
{
    if( sd_vehicle_interface::twizy_string== sd_vehicle_interface::_sd_vehicle){
        sd_vehicle_interface::FinalDBWTorqueRequest_Pc = speedcontroller::CalculateTorqueRequestTwizy(sd_vehicle_interface::TargetTwistLinear_Mps, sd_vehicle_interface::CurrentTwistLinearSD_Mps_Final, sd_vehicle_interface::P_Contribution_Pc, sd_vehicle_interface::I_Contribution_Pc, sd_vehicle_interface::D_Contribution_Pc, sd_vehicle_interface::FF_Contribution_Pc);
        return "Twizy";
    }else{
        sd_vehicle_interface::FinalDBWTorqueRequest_Pc = speedcontroller::CalculateTorqueRequestEnv200(sd_vehicle_interface::TargetTwistLinear_Mps, sd_vehicle_interface::CurrentTwistLinearSD_Mps_Final, sd_vehicle_interface::P_Contribution_Pc, sd_vehicle_interface::I_Contribution_Pc, sd_vehicle_interface::D_Contribution_Pc, sd_vehicle_interface::FF_Contribution_Pc);
        return "Env200";
    }
}

void sd_vehicle_interface::publish_states(geometry_msgs::TwistStamped current_Twist, sensor_msgs::NavSatFix current_GPS, sensor_msgs::Imu current_IMU)
{
    if(! sd_vehicle_interface::_sd_simulation_mode){ //If we are not in simulation mode, output on the CANbus the Control and Feedback Messages
        //Publish prepared messages
        sd_vehicle_interface::sent_msgs_pub.publish(sd_vehicle_interface::CustomerControlCANTx); //Publish the output CAN data
        sd_vehicle_interface::sent_msgs_pub.publish(sd_vehicle_interface::ControllerFeedbackCANTx);
    }
    sd_vehicle_interface::current_twist_pub.publish(current_Twist);
    sd_vehicle_interface::current_GPS_pub.publish(current_GPS);

    if( sd_vehicle_interface::no_imu_string != sd_vehicle_interface::_sd_gps_imu){ //If we have specified an IMU is present, publish an IMU message
        sd_vehicle_interface::current_IMU_pub.publish(current_IMU);
    }
}

int sd_vehicle_interface::RUN(int argc, char **argv)
{

    ros::init(argc ,argv, "sd_twizy_interface_node") ;

    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    private_nh.param<string>("sd_vehicle",  sd_vehicle_interface::_sd_vehicle, "env200");
    private_nh.param<string>("sd_gps_imu",  sd_vehicle_interface::_sd_speed_source, "oxts");
    private_nh.param<string>("sd_speed_source",  sd_vehicle_interface::_sd_speed_source, "vehicle_can");
    private_nh.param<bool>("sd_simulation_mode",  sd_vehicle_interface::_sd_simulation_mode, false);

    //initialise the StreetDrone Output Can variables
    sd::InitSDInterfaceControl(sd_vehicle_interface::CustomerControlCANTx);
    sd::InitSDInterfaceFeedback(sd_vehicle_interface::ControllerFeedbackCANTx);

    geometry_msgs::TwistStamped current_Twist;
    sensor_msgs::NavSatFix current_GPS;
    sensor_msgs::Imu current_IMU;
    aslan_msgs::SDControl SD_Current_Control;

    //Subscriber
    ros::Subscriber ReceivedFrameCANRx_sub = nh.subscribe("received_messages", 100, &sd_vehicle_interface::ReceivedFrameCANRx_callback, this);
    ros::Subscriber current_velocity_sub = nh.subscribe<geometry_msgs::TwistStamped>("current_velocity", 1, &sd_vehicle_interface::CurrentVelocity_callback, this);
    ros::Subscriber twist_cmd_sub = nh.subscribe<geometry_msgs::TwistStamped>("twist_cmd", 100, &sd_vehicle_interface::TwistCommand_callback, this);

    //publisher
    sd_vehicle_interface::sent_msgs_pub = nh.advertise<can_msgs::Frame>("sent_messages", 100);
    sd_vehicle_interface::current_twist_pub = nh.advertise<geometry_msgs::TwistStamped>("sd_current_twist", 100);
    sd_vehicle_interface::current_GPS_pub = nh.advertise<sensor_msgs::NavSatFix>("sd_current_GPS", 100);
    sd_vehicle_interface::current_IMU_pub = nh.advertise<sensor_msgs::Imu>("sd_imu_raw",100);
    sd_vehicle_interface::sd_control_pub = nh.advertise<aslan_msgs::SDControl>("sd_control", 1, true);


    ros::Rate loop_rate(ROS_LOOP);
    while(ros::ok())
    {
        //Choose the vehicle speed source as specified at launch
        sd_vehicle_interface::CurrentTwistLinearSD_Mps_Final = speedSource( sd_vehicle_interface::_sd_speed_source);

        Twist_setup(&current_Twist);
        //Prepare the GPS message with latest data
        current_GPS = GPS_setup(current_GPS);

        //Prepare the IMU message with latest data
        sd::PackImuMessage(sd_vehicle_interface::IMUVarianceKnown_B, current_IMU, sd_vehicle_interface::IMU_Angle_X, sd_vehicle_interface::IMU_Angle_Y, sd_vehicle_interface::IMU_Angle_Z, sd_vehicle_interface::IMU_Rate_X, sd_vehicle_interface::IMU_Rate_Y, sd_vehicle_interface::IMU_Rate_Z, sd_vehicle_interface::IMU_Accel_X, sd_vehicle_interface::IMU_Accel_Y, sd_vehicle_interface::IMU_Accel_Z);

        //Prepare the sd TX CAN messages with latest data
        sd_vehicle_interface::AliveCounter_Z++; //Increment the alive counter
        //Check Errors
        AutonomousControl_setup();

        AutonomousControl_action(&SD_Current_Control);

        publish_states(current_Twist, current_GPS, current_IMU);

        loop_rate.sleep(); //And loop
        ros::spinOnce();
    }
    return 0;
}