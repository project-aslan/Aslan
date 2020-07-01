#include "sd_control/sd_teleop_joy.hpp"

#include <aslan_msgs/SDControl.h>

namespace sd_control
{

  SdTeleopJoy::SdTeleopJoy(ros::NodeHandle * nh, ros::NodeHandle * nh_param)
  {
    joy_sub_ = nh->subscribe("joy", 1, &SdTeleopJoy::joyCallback, this);
    control_pub_ = nh->advertise<aslan_msgs::SDControl>("/sd_control", 1, true);

    nh_param->param<int>("enable_button", enable_button_, 0);
    nh_param->param<int>("throttle_axis", throttle_axis_, 1);
    nh_param->param<int>("steer_axis", steer_axis_, 0);
  }

  void SdTeleopJoy::joyCallback(sensor_msgs::Joy::ConstPtr const & joy)
  {
    if (joy->buttons[enable_button_] == 1) {
      auto control_msg = aslan_msgs::SDControl{};
      control_msg.torque = joy->axes[throttle_axis_] * 100;
      control_msg.steer = joy->axes[steer_axis_] * 100;
      control_pub_.publish(control_msg);
    }
  }
}
