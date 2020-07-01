#ifndef SD_CONTROL__SD_TELEOP_JOY_HPP_
#define SD_CONTROL__SD_TELEOP_JOY_HPP_

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>

namespace sd_control
{

  class SdTeleopJoy
  {
  public:
    SdTeleopJoy(ros::NodeHandle * nh, ros::NodeHandle * nh_param);

  private:
    int enable_button_;
    int throttle_axis_;
    int steer_axis_;

    std::map<std::string, double> scale_throttle_map_;

    ros::Subscriber joy_sub_;
    ros::Publisher control_pub_;

    void joyCallback(sensor_msgs::Joy::ConstPtr const & joy);
  };
}

#endif  // SD_CONTROL__SD_TELEOP_JOY_HPP_
