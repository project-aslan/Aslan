#include "sd_control/sd_teleop_joy.hpp"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sd_teleop_joy_node");

  auto nh = ros::NodeHandle{""};
  auto nh_param = ros::NodeHandle{"~"};

  auto joy_teleop = sd_control::SdTeleopJoy{&nh, &nh_param};

  ros::spin();
}
