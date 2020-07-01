#ifndef SD_CONTROL__SD_CONTROL_PLUGIN_HPP
#define SD_CONTROL__SD_CONTROL_PLUGIN_HPP

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>

#include <ros/ros.h>
#include <aslan_msgs/SDControl.h>

namespace sd_control
{

  class SdControlPlugin : public gazebo::ModelPlugin
  {
  public:
    SdControlPlugin();

    void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf);

  private:
    std::string robot_namespace_;

    gazebo::physics::ModelPtr model_;
    gazebo::physics::WorldPtr world_;
    gazebo::transport::NodePtr gznode_;

    gazebo::physics::LinkPtr chassis_link_;

    gazebo::physics::JointPtr fl_wheel_joint_;
    gazebo::physics::JointPtr fr_wheel_joint_;
    gazebo::physics::JointPtr bl_wheel_joint_;
    gazebo::physics::JointPtr br_wheel_joint_;
    gazebo::physics::JointPtr fl_wheel_steering_joint_;
    gazebo::physics::JointPtr fr_wheel_steering_joint_;

    gazebo::common::PID fl_wheel_steering_pid_;
    gazebo::common::PID fr_wheel_steering_pid_;

    double fl_wheel_radius_;
    double fr_wheel_radius_;
    double bl_wheel_radius_;
    double br_wheel_radius_;

    double front_track_width_;
    double back_track_width_;
    double wheel_base_length_;

    double max_steer_;
    double max_speed_;
    double max_torque_;
    double front_brake_torque_;
    double back_brake_torque_;
    double chassis_aero_force_gain_;

    ros::Subscriber control_sub_;
    ros::Publisher odometry_pub_;

    gazebo::common::Time last_sim_time_;

    aslan_msgs::SDControl control_cmd_;

    std::mutex mutex_;

    gazebo::event::ConnectionPtr update_connection_;
    void Update();

    void publishOdometry();

    void controlCallback(const aslan_msgs::SDControl & msg);

    double collisionRadius(gazebo::physics::CollisionPtr coll) const;
  };

}

#endif
