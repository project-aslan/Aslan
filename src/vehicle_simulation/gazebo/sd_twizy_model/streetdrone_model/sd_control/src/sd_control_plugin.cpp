#include "sd_control/sd_control_plugin.hpp"

#include <gazebo/physics/physics.hh>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

namespace sd_control
{

  SdControlPlugin::SdControlPlugin()
    : fl_wheel_radius_{0},
      fr_wheel_radius_{0},
      bl_wheel_radius_{0},
      br_wheel_radius_{0},
      last_sim_time_{0},
      robot_namespace_{""}
  {
  }

  void SdControlPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
  {
    ROS_INFO("Loading plugin!");

    model_ = model;
    world_ = model_->GetWorld();
    auto physicsEngine = world_->Physics();
    physicsEngine->SetParam("friction_model", std::string{"cone_model"});

    gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    gznode_->Init();

    if (sdf->HasElement("robotNamespace"))
      robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

    ros::NodeHandle nh(this->robot_namespace_);
    control_sub_ = nh.subscribe(
      "/sd_control", 10, &SdControlPlugin::controlCallback, this
      );

    odometry_pub_ = nh.advertise<nav_msgs::Odometry>("/odom", 1);

    // Find joints and links
    auto findLink = [&](std::string const& link_name) {
      auto full_link_name = model_->GetName() + "::"
        + sdf->Get<std::string>(link_name);
      auto link = model_->GetLink(full_link_name);
      if (!link)
        std::cerr << "could not find link: " << full_link_name << "\n";
      return link;
    };

    auto findJoint = [&](std::string const& joint_name) {
      auto full_joint_name = model_->GetName() + "::"
        + sdf->Get<std::string>(joint_name);
      auto joint = model_->GetJoint(full_joint_name);
      if (!joint)
        std::cerr << "could not find joint: " << full_joint_name << "\n";
      return joint;
    };

    chassis_link_ = findLink("chassis");

    fl_wheel_joint_ = findJoint("front_left_wheel");
    fr_wheel_joint_ = findJoint("front_right_wheel");
    bl_wheel_joint_ = findJoint("back_left_wheel");
    br_wheel_joint_ = findJoint("back_right_wheel");
    fl_wheel_steering_joint_ = findJoint("front_left_wheel_steering");
    fr_wheel_steering_joint_ = findJoint("front_right_wheel_steering");

    // Read parameters
    auto findParameter = [&](std::string const& param_name, double default_value) {
      if (sdf->HasElement(param_name))
        return sdf->Get<double>(param_name);
      else return default_value;
    };

    max_steer_ = findParameter("max_steer", 0.785398);
    max_speed_ = findParameter("max_speed", 10.0);
    max_torque_ = findParameter("max_torque", 57.0);
    front_brake_torque_ = findParameter("front_brake_torque", 500.0);
    back_brake_torque_ = findParameter("back_brake_torque", 500.0);
    chassis_aero_force_gain_ = findParameter("chassis_aero_force_gain", 1.0);

    fl_wheel_steering_pid_.SetPGain(findParameter("fl_wheel_steering_p_gain", 0.0));
    fl_wheel_steering_pid_.SetIGain(findParameter("fl_wheel_steering_i_gain", 0.0));
    fl_wheel_steering_pid_.SetDGain(findParameter("fl_wheel_steering_d_gain", 0.0));

    fr_wheel_steering_pid_.SetPGain(findParameter("fr_wheel_steering_p_gain", 0.0));
    fr_wheel_steering_pid_.SetIGain(findParameter("fr_wheel_steering_i_gain", 0.0));
    fr_wheel_steering_pid_.SetDGain(findParameter("fr_wheel_steering_d_gain", 0.0));

    fl_wheel_steering_pid_.SetCmdMin(-5000);
    fl_wheel_steering_pid_.SetCmdMax(5000);
    fr_wheel_steering_pid_.SetCmdMin(-5000);
    fr_wheel_steering_pid_.SetCmdMax(5000);

    // Determine physical properties
    auto id = unsigned{0};
    fl_wheel_radius_ = collisionRadius(fl_wheel_joint_->GetChild()->GetCollision(id));
    fr_wheel_radius_ = collisionRadius(fr_wheel_joint_->GetChild()->GetCollision(id));
    bl_wheel_radius_ = collisionRadius(bl_wheel_joint_->GetChild()->GetCollision(id));
    br_wheel_radius_ = collisionRadius(br_wheel_joint_->GetChild()->GetCollision(id));

    ROS_INFO_STREAM("Radii found:" << fl_wheel_radius_ << " " << fr_wheel_radius_ << " " << bl_wheel_radius_ << " " << br_wheel_radius_);

    // Compute wheelbase, frontTrackWidth, and rearTrackWidth
    //  first compute the positions of the 4 wheel centers
    //  again assumes wheel link is child of joint and has only one collision
    auto fl_center_pos = fl_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();
    auto fr_center_pos = fr_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();
    auto bl_center_pos = bl_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();
    auto br_center_pos = br_wheel_joint_->GetChild()->GetCollision(id)->WorldPose().Pos();

    // track widths are computed first
    auto vec3 = fl_center_pos - fr_center_pos;
    front_track_width_ = vec3.Length();
    vec3 = fl_center_pos - fr_center_pos;
    back_track_width_ = vec3.Length();

    // to compute wheelbase, first position of axle centers are computed
    auto front_axle_pos = (fl_center_pos + fr_center_pos) / 2;
    auto back_axle_pos = (bl_center_pos + br_center_pos) / 2;
    // then the wheelbase is the distance between the axle centers
    vec3 = front_axle_pos - back_axle_pos;
    wheel_base_length_ = vec3.Length();

    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&SdControlPlugin::Update, this));
  }

  void SdControlPlugin::controlCallback(const aslan_msgs::SDControl & msg)
  {
    std::lock_guard<std::mutex> lock{mutex_};
    control_cmd_ = msg;
	 if (control_cmd_.torque > 0 && control_cmd_.torque <= 25){
	 control_cmd_.torque = 0; //This captures the dead pedal band on the real vehicle. 
	 }
  }

  void SdControlPlugin::Update()
  {
    std::lock_guard<std::mutex> lock{mutex_};

    auto cur_time = world_->SimTime();
    auto dt = (cur_time - last_sim_time_).Double();
    if (dt < 0) {
      // TODO: reset
      return;
    }
    else if (dt == 0.0)
    {
      // TODO: use ignition::math::equal?
      return;
    }

    auto fl_steering_angle = fl_wheel_steering_joint_->Position();
    auto fr_steering_angle = fr_wheel_steering_joint_->Position();

    auto fl_wheel_angular_velocity = fl_wheel_joint_->GetVelocity(0);
    auto fr_wheel_angular_velocity = fr_wheel_joint_->GetVelocity(0);
    auto bl_wheel_angular_velocity = bl_wheel_joint_->GetVelocity(0);
    auto br_wheel_angular_velocity = br_wheel_joint_->GetVelocity(0);

    auto chassis_linear_velocity = chassis_link_->WorldCoGLinearVel();

    auto drag_force = -chassis_aero_force_gain_
      * chassis_linear_velocity.SquaredLength()
      * chassis_linear_velocity.Normalized();
    chassis_link_->AddForce(drag_force);

    auto steer_ratio = std::max(-100.0, std::min(100.0, control_cmd_.steer)) / 100.0;
    auto steer_angle = steer_ratio * max_steer_;

    // Ackermann steering geometry
    auto tan_steer = std::tan(steer_angle);
    auto fl_wheel_steering_command =
      std::atan2(tan_steer, 1.0 + front_track_width_ / 2 / wheel_base_length_ * tan_steer);
    auto fr_wheel_steering_command =
      std::atan2(tan_steer, 1.0 - front_track_width_ / 2 / wheel_base_length_ * tan_steer);

    // Update steering PID controllers
    auto fl_steering_error = fl_steering_angle - fl_wheel_steering_command;
    auto fr_steering_error = fr_steering_angle - fr_wheel_steering_command;

    auto fl_wheel_steering_force = fl_wheel_steering_pid_.Update(fl_steering_error, dt);
    auto fr_wheel_steering_force = fr_wheel_steering_pid_.Update(fr_steering_error, dt);

    fl_wheel_steering_joint_->SetForce(0, fl_wheel_steering_force);
    fr_wheel_steering_joint_->SetForce(0, fr_wheel_steering_force);

    auto throttle_ratio = 0.0;
    auto brake_ratio = 0.0;
    if (control_cmd_.torque > 0)
      throttle_ratio = std::min(100.0, control_cmd_.torque) / 100.0;
    if (control_cmd_.torque < 0)
      brake_ratio = std::min(100.0, -control_cmd_.torque) / 100.0;

    auto regen_braking_ratio = 0.025;

    // use regen braking, unless it is overcome by throttle, or direct brake ratio is higher
    // examples:
    // * throttle = 0, brake = 0 ==> brake_ratio = regen_braking_ratio
    // * throttle = 0, brake > regen ==> brake_ratio = brake_ratio
    // * throttle > regen_braking_ratio, brake_ratio = 0 ==> brake_ratio = 0
    brake_ratio = std::max(regen_braking_ratio - throttle_ratio, brake_ratio);
    brake_ratio = std::max(0.0, std::min(1.0, brake_ratio));

    fl_wheel_joint_->SetParam("friction", 0, brake_ratio * front_brake_torque_);
    fr_wheel_joint_->SetParam("friction", 0, brake_ratio * front_brake_torque_);
    bl_wheel_joint_->SetParam("friction", 0, brake_ratio * back_brake_torque_);
    br_wheel_joint_->SetParam("friction", 0, brake_ratio * back_brake_torque_);

    auto throttle_torque = 0.0;
    if (std::abs(bl_wheel_angular_velocity * bl_wheel_radius_) < max_speed_ &&
        std::abs(br_wheel_angular_velocity * br_wheel_radius_) < max_speed_)
      throttle_torque = throttle_ratio * max_torque_;

    bl_wheel_joint_->SetForce(0, throttle_torque);
    br_wheel_joint_->SetForce(0, throttle_torque);

    publishOdometry();

    last_sim_time_ = cur_time;
  }

  void SdControlPlugin::publishOdometry()
  {
    auto current_time = ros::Time::now();
    auto pose = model_->WorldPose();

    // Global rotation and translation
    auto qt = tf::Quaternion ( pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W() );
    auto vt = tf::Vector3 ( pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z() );

    auto odom = nav_msgs::Odometry{};

    odom.pose.pose.position.x = vt.x();
    odom.pose.pose.position.y = vt.y();
    odom.pose.pose.position.z = vt.z();

    odom.pose.pose.orientation.x = qt.x();
    odom.pose.pose.orientation.y = qt.y();
    odom.pose.pose.orientation.z = qt.z();
    odom.pose.pose.orientation.w = qt.w();

    // get velocity in /odom frame
    ignition::math::Vector3d linear;
    linear = model_->WorldLinearVel();
    odom.twist.twist.angular.z = model_->WorldAngularVel().Z();

    // convert velocity to child_frame_id
    float yaw = pose.Rot().Yaw();
    odom.twist.twist.linear.x = cosf ( yaw ) * linear.X() + sinf ( yaw ) * linear.Y();
    odom.twist.twist.linear.y = cosf ( yaw ) * linear.Y() - sinf ( yaw ) * linear.X();

    // set covariance
    odom.pose.covariance[0] = 0.00001;
    odom.pose.covariance[7] = 0.00001;
    odom.pose.covariance[14] = 1000000000000.0;
    odom.pose.covariance[21] = 1000000000000.0;
    odom.pose.covariance[28] = 1000000000000.0;
    odom.pose.covariance[35] = 0.001;

    // set header
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odometry_pub_.publish(odom);
  }

  double SdControlPlugin::collisionRadius(gazebo::physics::CollisionPtr coll) const
  {
    if (coll == nullptr || coll->GetShape() == nullptr)
      return 0;

    if (coll->GetShape()->HasType(gazebo::physics::Base::CYLINDER_SHAPE))
    {
      gazebo::physics::CylinderShape *cyl =
        static_cast<gazebo::physics::CylinderShape*>(coll->GetShape().get());
      return cyl->GetRadius();
    }
    else if (coll->GetShape()->HasType(gazebo::physics::Base::SPHERE_SHAPE))
    {
      gazebo::physics::SphereShape *sph =
        static_cast<gazebo::physics::SphereShape*>(coll->GetShape().get());
      return sph->GetRadius();
    }
    return 0;
  }

GZ_REGISTER_MODEL_PLUGIN(SdControlPlugin)

}
