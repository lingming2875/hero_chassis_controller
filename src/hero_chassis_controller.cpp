#include "hero_chassis_controller/hero_chassis_controller.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <boost/bind.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf/transform_datatypes.h>

namespace hero_chassis_controller
{

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface* hw,
    ros::NodeHandle& nh)
{
  wheel_names_ = {
      "front_left",
      "front_right",
      "back_left",
      "back_right",
  };

  std::vector<std::string> joint_names;

  if (!nh.getParam("joints", joint_names) ||
      joint_names.size() != 4)
  {
    ROS_ERROR(
        "The 'joints' parameter must contain "
        "exactly four joint names.");
    return false;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    try
    {
      joints_[i] = hw->getHandle(joint_names[i]);
    }
    catch (
        const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM(
          "Failed to get joint handle '"
          << joint_names[i] << "': " << e.what());
      return false;
    }
  }

  nh.param<double>(
      "wheel_radius", wheel_radius_, wheel_radius_);
  nh.param<double>(
      "wheel_base", wheel_base_, wheel_base_);
  nh.param<double>(
      "track_width", track_width_, track_width_);
  nh.param<double>(
      "cmd_timeout", cmd_timeout_, cmd_timeout_);
  nh.param<double>(
      "max_linear_accel",
      max_linear_accel_,
      max_linear_accel_);
  nh.param<double>(
      "max_angular_accel",
      max_angular_accel_,
      max_angular_accel_);

  nh.param<std::string>(
      "odom_frame", odom_frame_, odom_frame_);
  nh.param<std::string>(
      "base_frame", base_frame_, base_frame_);
  nh.param<std::string>(
      "global_frame", global_frame_, global_frame_);

  nh.param<bool>(
      "global_mode", global_mode_, global_mode_);

  mecanum_k_ = (wheel_base_ + track_width_) / 2.0;

  if (wheel_radius_ <= 0.0 || mecanum_k_ <= 0.0)
  {
    ROS_ERROR(
        "Wheel radius and chassis dimensions "
        "must be positive.");
    return false;
  }

  cmd_vel_sub_ = nh.subscribe(
      "/cmd_vel",
      1,
      &HeroChassisController::cmdVelCallback,
      this);

  odom_pub_ =
      nh.advertise<nav_msgs::Odometry>("/odom", 1);

  wheel_debug_pub_ =
      nh.advertise<std_msgs::Float64MultiArray>(
          "wheel_debug", 1);

  reconfigure_server_ =
      std::make_shared<
          dynamic_reconfigure::Server<PidConfig>>(nh);

  reconfigure_server_->setCallback(
      boost::bind(
          &HeroChassisController::reconfigureCallback,
          this,
          _1,
          _2));

  ROS_INFO("Hero chassis controller initialized.");
  return true;
}

void HeroChassisController::starting(const ros::Time&)
{
  for (auto& pid : pids_)
  {
    pid.reset();
  }

  cmd_vel_ = geometry_msgs::Twist();
  limited_cmd_vel_ = geometry_msgs::Twist();
  last_cmd_time_ = ros::Time(0);

  x_ = 0.0;
  y_ = 0.0;
  yaw_ = 0.0;
}

void HeroChassisController::update(
    const ros::Time& time,
    const ros::Duration& period)
{
  const double dt = period.toSec();

  if (dt <= 0.0)
  {
    return;
  }

  // Stop if no new command has arrived recently.
  if (last_cmd_time_.isZero() ||
      (time - last_cmd_time_).toSec() > cmd_timeout_)
  {
    cmd_vel_ = geometry_msgs::Twist();
  }

  // Convert global velocity to chassis velocity if needed.
  const geometry_msgs::Twist base_cmd = getBaseCommand();

  // Limit linear and angular acceleration.
  limitAcceleration(base_cmd, dt);

  const double vx = limited_cmd_vel_.linear.x;
  const double vy = limited_cmd_vel_.linear.y;
  const double wz = limited_cmd_vel_.angular.z;

  // Mecanum inverse kinematics.
  // Wheel order: LF, RF, LB, RB.
  std::array<double, 4> desired_velocities = {
      (vx - vy - mecanum_k_ * wz) / wheel_radius_,
      (vx + vy + mecanum_k_ * wz) / wheel_radius_,
      (vx + vy - mecanum_k_ * wz) / wheel_radius_,
      (vx - vy + mecanum_k_ * wz) / wheel_radius_,
  };

  std::array<double, 4> actual_velocities;
  std::array<double, 4> efforts;

  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    actual_velocities[i] = joints_[i].getVelocity();

    const double error =
        desired_velocities[i] - actual_velocities[i];

    efforts[i] =
        pids_[i].computeCommand(error, period);

    joints_[i].setCommand(efforts[i]);
  }

  // Debug data for rqt_plot:
  // data[0..3]: desired wheel speeds
  // data[4..7]: actual wheel speeds
  // data[8..11]: PID output efforts
  std_msgs::Float64MultiArray debug_msg;
  debug_msg.data.resize(12);

  for (std::size_t i = 0; i < joints_.size(); ++i)
  {
    debug_msg.data[i] = desired_velocities[i];
    debug_msg.data[i + 4] = actual_velocities[i];
    debug_msg.data[i + 8] = efforts[i];
  }

  wheel_debug_pub_.publish(debug_msg);

  updateOdometry(actual_velocities, time, dt);
}

void HeroChassisController::stopping(const ros::Time&)
{
  for (auto& joint : joints_)
  {
    joint.setCommand(0.0);
  }
}

void HeroChassisController::cmdVelCallback(
    const geometry_msgs::TwistConstPtr& msg)
{
  cmd_vel_ = *msg;
  last_cmd_time_ = ros::Time::now();
}






void HeroChassisController::reconfigureCallback(
    PidConfig& config,
    uint32_t)
{
  const std::array<double, 4> p = {
      config.front_left_p,
      config.front_right_p,
      config.back_left_p,
      config.back_right_p,
  };

  const std::array<double, 4> i = {
      config.front_left_i,
      config.front_right_i,
      config.back_left_i,
      config.back_right_i,
  };

  const std::array<double, 4> d = {
      config.front_left_d,
      config.front_right_d,
      config.back_left_d,
      config.back_right_d,
  };

  const std::array<double, 4> i_clamp = {
      config.front_left_i_clamp,
      config.front_right_i_clamp,
      config.back_left_i_clamp,
      config.back_right_i_clamp,
  };

  for (std::size_t index = 0;
       index < pids_.size();
       ++index)
  {
    pids_[index].setGains(
        p[index],
        i[index],
        d[index],
        i_clamp[index],
        -i_clamp[index]);
  }
}

geometry_msgs::Twist
HeroChassisController::getBaseCommand()
{
  geometry_msgs::Twist base_cmd = cmd_vel_;

  if (!global_mode_)
  {
    return base_cmd;
  }

  try
  {
    tf::Stamped<tf::Vector3> global_vector(
        tf::Vector3(
            cmd_vel_.linear.x,
            cmd_vel_.linear.y,
            0.0),
        ros::Time(0),
        global_frame_);

    tf::Stamped<tf::Vector3> base_vector;

    tf_listener_.transformVector(
        base_frame_,
        global_vector,
        base_vector);

    base_cmd.linear.x = base_vector.x();
    base_cmd.linear.y = base_vector.y();
    base_cmd.linear.z = 0.0;
  }
  catch (const tf::TransformException& e)
  {
    ROS_WARN_THROTTLE(
        1.0,
        "Failed to transform global velocity: %s",
        e.what());

    // Avoid using an untransformed global velocity.
    base_cmd.linear.x = 0.0;
    base_cmd.linear.y = 0.0;
    base_cmd.linear.z = 0.0;
  }

  return base_cmd;
}

void HeroChassisController::limitAcceleration(
    const geometry_msgs::Twist& target,
    double dt)
{
  const auto limit =
      [dt](double current,
           double target_value,
           double max_accel) {
        const double max_delta = max_accel * dt;
        const double delta = target_value - current;

        return current +
               std::max(
                   -max_delta,
                   std::min(max_delta, delta));
      };

  limited_cmd_vel_.linear.x =
      limit(
          limited_cmd_vel_.linear.x,
          target.linear.x,
          max_linear_accel_);

  limited_cmd_vel_.linear.y =
      limit(
          limited_cmd_vel_.linear.y,
          target.linear.y,
          max_linear_accel_);

  limited_cmd_vel_.angular.z =
      limit(
          limited_cmd_vel_.angular.z,
          target.angular.z,
          max_angular_accel_);
}

void HeroChassisController::updateOdometry(
    const std::array<double, 4>& wheel_velocities,
    const ros::Time& time,
    double dt)
{
  const double w0 = wheel_velocities[0];
  const double w1 = wheel_velocities[1];
  const double w2 = wheel_velocities[2];
  const double w3 = wheel_velocities[3];

  // Mecanum forward kinematics.
  const double vx =
      wheel_radius_ / 4.0 *
      (w0 + w1 + w2 + w3);

  const double vy =
      wheel_radius_ / 4.0 *
      (-w0 + w1 + w2 - w3);

  const double wz =
      wheel_radius_ / (4.0 * mecanum_k_) *
      (-w0 + w1 - w2 + w3);

  // Integrate using midpoint yaw.
  const double midpoint_yaw =
      yaw_ + wz * dt / 2.0;

  x_ += (vx * std::cos(midpoint_yaw) -
         vy * std::sin(midpoint_yaw)) * dt;

  y_ += (vx * std::sin(midpoint_yaw) +
         vy * std::cos(midpoint_yaw)) * dt;

  yaw_ += wz * dt;

  // Keep yaw within approximately [-pi, pi].
  yaw_ = std::atan2(
      std::sin(yaw_),
      std::cos(yaw_));

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = time;
  odom_msg.header.frame_id = odom_frame_;
  odom_msg.child_frame_id = base_frame_;

  odom_msg.pose.pose.position.x = x_;
  odom_msg.pose.pose.position.y = y_;
  odom_msg.pose.pose.position.z = 0.0;

  odom_msg.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(yaw_);

  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = wz;

  odom_pub_.publish(odom_msg);

  geometry_msgs::TransformStamped transform;
  transform.header.stamp = time;
  transform.header.frame_id = odom_frame_;
  transform.child_frame_id = base_frame_;

  transform.transform.translation.x = x_;
  transform.transform.translation.y = y_;
  transform.transform.translation.z = 0.0;

  transform.transform.rotation =
      odom_msg.pose.pose.orientation;

  tf_broadcaster_.sendTransform(transform);
}

}  // namespace hero_chassis_controller

PLUGINLIB_EXPORT_CLASS(
    hero_chassis_controller::HeroChassisController,
    controller_interface::ControllerBase)























