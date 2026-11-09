#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <hero_chassis_controller/PidConfig.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

namespace hero_chassis_controller
{

class HeroChassisController
    : public controller_interface::Controller<
          hardware_interface::EffortJointInterface>
{
public:
  HeroChassisController() = default;

  bool init(hardware_interface::EffortJointInterface* hw,
            ros::NodeHandle& nh) override;

  void starting(const ros::Time& time) override;

  void update(const ros::Time& time,
              const ros::Duration& period) override;

  void stopping(const ros::Time& time) override;

private:
  void cmdVelCallback(const geometry_msgs::TwistConstPtr& msg);

  void reconfigureCallback(PidConfig& config, uint32_t level);

  geometry_msgs::Twist getBaseCommand();

  void limitAcceleration(const geometry_msgs::Twist& target, double dt);

  void updateOdometry(const std::array<double, 4>& wheel_velocities,
                      const ros::Time& time, double dt);

  std::array<hardware_interface::JointHandle, 4> joints_;
  std::array<control_toolbox::Pid, 4> pids_;
  std::array<std::string, 4> wheel_names_;

  ros::Subscriber cmd_vel_sub_;
  ros::Publisher odom_pub_;
  ros::Publisher wheel_debug_pub_;

  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  std::shared_ptr<dynamic_reconfigure::Server<PidConfig>>
      reconfigure_server_;

  geometry_msgs::Twist cmd_vel_;
  geometry_msgs::Twist limited_cmd_vel_;
  ros::Time last_cmd_time_;

  std::string odom_frame_ = "odom";
  std::string base_frame_ = "base_link";
  std::string global_frame_ = "odom";

  bool global_mode_ = false;

  double wheel_radius_ = 0.07625;
  double wheel_base_ = 0.4;
  double track_width_ = 0.4;
  double mecanum_k_ = 0.4;

  double cmd_timeout_ = 0.5;
  double max_linear_accel_ = 1.0;
  double max_angular_accel_ = 2.0;

  double x_ = 0.0;
  double y_ = 0.0;
  double yaw_ = 0.0;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
