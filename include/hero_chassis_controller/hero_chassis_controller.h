//
// Created by Zhenyu Ye on 2021/2/21.
//

#ifndef HERO_CHASSIS_CONTROLLER_INCLUDE_HERO_CHASSIS_CONTROLLER_H_
#define HERO_CHASSIS_CONTROLLER_INCLUDE_HERO_CHASSIS_CONTROLLER_H_

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController()override;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &n) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  void starting(const ros::Time &time) override;

  ros::Subscriber sub_command;

  ros::Publisher odom_pub;

  control_toolbox::Pid pid1_controller_, pid2_controller_, pid3_controller_, pid4_controller_;
  //internal PID controllers for four wheels.
  hardware_interface::JointHandle
      front_left_joint_, front_right_joint_, back_left_joint_,
      back_right_joint_;
 private:
  double com1, com2, com3, com4;
  //command of four wheels.
  double Vx, Vy, yaw;
  //speed of the chassis.
  double Vxt, Vyt, YAW;

  double velocity1, velocity2, velocity3, velocity4;

  double x, y, th;

  ros::Time  lasttime;

  void getchassisstate(const geometry_msgs::TwistConstPtr &msg);

  void compute_mecvel(double x, double y, double YAW);

  void chassis_velocity();



};

} //namespace
#endif //HERO_CHASSIS_CONTROLLER_INCLUDE_HERO_CHASSIS_CONTROLLER_H_
