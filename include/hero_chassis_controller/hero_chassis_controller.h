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
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_publisher.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

#define RADIUS 0.07625

namespace hero_chassis_controller {

class HeroChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  ros::Subscriber sub_command;

  ros::Publisher odom_pub;

  tf::TransformBroadcaster odom_broadcaster;
  tf::TransformListener listener;

  tf::StampedTransform transform;
  geometry_msgs::TransformStamped odom_trans;
  geometry_msgs::Vector3Stamped stamped_in, stamped_out;
  geometry_msgs::Quaternion odom_quat;

  nav_msgs::Odometry odom;

  control_toolbox::Pid pid1_controller_, pid2_controller_, pid3_controller_, pid4_controller_;
  //internal PID controllers for four wheels
  hardware_interface::JointHandle
      front_left_joint_, front_right_joint_, back_left_joint_,
      back_right_joint_;
 private:
  int loop_count_{};
  //command of four wheels
  double vel_cmd[5]{};
  //actual velocity of wheels
  double vel_act[5]{};
  //middle velocity of wheels
  double vel_mid[5]{};
  double Vxe{}, Vye{}, yawe{};
  //expected speed of the chassis
  double Vxa{}, Vya{}, yawa{};
  //actual speed of the chassis
  double Angle_Acceleration{};
  double dt{};
  double x{}, y{}, th{};
  double Wheel_Track{};
  double Wheel_Base{};
  bool Odom_Framecoordinate_Mode{};

  ros::Time last_time;
  ros::Time now;

  std::unique_ptr<
      realtime_tools::RealtimePublisher<
          control_msgs::JointControllerState> > controller_state_publisher_;
  //call back function of subscriber
  void get_chassis_state(const geometry_msgs::TwistConstPtr &msg);
  //calculate the speed  of four wheels
  void compute_mecvel();
  //calculate the speed of chassis
  void compute_chassis_velocity();
  void compute_vel_mid();
  void Transform_broadcast();
  void Odometry_publish();
};

} //namespace
#endif //HERO_CHASSIS_CONTROLLER_INCLUDE_HERO_CHASSIS_CONTROLLER_H_
