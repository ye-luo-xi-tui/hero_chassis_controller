//
// Created by yezi on 2021/2/25.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

HeroChassisController::~HeroChassisController(){
 sub_command.shutdown();
  odom_pub.shutdown();
}

bool HeroChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                 ros::NodeHandle &n) {
  //get joint handle from hardware interface
  front_left_joint_ = effort_joint_interface->getHandle("front_left_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("front_right_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("back_left_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("back_right_wheel_joint");

  //load PID Controller using gains set on parameter server
  pid1_controller_.init(ros::NodeHandle(n,"pid1"));
  pid2_controller_.init(ros::NodeHandle(n,"pid2"));
  pid3_controller_.init(ros::NodeHandle(n,"pid3"));
  pid4_controller_.init(ros::NodeHandle(n,"pid4"));

  //initialiaze chassis speed
  Vx = 0.0;
  Vy = 0.0;
  yaw = 0.0;

  //initialiaze chassis position
  x= 0.0;
  y=0.0;
  th=0.0;


  //start command subscriber
  sub_command = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &HeroChassisController::getchassisstate, this);

  odom_pub= n.advertise<nav_msgs::Odometry>( "odom", 50);


  return true;
}

void HeroChassisController::starting(const ros::Time &time) {
  com1 = 0.0;
  com2 = 0.0;
  com3 = 0.0;
  com4 = 0.0;
  pid1_controller_.reset();
  pid2_controller_.reset();
  pid3_controller_.reset();
  pid4_controller_.reset();
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
  compute_mecvel(Vx, Vy, yaw);
  double error1 = com1 - front_right_joint_.getVelocity();
  double error2 = com2 - front_left_joint_.getVelocity();
  double error3 = com3 - back_left_joint_.getVelocity();
  double error4 = com4 - back_right_joint_.getVelocity();
  front_right_joint_.setCommand(pid1_controller_.computeCommand(error1, period));
  front_left_joint_.setCommand(pid2_controller_.computeCommand(error2, period));
  back_left_joint_.setCommand(pid3_controller_.computeCommand(error3, period));
  back_right_joint_.setCommand(pid4_controller_.computeCommand(error4, period));
  tf::TransformBroadcaster odom_broadcaster;




  chassis_velocity();
  double dt = (time - lasttime).toSec();
  double delta_x = (Vx * cos(th) - Vy * sin(th)) * dt;
  double delta_y = (Vx * sin(th) + Vy * cos(th)) * dt;
  double delta_th = yaw * dt;
  x += delta_x;
  y += delta_y;
  th += delta_th;
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = time;

  odom_trans.header.frame_id = "odom";

  odom_trans.child_frame_id = "base_link";
  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;
  //send the transform
  odom_broadcaster.sendTransform(odom_trans);

  //publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = Vx;
  odom.twist.twist.linear.y = Vy;
  odom.twist.twist.angular.z = yaw;

  //publish the message
  odom_pub.publish(odom);



}

void HeroChassisController::getchassisstate(const geometry_msgs::TwistConstPtr &msg) {
  Vx = msg->linear.x;
  Vy = msg->linear.y;
  yaw = msg->angular.z;
}

void HeroChassisController::compute_mecvel() {
  com1 = x - y - YAW * 0.4875;
  com2 = x + y + YAW * 0.4875;
  com3 = x + y - YAW * 0.4875;
  com4 = x - y + YAW * 0.4875;
}

void HeroChassisController::chassis_velocity(){
  Vx = (velocity1 + velocity2 + velocity3 + velocity4)/4;
  Vy = (velocity1 - velocity2 + velocity3 - velocity4)/4;
  yaw = (velocity1 - velocity2 - velocity3 + velocity4)/4/0.4875;
}

}// namespace

PLUGINLIB_EXPORT_CLASS( hero_chassis_controller::HeroChassisController,controller_interface::ControllerBase)
