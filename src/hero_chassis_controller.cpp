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
  n.getParam("Wheel_Track", Wheel_Track);
  n.getParam("Wheel_Base", Wheel_Base);

  //get joint handle from hardware interface
  front_left_joint_ = effort_joint_interface->getHandle("front_left_wheel_joint");
  front_right_joint_ = effort_joint_interface->getHandle("front_right_wheel_joint");
  back_left_joint_ = effort_joint_interface->getHandle("back_left_wheel_joint");
  back_right_joint_ = effort_joint_interface->getHandle("back_right_wheel_joint");

  //load PID Controller using gains set on parameter server
  pid1_controller_.init(ros::NodeHandle(n, "pid1"));
  pid2_controller_.init(ros::NodeHandle(n, "pid2"));
  pid3_controller_.init(ros::NodeHandle(n, "pid3"));
  pid4_controller_.init(ros::NodeHandle(n, "pid4"));

  //initialiaze chassis speed
  Vxe = 0.0;
  Vye = 0.0;
  yawe = 0.0;

  //initialize command
  vel_cmd1 = 0.0;
  vel_cmd2 = 0.0;
  vel_cmd3 = 0.0;
  vel_cmd4 = 0.0;

  //initialiaze chassis position
  x = 0.0;
  y = 0.0;
  th = 0.0;

  //initializae last_time
  last_time = ros::Time::now();

  //Start realtime state publisher
  controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
          (n, "state", 1));

  //start command subscriber
  sub_command = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &HeroChassisController::get_chassis_state, this);

  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

  return true;
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
  vel_act1 = front_right_joint_.getVelocity();
  vel_act2 = front_left_joint_.getVelocity();
  vel_act3 = back_left_joint_.getVelocity();
  vel_act4 = back_right_joint_.getVelocity();
  //calculate speed of wheels
  compute_mecvel();
  //the error of wheels
  double error1 = vel_cmd1 - vel_act1;
  double error2 = vel_cmd2 - vel_act2;
  double error3 = vel_cmd3 - vel_act3;
  double error4 = vel_cmd4 - vel_act4;
  //set command for wheels
  front_right_joint_.setCommand(pid1_controller_.computeCommand(error1, period));
  front_left_joint_.setCommand(pid2_controller_.computeCommand(error2, period));
  back_left_joint_.setCommand(pid3_controller_.computeCommand(error3, period));
  back_right_joint_.setCommand(pid4_controller_.computeCommand(error4, period));

  if (loop_count_ % 10 == 0) {
    if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = vel_cmd1;
      controller_state_publisher_->msg_.process_value = vel_act1;
      controller_state_publisher_->msg_.error = error1;
      controller_state_publisher_->msg_.time_step = period.toSec();
      controller_state_publisher_->msg_.command = pid1_controller_.computeCommand(error1, period);

      double dummy;
      bool antiwindup;
      pid1_controller_.getGains(controller_state_publisher_->msg_.p,
                                controller_state_publisher_->msg_.i,
                                controller_state_publisher_->msg_.d,
                                controller_state_publisher_->msg_.i_clamp,
                                dummy,
                                antiwindup);
      controller_state_publisher_->msg_.antiwindup = static_cast<char>(antiwindup);
      controller_state_publisher_->unlockAndPublish();
    }
  }
  loop_count_++;



  //calculate the speed of chassis
  chassis_velocity();
  double dt = (time - last_time).toSec();
  double delta_x = (Vxa * cos(th) - Vya * sin(th)) * dt;
  double delta_y = (Vxa * sin(th) + Vya * cos(th)) * dt;
  double delta_th = yawa * dt;
  x += delta_x;
  y += delta_y;
  th += delta_th;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

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
  odom.header.stamp = time;
  odom.header.frame_id = "odom";
  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = Vxa;
  odom.twist.twist.linear.y = Vya;
  odom.twist.twist.angular.z = yawa;

  //publish the message
  odom_pub.publish(odom);

  last_time = time;

}

void HeroChassisController::get_chassis_state(const geometry_msgs::TwistConstPtr &msg) {
  Vxe = msg->linear.x;
  Vye = msg->linear.y;
  yawe = msg->angular.z;
}

void HeroChassisController::compute_mecvel() {
  vel_cmd1 = (Vxe + Vye + yawe * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
  vel_cmd2 = (Vxe - Vye - yawe * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
  vel_cmd3 = (Vxe + Vye - yawe * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
  vel_cmd4 = (Vxe - Vye + yawe * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
}

void HeroChassisController::chassis_velocity() {
  Vxa = (vel_act1 + vel_act2 + vel_act3 + vel_act4) * RADIUS / 4;
  Vya = (vel_act1 - vel_act2 + vel_act3 - vel_act4) * RADIUS / 4;
  yawa = (vel_act1 - vel_act2 - vel_act3 + vel_act4) * RADIUS / 4 / (Wheel_Track + Wheel_Base);
}
}// namespace

PLUGINLIB_EXPORT_CLASS( hero_chassis_controller::HeroChassisController,controller_interface::ControllerBase)
