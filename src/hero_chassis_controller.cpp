//
// Created by yezi on 2021/2/25.
//

#include "hero_chassis_controller/hero_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

HeroChassisController::~HeroChassisController(){
 sub_command.shutdown();
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
  Vx = 0.0;
  Vy = 0.0;
  yaw = 0.0;

  //initialize command
  com1 = 0.0;
  com2 = 0.0;
  com3 = 0.0;
  com4 = 0.0;

  //Start realtime state publisher
  controller_state_publisher_.reset(
      new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>
          (n, "state", 1));

  //start command subscriber
  sub_command = n.subscribe<geometry_msgs::Twist>("cmd_vel", 1, &HeroChassisController::get_chassis_state, this);

  return true;
}

void HeroChassisController::update(const ros::Time &time, const ros::Duration &period) {
  //calculate speed of wheels
  compute_mecvel();
  //the error of wheels
  double error1 = com1 - front_right_joint_.getVelocity();
  double error2 = com2 - front_left_joint_.getVelocity();
  double error3 = com3 - back_left_joint_.getVelocity();
  double error4 = com4 - back_right_joint_.getVelocity();
  //set command for wheels
  front_right_joint_.setCommand(pid1_controller_.computeCommand(error1, period));
  front_left_joint_.setCommand(pid2_controller_.computeCommand(error2, period));
  back_left_joint_.setCommand(pid3_controller_.computeCommand(error3, period));
  back_right_joint_.setCommand(pid4_controller_.computeCommand(error4, period));

  if (loop_count_ % 10 == 0) {
    if (controller_state_publisher_ && controller_state_publisher_->trylock()) {
      controller_state_publisher_->msg_.header.stamp = time;
      controller_state_publisher_->msg_.set_point = com1;
      controller_state_publisher_->msg_.process_value = front_right_joint_.getVelocity();
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
}

void HeroChassisController::get_chassis_state(const geometry_msgs::TwistConstPtr &msg) {
  Vx = msg->linear.x;
  Vy = msg->linear.y;
  yaw = msg->angular.z;
}

void HeroChassisController::compute_mecvel() {
  com1 = (Vx + Vy + yaw * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
  com2 = (Vx - Vy - yaw * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
  com3 = (Vx + Vy - yaw * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
  com4 = (Vx - Vy + yaw * (Wheel_Track + Wheel_Base) / 2) / RADIUS;
}

}// namespace

PLUGINLIB_EXPORT_CLASS( hero_chassis_controller::HeroChassisController,controller_interface::ControllerBase)
