#include <cinttypes>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <chrono>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging_macros.h>
#include <sensor_msgs/msg/joy.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "nuri_joy/teleop_joy.hpp"

#define ROS_INFO_NAMED RCUTILS_LOG_INFO_NAMED
#define ROS_INFO_COND_NAMED RCUTILS_LOG_INFO_EXPRESSION_NAMED

struct TeleopJoy::Impl
{
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
  void sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr, const std::string& which_map);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  TeleopJoy* node_; 

  void cb_timer();
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist twist_;

  double target_linear_ = 0.0, target_angular_ = 0.0;
  double current_linear_ = 0.0, current_angular_ = 0.0;
  double easeInSine(double value);
  std::chrono::steady_clock::time_point last_time_;
  bool haveSameSign(double a, double b);
};

TeleopJoy::TeleopJoy(const rclcpp::NodeOptions &options): Node("nuri_joy_node", options)
{
  pimpl_ = new Impl;
  pimpl_->node_ = this;

  pimpl_->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  pimpl_->joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("hc/joy", rclcpp::QoS(10),
    std::bind(&TeleopJoy::Impl::joyCallback, this->pimpl_, std::placeholders::_1));

  pimpl_->twist_ = geometry_msgs::msg::Twist();
  pimpl_->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TeleopJoy::Impl::cb_timer, this->pimpl_));  
   
}

TeleopJoy::~TeleopJoy()
{
  delete pimpl_;
}

void TeleopJoy::Impl::sendCmdVelMsg(const sensor_msgs::msg::Joy::SharedPtr joy_msg,
                                         const std::string& which_map)
{
  if (joy_msg->axes[1] > 0.0) {
    target_linear_ = joy_msg->axes[1] * 0.6;
  }
  else if (joy_msg->axes[1] < 0.0) {
    target_linear_ = joy_msg->axes[1] * 0.4;
  }
  else {
    target_linear_ = 0;
  }

  target_angular_ = joy_msg->axes[0] * 1.0;
}

void TeleopJoy::Impl::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  sendCmdVelMsg(joy_msg, "normal");
}

void TeleopJoy::Impl::cb_timer()
{
  auto now = std::chrono::steady_clock::now();
  double elapsed_time = std::chrono::duration<double>(now - last_time_).count();
  last_time_ = now;

  // RCLCPP_INFO(
  //     this->node_->get_logger(),
  //     "[joy_msg] x : %f, z: %f",
  //     target_linear_,
  //     target_angular_);  

  if (target_linear_ > 0.0 ) {
    // 전진
    if (current_linear_ < 0.0){
      // 후진일 때
      current_linear_ = 0;
    }
    else {
      // 전진 중일 때, 정지일 때
      current_linear_ += (target_linear_ - current_linear_) * easeInSine(elapsed_time);
    }
  }
  else if (target_linear_ < 0.0 ) {
    // 후진
    if (current_linear_ > 0.0) {
      // 전진 중일 때
      current_linear_ = 0;
    }
    else {
      // 후진일 때, 정지일 때
      current_linear_ += (target_linear_ - current_linear_) * easeInSine(elapsed_time);
    }
  }
  else {
    current_linear_ = 0;
  }

  current_linear_ += (target_linear_ - current_linear_) * easeInSine(elapsed_time);
  if (target_linear_ > 0.0 ) {
    // 전진
    if (current_linear_ < 0.0){
      // 후진일 때
      current_linear_ = 0;
    } 
    else {
      if (target_linear_ < current_linear_) {
        current_linear_ = target_linear_;
      }
    }
  }
  else if (target_linear_ < 0.0 ) {
    // 후진
    if (current_linear_ > 0.0) {
      // 전진 중일 때
      current_linear_ = 0;
    }
    else {
      if (target_linear_ > current_linear_) {
        current_linear_ = target_linear_;
      }      
    }
  }
  else {
    current_linear_ = 0;
  }

  // current_angular_ += (target_angular_ - current_angular_) * easeInSine(elapsed_time);
  // if (target_angular_ > 0.0) {
  //   if (current_angular_ < 0.0) {
  //     current_angular_ = 0;
  //   }
  //   else {
  //     if (target_angular_ < current_angular_) {
  //       current_angular_ = target_angular_;
  //     }
  //   }
  // }
  // else if (target_angular_ < 0.0) {
  //   if (current_angular_ > 0.0) {
  //     current_angular_ = 0;
  //   }
  //   else {
  //     if (target_angular_ > current_angular_) {
  //       current_angular_ = target_angular_;
  //     }
  //   }
  // }
  // else {
  //   current_angular_ = 0;
  // }
  current_angular_ = target_angular_;

  // current_linear_ += (target_linear_ - current_linear_) * easeInSine(elapsed_time);
  // current_angular_ += (target_angular_ - current_angular_) * easeInSine(elapsed_time);

  if (target_linear_ == 0) {
    current_linear_ = 0;
  }
  if (target_angular_ == 0) {
    current_angular_ = 0;
  }

  twist_.linear.x = current_linear_;
  twist_.angular.z = current_angular_;  

  cmd_vel_pub->publish(twist_);
}

double TeleopJoy::Impl::easeInSine(double value)
{
  return (value == 0) ? 0 : 1 - std::cos((value * M_PI) / 2);
}

bool TeleopJoy::Impl::haveSameSign(double a, double b)
{
  return (a * b) >= 0.0;
}

RCLCPP_COMPONENTS_REGISTER_NODE(TeleopJoy)