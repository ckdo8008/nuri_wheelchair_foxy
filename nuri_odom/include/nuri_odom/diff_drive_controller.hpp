#ifndef NURI_NODE__DIFF_DIRVER_CONTROLLER
#define NURI_NODE__DIFF_DIRVER_CONTROLLER

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include "nuri_odom/odometry.hpp"

class DiffDriveController : public rclcpp::Node
{
public:
  explicit DiffDriveController(const float wheel_seperation, const float wheel_radius);
  virtual ~DiffDriveController() {}

private:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> odometry_;
};

#endif