#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "nuri_joy/teleop_joy.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_unique<TeleopJoy>(rclcpp::NodeOptions()));

  rclcpp::shutdown();

  return 0;
}