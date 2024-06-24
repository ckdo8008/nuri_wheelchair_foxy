// #include

#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>
#include "nuri_odom/diff_drive_controller.hpp"

int main(int argc, char *argv[])
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    auto diff_drive_controller =
        std::make_shared<DiffDriveController>(
            0.45,  // wheel_separation
            0.01088); // wheel_radius

    executor.add_node(diff_drive_controller);
    executor.spin();

    rclcpp::shutdown();
}