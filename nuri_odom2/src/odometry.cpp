#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nurirobot_msgs/msg/nurirobot_pos.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class DiffDriveWheelchair : public rclcpp::Node {
public:
    DiffDriveWheelchair()
    : Node("diff_drive_wheelchair"), 
      prev_left_pos_(0), prev_right_pos_(0),
      first_callback_(true),
      wheel_base_(0.45), wheel_diameter_(0.01088)
    {
        left_wheel_sub_ = this->create_subscription<nurirobot_msgs::msg::NurirobotPos>(
            "left_wheel_pos", 10, std::bind(&DiffDriveWheelchair::leftWheelCallback, this, std::placeholders::_1));
        right_wheel_sub_ = this->create_subscription<nurirobot_msgs::msg::NurirobotPos>(
            "right_wheel_pos", 10, std::bind(&DiffDriveWheelchair::rightWheelCallback, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), 
            std::bind(&DiffDriveWheelchair::publishOdometry, this));
    }

private:
    void leftWheelCallback(const nurirobot_msgs::msg::NurirobotPos::SharedPtr msg) {
        left_pos_ = msg->pos;
    }

    void rightWheelCallback(const nurirobot_msgs::msg::NurirobotPos::SharedPtr msg) {
        right_pos_ = msg->pos;
    }

    void publishOdometry() {
        calculateOdometry();

        // Publish odom message
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_footprint";

        odom_msg.pose.pose.position.x = x_;
        odom_msg.pose.pose.position.y = y_;
        odom_msg.pose.pose.position.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation = tf2::toMsg(q);

        odom_pub_->publish(odom_msg);

        // Publish tf transform
        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.header.stamp = this->get_clock()->now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_footprint";

        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf2::toMsg(q);

        tf_broadcaster_->sendTransform(odom_trans);
    }

    void calculateOdometry() {
        if (first_callback_) {
            prev_left_pos_ = left_pos_;
            prev_right_pos_ = right_pos_;
            first_callback_ = false;
            return;
        }

        double delta_left = calculateDelta(prev_left_pos_, left_pos_);
        double delta_right = calculateDelta(prev_right_pos_, right_pos_);

        // Right motor position should be reversed
        delta_right = -delta_right;

        double delta_s = wheel_diameter_ * (delta_left + delta_right) / 2.0;
        double delta_theta = wheel_diameter_ * (delta_right - delta_left) / wheel_base_;

        double delta_x = delta_s * cos(theta_ + delta_theta / 2.0);
        double delta_y = delta_s * sin(theta_ + delta_theta / 2.0);

        x_ += delta_x;
        y_ += delta_y;
        theta_ += delta_theta;

        prev_left_pos_ = left_pos_;
        prev_right_pos_ = right_pos_;
    }

    double calculateDelta(uint16_t prev_pos, uint16_t current_pos) {
        int delta = current_pos - prev_pos;
        if (delta > 32766) delta -= 65533;
        if (delta < -32766) delta += 65533;
        return delta * (M_PI / 1800.0); // 0.1 degree to radian
    }

    rclcpp::Subscription<nurirobot_msgs::msg::NurirobotPos>::SharedPtr left_wheel_sub_;
    rclcpp::Subscription<nurirobot_msgs::msg::NurirobotPos>::SharedPtr right_wheel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    uint16_t prev_left_pos_;
    uint16_t prev_right_pos_;
    uint16_t left_pos_;
    uint16_t right_pos_;
    bool first_callback_;
    double wheel_base_;
    double wheel_diameter_;
    double x_ = 0.0;
    double y_ = 0.0;
    double theta_ = 0.0;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor executor;
    auto node = std::make_shared<DiffDriveWheelchair>();
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
}
