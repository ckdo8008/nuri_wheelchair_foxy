#ifndef NURI_NODE__ODOM_HPP
#define NURI_NODE__ODOM_HPP

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "nurirobot_msgs/msg/nurirobot_pos.hpp"
#include "nuri_odom/ComplementaryFilter.hpp"

#include <array>
#include <chrono>
#include <memory>
#include <string> 

class Odometry
{
public:
    explicit Odometry(
        std::shared_ptr<rclcpp::Node> &nh,
        const double wheels_separation,
        const double wheels_radius);
    virtual ~Odometry(){}

private:
    bool calculate_odometry(const rclcpp::Duration &duration);

    // void update_imu(const std::shared_ptr<sensor_msgs::msg::Imu const> &imu);
    void update_pos_state(const std::shared_ptr<nurirobot_msgs::msg::NurirobotPos const> &left_wheel_msg,
        const std::shared_ptr<nurirobot_msgs::msg::NurirobotPos const> &right_wheel_msg);
    // void update_joint_state(const std::shared_ptr<sensor_msgs::msg::JointState const> &joint_state);
    // void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg);
    // void joint_state_and_imu_callback(
    //     const std::shared_ptr<nurirobot_msgs::msg::NurirobotPos const> &left_wheel_msg,
    //     const std::shared_ptr<nurirobot_msgs::msg::NurirobotPos const> &right_wheel_msg,
    //     const std::shared_ptr<sensor_msgs::msg::Imu const> &imu_msg);
    void joint_state_and_imu_callback(
        const std::shared_ptr<nurirobot_msgs::msg::NurirobotPos const> &left_wheel_msg,
        const std::shared_ptr<nurirobot_msgs::msg::NurirobotPos const> &right_wheel_msg);    

    void publish(const rclcpp::Time &now);
    void update_joint_state(const rclcpp::Time &now);

    std::shared_ptr<rclcpp::Node> nh_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    // std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::JointState>> msg_ftr_joint_state_sub_;

    std::shared_ptr<message_filters::Subscriber<nurirobot_msgs::msg::NurirobotPos>> msg_ftr_left_wheel_sub_;
    std::shared_ptr<message_filters::Subscriber<nurirobot_msgs::msg::NurirobotPos>> msg_ftr_right_wheel_sub_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> msg_ftr_imu_sub_;

    // typedef message_filters::sync_policies::ApproximateTime<
    //     nurirobot_msgs::msg::NurirobotPos,
    //     nurirobot_msgs::msg::NurirobotPos,
    //     sensor_msgs::msg::Imu>
    //     SyncPolicyJointStateImu;
    typedef message_filters::sync_policies::ApproximateTime<
        nurirobot_msgs::msg::NurirobotPos,
        nurirobot_msgs::msg::NurirobotPos>
        SyncPolicyJointStateImu;    
    typedef message_filters::Synchronizer<SyncPolicyJointStateImu> SynchronizerJointStateImu;

    std::shared_ptr<SynchronizerJointStateImu> joint_state_imu_sync_;

    double wheels_separation_;
    double wheels_radius_;

    std::string frame_id_of_odometry_;
    std::string child_frame_id_of_odometry_;

    bool use_imu_;
    bool publish_tf_;

    std::array<uint16_t, 2> diff_positions_;
    std::array<double, 2> diff_joint_positions_;
    double imu_angle_;
    double imu_z_;

    std::array<double, 3> robot_pose_;
    std::array<double, 3> robot_vel_;

    ComplementaryFilter calc_yaw_;

    double calculate_angle_difference(uint16_t prev_angle, uint16_t current_angle);
    double degreesToRadians(double degrees);

    const double RPM_TO_MS = 0.229 * 0.0034557519189487725;
    const double TICK_TO_RAD = 0.001533981;
    double robot_vel_l_;
    double robot_vel_r_;
};

#endif