#include "nuri_odom/odometry.hpp"

#include <memory>
#include <string>
#include <utility>

using namespace std::chrono_literals;

Odometry::Odometry(
    std::shared_ptr<rclcpp::Node> &nh,
    const double wheels_separation,
    const double wheels_radius)
    : nh_(nh),
      wheels_separation_(wheels_separation),
      wheels_radius_(wheels_radius),
      publish_tf_(true),
      imu_angle_(0.0f),
      imu_z_(0.0f)
{
    RCLCPP_INFO(nh_->get_logger(), "Init Odometry");

    nh_->declare_parameter("odometry.frame_id");
    nh_->declare_parameter("odometry.child_frame_id");

    nh_->declare_parameter("odometry.publish_tf");

    nh_->get_parameter_or<bool>(
        "odometry.publish_tf",
        publish_tf_,
        true);

    nh_->get_parameter_or<std::string>(
        "odometry.frame_id",
        frame_id_of_odometry_,
        std::string("odom"));

    nh_->get_parameter_or<std::string>(
        "odometry.child_frame_id",
        child_frame_id_of_odometry_,
        std::string("base_footprint"));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    odom_pub_ = nh_->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
    // joint_state_pub_ = nh_->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);

    calc_yaw_ = ComplementaryFilter();

    robot_pose_[0] = 0.0;
    robot_pose_[1] = 0.0;
    robot_pose_[2] = 0.0;

    robot_vel_[0] = 0.0;
    robot_vel_[1] = 0.0;
    robot_vel_[2] = 0.0;

    uint32_t queue_size = 10;
    joint_state_imu_sync_ = std::make_shared<SynchronizerJointStateImu>(queue_size);

    msg_ftr_left_wheel_sub_ =
        std::make_shared<message_filters::Subscriber<nurirobot_msgs::msg::NurirobotPos>>(
            nh_,
            "left_wheel_pos");

    msg_ftr_right_wheel_sub_ =
        std::make_shared<message_filters::Subscriber<nurirobot_msgs::msg::NurirobotPos>>(
            nh_,
            "right_wheel_pos");

    joint_state_imu_sync_->connectInput(*msg_ftr_left_wheel_sub_, *msg_ftr_right_wheel_sub_);

    joint_state_imu_sync_->setInterMessageLowerBound(
        0,
        rclcpp::Duration(10ms));

    joint_state_imu_sync_->setInterMessageLowerBound(
        1,
        rclcpp::Duration(10ms));

    joint_state_imu_sync_->registerCallback(
        std::bind(
            &Odometry::joint_state_and_imu_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
}

void Odometry::joint_state_and_imu_callback(
    const std::shared_ptr<nurirobot_msgs::msg::NurirobotPos const> &left_wheel_msg,
    const std::shared_ptr<nurirobot_msgs::msg::NurirobotPos const> &right_wheel_msg)
{
    static rclcpp::Time last_time = nh_->get_clock()->now();
    rclcpp::Time current_time = nh_->get_clock()->now();
    rclcpp::Duration duration(current_time.nanoseconds() - last_time.nanoseconds());

    update_pos_state(left_wheel_msg, right_wheel_msg);
    calculate_odometry(duration);

    publish(current_time);
    // update_joint_state(left_wheel_msg->header.stamp);

    last_time = current_time;
}

void Odometry::publish(const rclcpp::Time &now)
{
    auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();

    odom_msg->header.frame_id = frame_id_of_odometry_;
    odom_msg->child_frame_id = child_frame_id_of_odometry_;
    odom_msg->header.stamp = now;

    odom_msg->pose.pose.position.x = robot_pose_[0];
    odom_msg->pose.pose.position.y = robot_pose_[1];
    odom_msg->pose.pose.position.z = 0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, robot_pose_[2]);

    odom_msg->pose.pose.orientation.x = q.x();
    odom_msg->pose.pose.orientation.y = q.y();
    odom_msg->pose.pose.orientation.z = q.z();
    odom_msg->pose.pose.orientation.w = q.w();

    odom_msg->twist.twist.linear.x = robot_vel_[0];
    odom_msg->twist.twist.angular.z = robot_vel_[2];

    geometry_msgs::msg::TransformStamped odom_tf;

    odom_tf.transform.translation.x = odom_msg->pose.pose.position.x;
    odom_tf.transform.translation.y = odom_msg->pose.pose.position.y;
    odom_tf.transform.translation.z = odom_msg->pose.pose.position.z;
    odom_tf.transform.rotation = odom_msg->pose.pose.orientation;

    odom_tf.header.frame_id = frame_id_of_odometry_;
    odom_tf.child_frame_id = child_frame_id_of_odometry_;
    odom_tf.header.stamp = now;

    odom_pub_->publish(std::move(odom_msg));

    if (publish_tf_)
    {
        tf_broadcaster_->sendTransform(odom_tf);
    }
}

// void Odometry::update_joint_state(const rclcpp::Time &now)
// {
//     auto msg = std::make_unique<sensor_msgs::msg::JointState>();
//     msg->header.frame_id = "base_link";
//     msg->header.stamp  = now;
//     msg->name.push_back("wheel_left_joint");
//     msg->name.push_back("wheel_right_joint");

//     msg->position.push_back(TICK_TO_RAD * degreesToRadians(diff_joint_positions_[0]));
//     msg->position.push_back(TICK_TO_RAD * degreesToRadians(diff_joint_positions_[1]));

//     msg->velocity.push_back(RPM_TO_MS * robot_vel_l_);
//     msg->velocity.push_back(RPM_TO_MS * robot_vel_r_);

//     joint_state_pub_->publish(std::move(msg));
// }

void Odometry::update_pos_state(const std::shared_ptr<nurirobot_msgs::msg::NurirobotPos const> &left_wheel_msg,
                                const std::shared_ptr<nurirobot_msgs::msg::NurirobotPos const> &right_wheel_msg)
{
    static std::array<uint16_t, 2> last_joint_positions = {left_wheel_msg->pos, right_wheel_msg->pos};

    diff_joint_positions_[0] = calculate_angle_difference(last_joint_positions[0], left_wheel_msg->pos);
    diff_joint_positions_[1] = calculate_angle_difference(last_joint_positions[1], right_wheel_msg->pos) * -1;

    last_joint_positions[0] = left_wheel_msg->pos;
    last_joint_positions[1] = right_wheel_msg->pos;
}

bool Odometry::calculate_odometry(const rclcpp::Duration &duration)
{
    // rotation value of wheel [rad]
    double wheel_l = degreesToRadians(diff_joint_positions_[0]);
    double wheel_r = degreesToRadians(diff_joint_positions_[1]);

    double delta_s = 0.0;
    double delta_theta = 0.0;

    double theta = 0.0;
    static double last_theta = 0.0;

    double v = 0.0;
    double w = 0.0;


    double step_time = duration.seconds();

    if (step_time == 0.0)
    {
        // RCLCPP_INFO(nh_->get_logger(), "return ===================== false");
        return false;
    }

    if (std::isnan(wheel_l))
    {
        wheel_l = 0.0;
    }

    if (std::isnan(wheel_r))
    {
        wheel_r = 0.0;
    }

    delta_s = wheels_radius_ * (wheel_r + wheel_l) / 2.0;
    theta = wheels_radius_ * (wheel_r - wheel_l) / wheels_separation_;
    delta_theta = theta;    

    // compute odometric pose
    robot_pose_[0] += delta_s * cos(robot_pose_[2] + (delta_theta / 2.0));
    robot_pose_[1] += delta_s * sin(robot_pose_[2] + (delta_theta / 2.0));
    robot_pose_[2] += delta_theta;

    RCLCPP_DEBUG(nh_->get_logger(), "x : %f, y : %f, z : %f", robot_pose_[0], robot_pose_[1], robot_pose_[2]);

    // compute odometric instantaneouse velocity
    v = delta_s / step_time;
    w = delta_theta / step_time;

    robot_vel_l_ = wheels_radius_ * wheel_l / step_time;
    robot_vel_r_ = wheels_radius_ * wheel_r / step_time;

    robot_vel_[0] = v;
    robot_vel_[1] = 0.0;
    robot_vel_[2] = w;

    last_theta = theta;
    return true;
}

double Odometry::calculate_angle_difference(uint16_t prev_angle, uint16_t current_angle)
{
    int diff = current_angle - prev_angle;

    if (diff > 32766)
    { // 반대 방향으로 큰 변화가 감지된 경우 (0에서 65533로의 점프)
        diff -= 65533;
    }
    else if (diff < -32766)
    { // 반대 방향으로 큰 변화가 감지된 경우 (65533에서 0으로의 점프)
        diff += 65533;
    }

    return static_cast<double>(diff) / 10.0;
}

double Odometry::degreesToRadians(double degrees)
{
    return degrees * M_PI / 180.0;
}

