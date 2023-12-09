#include "nuri_imu/imu_node.h"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

IMU_Node::IMU_Node()
: Node("nuri_imu_node"), mpu6050_{std::make_unique<MPU6050Sensor>()}, hmc5883_{std::make_unique<HMC5883Sensor>()}
{
  declareParameters();

  mpu6050_->setGyroscopeRange(
      static_cast<MPU6050Sensor::GyroRange>(this->get_parameter("gyro_range").as_int()));
  mpu6050_->setAccelerometerRange(
      static_cast<MPU6050Sensor::AccelRange>(this->get_parameter("accel_range").as_int()));
  mpu6050_->setDlpfBandwidth(
      static_cast<MPU6050Sensor::DlpfBandwidth>(this->get_parameter("dlpf_bandwidth").as_int()));
  mpu6050_->setGyroscopeOffset(this->get_parameter("gyro_x_offset").as_double(),
                               this->get_parameter("gyro_y_offset").as_double(),
                               this->get_parameter("gyro_z_offset").as_double());
  mpu6050_->setAccelerometerOffset(this->get_parameter("accel_x_offset").as_double(),
                                   this->get_parameter("accel_y_offset").as_double(),
                                   this->get_parameter("accel_z_offset").as_double());
  
  if (this->get_parameter("calibrate").as_bool()) {
    RCLCPP_INFO(this->get_logger(), "Calibrating...");
    mpu6050_->calibrate();
  }
  mpu6050_->printConfig();
  mpu6050_->printOffsets();

  publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
  publisher_raw_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  publisher_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("magnetic_field", 10);

  std::chrono::duration<int64_t, std::milli> frequency = 1000ms / this->get_parameter("frequency").as_int();
  timer_ = this->create_wall_timer(frequency, std::bind(&IMU_Node::handleInput, this));
}

void IMU_Node::declareParameters() 
{
  this->declare_parameter<bool>("calibrate", true);
  this->declare_parameter<int>("gyro_range", MPU6050Sensor::GyroRange::GYR_250_DEG_S);
  this->declare_parameter<int>("accel_range", MPU6050Sensor::AccelRange::ACC_2_G);
  this->declare_parameter<int>("dlpf_bandwidth", MPU6050Sensor::DlpfBandwidth::DLPF_260_HZ);
  this->declare_parameter<double>("gyro_x_offset", 0.0);
  this->declare_parameter<double>("gyro_y_offset", 0.0);
  this->declare_parameter<double>("gyro_z_offset", 0.0);
  this->declare_parameter<double>("accel_x_offset", 0.0);
  this->declare_parameter<double>("accel_y_offset", 0.0);
  this->declare_parameter<double>("accel_z_offset", 0.0);
  this->declare_parameter<int>("frequency", 0.0);  
}

void IMU_Node::handleInput()
{
  rclcpp::Time stamp = this->get_clock()->now();

  auto message = sensor_msgs::msg::Imu();
  message.header.stamp = stamp;
  message.header.frame_id = "imu_link_raw";
  auto messagemag = sensor_msgs::msg::MagneticField();
  messagemag.header.stamp = stamp;
  messagemag.header.frame_id = "imu_link";
  auto msg = sensor_msgs::msg::Imu();
  msg.header.stamp = stamp;
  msg.header.frame_id = "imu_link";
  
  message.linear_acceleration_covariance = {0};
  message.linear_acceleration.x = mpu6050_->getAccelerationX();
  message.linear_acceleration.y = mpu6050_->getAccelerationY();
  message.linear_acceleration.z = mpu6050_->getAccelerationZ();
  message.angular_velocity_covariance[0] = {0};
  message.angular_velocity.x = mpu6050_->getAngularVelocityX();
  message.angular_velocity.y = mpu6050_->getAngularVelocityY();
  message.angular_velocity.z = mpu6050_->getAngularVelocityZ();
  
  messagemag.magnetic_field.x = hmc5883_->getMagneticX();
  messagemag.magnetic_field.y = hmc5883_->getMagneticY();
  messagemag.magnetic_field.z = hmc5883_->getMagneticZ();  
  // Invalidate quaternion
  message.orientation_covariance[0] = -1;
  message.orientation.x = 0;
  message.orientation.y = 0;
  message.orientation.z = 0;
  message.orientation.w = 0;

  MadgwickAHRSupdate(
    (float)message.angular_velocity.x, (float)message.angular_velocity.y, (float)message.angular_velocity.z,
    (float)message.linear_acceleration.x, (float)message.linear_acceleration.y, (float)message.linear_acceleration.z,
    (float)messagemag.magnetic_field.x, (float)messagemag.magnetic_field.y, (float)messagemag.magnetic_field.z);

  msg.linear_acceleration_covariance = {0};
  msg.angular_velocity_covariance = {0};
  msg.orientation_covariance = {0};
  msg.linear_acceleration.x = message.linear_acceleration.x;
  msg.linear_acceleration.y = message.linear_acceleration.y;
  msg.linear_acceleration.z = message.linear_acceleration.z;
  msg.angular_velocity.x = message.angular_velocity.x;
  msg.angular_velocity.y = message.angular_velocity.y;
  msg.angular_velocity.z = message.angular_velocity.z;
  msg.orientation.x = q0;
  msg.orientation.y = q1;
  msg.orientation.z = q2;
  msg.orientation.w = q3;

  publisher_raw_->publish(message); 
  publisher_mag_->publish(messagemag);
  publisher_->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMU_Node>(); // What is auto??
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  return 0;
}
