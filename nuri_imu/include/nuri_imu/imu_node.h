#ifndef NURI_IMU_IMU_NODE_H
#define NURI_IMU_IMU_NODE_H

#include "nuri_imu/mpu6050sensor.h"
#include "nuri_imu/hmc5883lsensor.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

extern "C" {
    #include "nuri_imu/MadgwickAHRS.h"
}



class IMU_Node: public rclcpp::Node {
    public:
        IMU_Node();

    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_raw_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_mag_;
        std::unique_ptr<MPU6050Sensor> mpu6050_;
        std::unique_ptr<HMC5883Sensor> hmc5883_;
        size_t count_;
        rclcpp::TimerBase::SharedPtr timer_;
        void handleInput();
        void declareParameters();
        float deg_to_rad = 0.0174533;  
};

#endif