#include "nuri_odom/ComplementaryFilter.hpp"

ComplementaryFilter::ComplementaryFilter()
: theta(0.0), pre_theta(0.0), wheel_ang(0.0), filter_coef(2.5), gyro_bias(0.0), count_for_gyro_bias(110) {}

std::string ComplementaryFilter::gyro_calibration(double gyro)
{
    --count_for_gyro_bias;

    if (count_for_gyro_bias > 100) {
        return "Prepare for gyro_calibration";
    }

    gyro_bias += gyro;
    if (count_for_gyro_bias == 1) {
        gyro_bias /= 100;
        std::cout << "Complete : Gyro calibration" << std::endl;
        return "gyro_calibration OK";
    }

    return "During gyro_calibration";
}

double ComplementaryFilter::calc_filter(double gyro, double d_time)
{
    if (count_for_gyro_bias != 1) {
        std::string tmp = gyro_calibration(gyro);
        return 0;
    }

    gyro -= gyro_bias;
    pre_theta = theta;
    double temp = -1 / filter_coef * (-wheel_ang + pre_theta) + gyro;
    theta = pre_theta + temp * d_time;
    return theta;
}
