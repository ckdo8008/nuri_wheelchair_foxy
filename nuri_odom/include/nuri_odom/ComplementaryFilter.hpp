#ifndef NURI_NODE__COMPLEMENTARY_FILTER
#define NURI_NODE__COMPLEMENTARY_FILTER

#include <iostream>

class ComplementaryFilter {
public:
    ComplementaryFilter();
    virtual ~ComplementaryFilter() {}
    std::string gyro_calibration(double gyro);
    double calc_filter(double gyro, double d_time);
    double wheel_ang;

private:
    double theta;
    double pre_theta;
    double filter_coef;
    double gyro_bias;
    int count_for_gyro_bias;    
};

#endif