#ifndef HMC5883SENSOR_H
#define HMC5883SENSOR_H

#include <string>
#include <unordered_map>

class HMC5883Sensor {
 public:
  HMC5883Sensor(int bus_number = 0);
  ~HMC5883Sensor();

  double getMagneticX() const;
  double getMagneticY() const;
  double getMagneticZ() const;

 private:
  void reportError(int error);

  int file_;
  char filename_[10] = "/dev/i2c-";

  // HMC5883 registers and addresses (s. datasheet for details)
  static constexpr int HMC5883_ADDRESS_DEFAULT = 0x1E;
  static constexpr int MAG_XOUT_H = 0x03;
  static constexpr int MAG_YOUT_H = 0x07;
  static constexpr int MAG_ZOUT_H = 0x05;

};

#endif  // HMC5883SENSOR_H
