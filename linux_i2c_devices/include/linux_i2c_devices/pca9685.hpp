#pragma once

#include <memory>
#include <unistd.h>

#include "linux_i2c_interface/i2c_interface.hpp"

#define PCA9685_DEFAULT_ADDRESS 0x40

#define PCA9685_MODE1         0x00
#define PCA9685_MODE2         0x01
#define PCA9685_SUBADR1       0x02
#define PCA9685_SUBADR2       0x03
#define PCA9685_SUBADR3       0x04
#define PCA9685_ALLCALLADR    0X05
#define PCA9685_LED00_ON_L    0x06
#define PCA9685_LED00_ON_H    0x07
#define PCA9685_LED00_OFF_L   0x08
#define PCA9685_LED00_OFF_H   0x09
#define PCA9685_LED01_ON_L    0x0A
#define PCA9685_LED01_ON_H    0x0B
#define PCA9685_LED01_OFF_L   0x0C
#define PCA9685_LED01_OFF_H   0x0D
#define PCA9685_LED02_ON_L    0x0E
#define PCA9685_LED02_ON_H    0x0F
#define PCA9685_LED02_OFF_L   0x10
#define PCA9685_LED02_OFF_H   0x11
#define PCA9685_LED03_ON_L    0x12
#define PCA9685_LED03_ON_H    0x13
#define PCA9685_LED03_OFF_L   0x14
#define PCA9685_LED03_OFF_H   0x15
#define PCA9685_LED04_ON_L    0x16
#define PCA9685_LED04_ON_H    0x17
#define PCA9685_LED04_OFF_L   0x18
#define PCA9685_LED04_OFF_H   0x19
#define PCA9685_LED05_ON_L    0x1A
#define PCA9685_LED05_ON_H    0x1B
#define PCA9685_LED05_OFF_L   0x1C
#define PCA9685_LED05_OFF_H   0x1D
#define PCA9685_LED06_ON_L    0x1E
#define PCA9685_LED06_ON_H    0x1F
#define PCA9685_LED06_OFF_L   0x20
#define PCA9685_LED06_OFF_H   0x21
#define PCA9685_LED07_ON_L    0x22
#define PCA9685_LED07_ON_H    0x23
#define PCA9685_LED07_OFF_L   0x24
#define PCA9685_LED07_OFF_H   0x25
#define PCA9685_LED08_ON_L    0x26
#define PCA9685_LED08_ON_H    0x27
#define PCA9685_LED08_OFF_L   0x28
#define PCA9685_LED08_OFF_H   0x29
#define PCA9685_LED09_ON_L    0x2A
#define PCA9685_LED09_ON_H    0x2B
#define PCA9685_LED09_OFF_L   0x2C
#define PCA9685_LED09_OFF_H   0x2D
#define PCA9685_LED10_ON_L    0x2E
#define PCA9685_LED10_ON_H    0x2F
#define PCA9685_LED10_OFF_L   0x30
#define PCA9685_LED10_OFF_H   0x31
#define PCA9685_LED11_ON_L    0x32
#define PCA9685_LED11_ON_H    0x33
#define PCA9685_LED11_OFF_L   0x34
#define PCA9685_LED11_OFF_H   0x35
#define PCA9685_LED12_ON_L    0x36
#define PCA9685_LED12_ON_H    0x37
#define PCA9685_LED12_OFF_L   0x38
#define PCA9685_LED12_OFF_H   0x39
#define PCA9685_LED13_ON_L    0x3A
#define PCA9685_LED13_ON_H    0x3B
#define PCA9685_LED13_OFF_L   0x3C
#define PCA9685_LED13_OFF_H   0x3D
#define PCA9685_LED14_ON_L    0x3E
#define PCA9685_LED14_ON_H    0x3F
#define PCA9685_LED14_OFF_L   0x40
#define PCA9685_LED14_OFF_H   0x41
#define PCA9685_LED15_ON_L    0x42
#define PCA9685_LED15_ON_H    0x43
#define PCA9685_LED15_OFF_L   0x44
#define PCA9685_LED15_OFF_H   0x45
#define PCA9685_ALL_LED_ON_L  0xFA
#define PCA9685_ALL_LED_ON_H  0xFB
#define PCA9685_ALL_LED_OFF_L 0xFC
#define PCA9685_ALL_LED_OFF_H 0xFD
#define PCA9685_PRE_SCALE     0xFE
#define PCA9685_TEST_MODE     0xFF

namespace linux_i2c_devices
{

class Pca9685
{
public:
  Pca9685(std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id);
  int initialize();
  void stop();

private:
  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface_;
  uint8_t device_id_;
};

}  // namespace linux_i2c_devices
