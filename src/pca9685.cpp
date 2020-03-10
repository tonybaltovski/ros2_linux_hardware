#include <cstring>
#include <iostream>

#include "ros2_firmware/pca9685.hpp"

namespace ros2_firmware
{

Pca9685::Pca9685(I2cInterface& i2c_interface, uint8_t device_id) :
  i2c_interface_(i2c_interface),
  device_id_(device_id)
{
  i2c_interface_.open_bus();
  i2c_interface_.set_slave_addr(device_id_);
}


int Pca9685::initialize()
{

}

void Pca9685::stop()
{
  i2c_interface_.close_bus();
}

}  // namespace ros2_firmware
