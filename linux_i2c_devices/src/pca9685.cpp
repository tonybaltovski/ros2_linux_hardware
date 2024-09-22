#include <cstring>
#include <iostream>

#include "linux_i2c_devices/pca9685.hpp"

namespace linux_i2c_devices
{

Pca9685::Pca9685(std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id) :
  i2c_interface_(i2c_interface),
  device_id_(device_id)
{
  i2c_interface_->open_bus();
  i2c_interface_->set_device_id(device_id_);
}


int Pca9685::initialize()
{

}

void Pca9685::stop()
{
  i2c_interface_->close_bus();
}

}  // namespace linux_i2c_devices
