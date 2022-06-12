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
  i2c_interface_.set_device_id(device_id_);
}


void Pca9685::initialize()
{
  uint8_t mode1 = 0b10000000;
  i2c_interface_.write_to_bus(PCA9685_MODE1, model1, 1);
  usleep(500000);

  mode1 = 0b00010000;
  i2c_interface_.write_to_bus(PCA9685_MODE1, model1, 1);
  usleep(2500);

  uint8_t prescale = static_cast<uint8_t>(25000000.0 / 4096.0 / 1600 + 0.5);
  i2c_interface_.write_to_bus(PCA9685_PRE_SCALE, prescale, 1);
  usleep(2500);

  uint8_t mode2 = 0x04;
  i2c_interface_.write_to_bus(PCA9685_MODE2, model2, 1);
  usleep(2500);

  mode1 = 0xA1;
  i2c_interface_.write_to_bus(PCA9685_MODE1, model1, 1);
  usleep(2500);

  uint8_t value = 0x00;
  i2c_interface_.write_to_bus(PCA9685_CHANNEL_ALL_REG + 0, value, 1);
  i2c_interface_.write_to_bus(PCA9685_CHANNEL_ALL_REG + 1, value, 1);
  i2c_interface_.write_to_bus(PCA9685_CHANNEL_ALL_REG + 2, value, 1);
  i2c_interface_.write_to_bus(PCA9685_CHANNEL_ALL_REG + 3, value, 1);
}

void Pca9685::stop()
{
  i2c_interface_.close_bus();
}

void Pca9685::reset()
{

}
void Pca9685::sleep()
{

}

void Pca9685::write(const uint8_t pin, const uint16_t value)
{
  i2c_interface_.write_to_bus(PCA9685_LED00_ON_L + 4 * pin, &value, 2);
}

void Pca9685::write_microseconds(const uint8_t pin, const uint32_t microseconds)
{

}

}  // namespace ros2_firmware
