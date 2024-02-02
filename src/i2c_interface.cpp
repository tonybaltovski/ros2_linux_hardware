
#include <iostream>

#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "ros2_firmware/i2c_interface.hpp"

namespace ros2_firmware
{

I2cInterface::I2cInterface(const std::string& i2c_bus) :
  i2c_bus_(i2c_bus) ,
  is_connected_(false)
{
}

I2cInterface::I2cInterface(const uint8_t& i2c_bus_number) :
  i2c_bus_("/dev/i2c-") ,
  is_connected_(false)
{
  this->i2c_bus_ += std::to_string(i2c_bus_number);
}

int8_t I2cInterface::open_bus()
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  std::cout << __PRETTY_FUNCTION__ << ": Trying to open: " << this->i2c_bus_ << std::endl;
  this->i2c_fd_ = open(this->i2c_bus_.c_str(), O_RDWR);

  if (this->i2c_fd_ < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ <<  ": Could not open: "<< this->i2c_bus_
              << " with error: " << strerror(errno) << std::endl;
    close(this->i2c_fd_);
    return -1;
  }
  else
  {
    std::cout << __PRETTY_FUNCTION__ <<  ": Connected to "  << this->i2c_bus_ << std::endl;
  }
  this->is_connected_ = true;
  return 0;
}

int8_t I2cInterface::close_bus()
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  std::cout << __PRETTY_FUNCTION__ << ": Closing bus" << std::endl;
  int ret = close(this->i2c_fd_);
  if (ret < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to close : "
              << strerror(errno) << std::endl;
  }
  else
    this->is_connected_ = false;
  return ret;
}

bool I2cInterface::is_connected() const
{
  return is_connected_;
}

int8_t I2cInterface::set_device_id(const uint8_t device_id)
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  int ret = ioctl(this->i2c_fd_, I2C_SLAVE, device_id);
  if (ret < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to " << static_cast<int>(device_id) << " device: "
              << strerror(errno) << std::endl;
    this->close_bus();
  }
  // else
  // {
  //   std::cout << __PRETTY_FUNCTION__ << ": Set device ID to 0x"  << std::hex
  //             << static_cast<int>(device_id) << std::endl;
  // }
  return ret;
}

int8_t I2cInterface::read_from_bus(const uint8_t address, void* data, uint32_t count)
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  if (write(this->i2c_fd_, &address, 1) != 1)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to write to address: " << strerror(errno) << std::endl;
    this->close_bus();
    return -1 ;
  }
  int ret = read(this->i2c_fd_, data, count);

  if (ret < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to read device(%d): " << strerror(errno) << std::endl;
    this->close_bus();
    return -1 ;
  }
  else if (ret != static_cast<int>(count))
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Short read from device, expected" << count << " , got "
              << ret << std::endl;
    this->close_bus();
    return -1 ;
  }
  return 0;
}

int8_t I2cInterface::write_to_bus(const uint8_t address)
{
  return this->write_to_bus(address, nullptr, 0);
}

int8_t I2cInterface::write_to_bus(const uint8_t device_id, const uint8_t address)
{
  int8_t ret = 0;
  ret = this->set_device_id(device_id);
  ret = this->write_to_bus(address, nullptr, 0);
  return ret;
}

int8_t I2cInterface::write_to_bus(const uint8_t address, void* data, uint32_t count)
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  if (write(this->i2c_fd_, &address, 1) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to write device: " << strerror(errno) << std::endl;
  }

  if (count == 0)
  {
    return 0;
  }

  int ret = write(this->i2c_fd_, data, count);
  if (ret < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to write device: " << strerror(errno) << std::endl;
    this->close_bus();
    return -1;
  }
  else if (static_cast<int>(count) != ret)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Short write to device, expected " << count <<", got " << ret
              << std::endl;
    this->close_bus();
    return -1;
  }
  return 0;
}
// int8_t I2cInterface::write_to_bus(uint8_t device_id, uint8_t address,
//                     void* data, uint32_t count)
// {
//
// }

}  // namespace ros2_firmware
