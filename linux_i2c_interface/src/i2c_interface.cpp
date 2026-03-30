// Copyright 2020 Tony Baltovski
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>

#include "linux_i2c_interface/i2c_interface.hpp"

namespace linux_i2c_interface
{

I2cInterface::I2cInterface(const std::string & i2c_bus) : i2c_bus_(i2c_bus) {}

I2cInterface::I2cInterface(uint8_t i2c_bus_number)
: i2c_bus_("/dev/i2c-" + std::to_string(i2c_bus_number))
{
}

int8_t I2cInterface::close_bus_unlocked()
{
  int ret = close(i2c_fd_);
  if (ret < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to close: " << strerror(errno) << std::endl;
  }
  else
  {
    is_connected_ = false;
  }
  return ret;
}

int8_t I2cInterface::open_bus()
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  std::cout << __PRETTY_FUNCTION__ << ": Trying to open: " << i2c_bus_ << std::endl;
  i2c_fd_ = open(i2c_bus_.c_str(), O_RDWR);

  if (i2c_fd_ < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Could not open: " << i2c_bus_
              << " with error: " << strerror(errno) << std::endl;
    return -1;
  }

  std::cout << __PRETTY_FUNCTION__ << ": Connected to " << i2c_bus_ << std::endl;
  is_connected_ = true;
  return 0;
}

int8_t I2cInterface::close_bus()
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);
  std::cout << __PRETTY_FUNCTION__ << ": Closing bus" << std::endl;
  return close_bus_unlocked();
}

bool I2cInterface::is_connected() const { return is_connected_; }

int8_t I2cInterface::set_device_id(const uint8_t device_id)
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  int ret = ioctl(i2c_fd_, I2C_SLAVE, device_id);
  if (ret < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to set device 0x" << std::hex
              << static_cast<int>(device_id) << std::dec << ": " << strerror(errno) << std::endl;
    close_bus_unlocked();
  }
  return ret;
}

int8_t I2cInterface::read_from_bus(const uint8_t address, void * data, uint32_t count)
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  if (write(i2c_fd_, &address, 1) != 1)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to write address: " << strerror(errno)
              << std::endl;
    close_bus_unlocked();
    return -1;
  }

  int ret = read(i2c_fd_, data, count);
  if (ret < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to read from device: " << strerror(errno)
              << std::endl;
    close_bus_unlocked();
    return -1;
  }
  else if (ret != static_cast<int>(count))
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Short read from device, expected " << count << ", got "
              << ret << std::endl;
    close_bus_unlocked();
    return -1;
  }
  return 0;
}

int8_t I2cInterface::write_to_bus(const uint8_t address)
{
  return write_to_bus(address, nullptr, 0);
}

int8_t I2cInterface::write_to_bus(const uint8_t device_id, const uint8_t address)
{
  int8_t ret = set_device_id(device_id);
  if (ret < 0)
  {
    return ret;
  }
  return write_to_bus(address, nullptr, 0);
}

int8_t I2cInterface::write_to_bus(const uint8_t address, void * data, uint32_t count)
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  if (write(i2c_fd_, &address, 1) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to write to device: " << strerror(errno)
              << std::endl;
    close_bus_unlocked();
    return -1;
  }

  if (count == 0)
  {
    return 0;
  }

  int ret = write(i2c_fd_, data, count);
  if (ret < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to write to device: " << strerror(errno)
              << std::endl;
    close_bus_unlocked();
    return -1;
  }
  else if (static_cast<int>(count) != ret)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Short write to device, expected " << count << ", got "
              << ret << std::endl;
    close_bus_unlocked();
    return -1;
  }
  return 0;
}

int8_t I2cInterface::write_raw(const void * data, uint32_t count)
{
  const std::lock_guard<std::mutex> lock(i2c_mutex_);

  int ret = write(i2c_fd_, data, count);
  if (ret < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to write to device: " << strerror(errno)
              << std::endl;
    close_bus_unlocked();
    return -1;
  }
  else if (static_cast<int>(count) != ret)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Short write to device, expected " << count << ", got "
              << ret << std::endl;
    close_bus_unlocked();
    return -1;
  }
  return 0;
}

}  // namespace linux_i2c_interface
