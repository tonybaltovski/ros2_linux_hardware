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

#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "linux_i2c_devices/lcm1602.hpp"
#include "rclcpp/logging.hpp"

namespace linux_i2c_devices
{
namespace
{
std::shared_ptr<linux_i2c_interface::I2cInterface> require_iface(
  std::shared_ptr<linux_i2c_interface::I2cInterface> iface)
{
  if (!iface)
  {
    throw std::invalid_argument("Lcm1602: i2c_interface must not be null");
  }
  return iface;
}

std::string make_log_name(const std::string & bus, uint8_t addr)
{
  const auto slash = bus.find_last_of('/');
  const std::string base = slash == std::string::npos ? bus : bus.substr(slash + 1);
  char buf[8];
  std::snprintf(buf, sizeof(buf), "0x%02x", static_cast<int>(addr));
  return "lcm1602." + base + "." + buf;
}
rclcpp::Logger logger(const std::string & name) { return rclcpp::get_logger(name); }
}  // namespace

Lcm1602::Lcm1602(
  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id, uint8_t rows,
  uint8_t columns)
: i2c_interface_(require_iface(std::move(i2c_interface))),
  device_id_(device_id),
  log_name_(make_log_name(i2c_interface_->bus_name(), device_id)),
  rows_(rows),
  columns_(columns),
  backlight_(LCM1602_BACKLIGHT_OFF),
  initialized_(false)
{
}

int Lcm1602::send(uint8_t value, uint32_t delay_us)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  uint8_t buffer = value | backlight_;
  if (i2c_interface_->write_to_bus(device_id_, buffer) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to write to I2C bus", __func__);
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
  return 0;
}

int Lcm1602::pulse_enable(uint8_t value)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  if (send(value | LCM1602_EN, 1) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to send enable pulse", __func__);
    return -1;
  }
  return send(value & ~LCM1602_EN, 50);
}

int Lcm1602::write_4bits(uint8_t value)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  if (send(value) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to send nibble", __func__);
    return -1;
  }
  return pulse_enable(value);
}

int Lcm1602::write(uint8_t value, uint8_t mode)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  if (write_4bits((value & 0xF0) | mode) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to write high nibble", __func__);
    return -1;
  }
  return write_4bits(((value << 4) & 0xF0) | mode);
}

int Lcm1602::command(uint8_t value, uint32_t delay_us)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  if (write(value, LCM1602_CMD) < 0)
  {
    RCLCPP_ERROR(
      logger(log_name_), "%s: Failed to write command 0x%02x", __func__, static_cast<int>(value));
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
  return 0;
}

int Lcm1602::initialize()
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  if (initialized_)
  {
    return 0;
  }

  RCLCPP_INFO(logger(log_name_), "%s: Starting initialization", __func__);
  std::this_thread::sleep_for(std::chrono::microseconds(400000));
  backlight_ = LCM1602_BACKLIGHT_ON;
  if (send(backlight_, 2000) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to enable backlight", __func__);
    return -1;
  }

  if (write_4bits(0x03 << 4) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed during 4-bit init sequence", __func__);
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(LCM1602_DELAY_4000_US));
  if (write_4bits(0x30) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed during 4-bit init sequence", __func__);
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(LCM1602_DELAY_4000_US));
  if (write_4bits(0x30) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed during 4-bit init sequence", __func__);
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(LCM1602_DELAY_100_US));
  if (write_4bits(0x20) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to switch to 4-bit mode", __func__);
    return -1;
  }

  if (
    command(LCM1602_FUNCTIONSET | LCM1602_2LINE) < 0 ||
    command(LCM1602_DISPLAYCONTROL | LCM1602_DISPLAYON) < 0 ||
    command(LCM1602_ENTRYMODESET | LCM1602_ENTRYLEFT) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to configure display", __func__);
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(LCM1602_DELAY_40000_US));
  if (clear() < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to clear display", __func__);
    return -1;
  }
  if (home() < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to home cursor", __func__);
    return -1;
  }
  RCLCPP_INFO(logger(log_name_), "%s: Initialization done", __func__);
  initialized_ = true;
  return 0;
}

int Lcm1602::stop()
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  int ret = 0;
  if (clear() < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to clear display", __func__);
    ret = -1;
  }
  backlight_ = LCM1602_BACKLIGHT_OFF;
  if (send(backlight_, LCM1602_DELAY_100_US) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to turn off backlight", __func__);
    ret = -1;
  }
  return ret;
}

int Lcm1602::clear()
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  if (command(LCM1602_CLEARDISPLAY) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to clear display", __func__);
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(LCM1602_DELAY_4000_US));
  return 0;
}

int Lcm1602::home()
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  if (command(LCM1602_RETURNHOME) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to return home", __func__);
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(LCM1602_DELAY_4000_US));
  return 0;
}

int Lcm1602::set_cursor(const uint8_t row, const uint8_t column)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  static constexpr uint8_t row_offsets[] = {0, 64, 20, 84};
  uint8_t offset = (row < 4) ? row_offsets[row] : 0;
  return command(LCM1602_SETDDRAMADDR | ((column % columns_) + offset));
}

int Lcm1602::print_char(char c) { return write(c, LCM1602_RS); }

int Lcm1602::print_msg(const std::string & msg)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  for (size_t i = 0; i < msg.length(); ++i)
  {
    uint8_t row = static_cast<uint8_t>(i / columns_);
    uint8_t col = static_cast<uint8_t>(i % columns_);
    if (set_cursor(row, col) < 0)
    {
      RCLCPP_ERROR(
        logger(log_name_), "%s: Failed to set cursor at row %d col %d", __func__,
        static_cast<int>(row), static_cast<int>(col));
      return -1;
    }
    if (print_char(msg[i]) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to print character", __func__);
      return -1;
    }
  }
  return 0;
}

int Lcm1602::turn_on()
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  return command(LCM1602_DISPLAYCONTROL | LCM1602_DISPLAYON);
}

int Lcm1602::turn_off()
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  return command(LCM1602_DISPLAYCONTROL);
}

}  // namespace linux_i2c_devices
