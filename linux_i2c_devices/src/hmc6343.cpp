// Copyright 2026 Tony Baltovski
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

#include "linux_i2c_devices/hmc6343.hpp"
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
    throw std::invalid_argument("Hmc6343: i2c_interface must not be null");
  }
  return iface;
}

std::string make_log_name(const std::string & bus, uint8_t addr)
{
  const auto slash = bus.find_last_of('/');
  const std::string base = slash == std::string::npos ? bus : bus.substr(slash + 1);
  char buf[8];
  std::snprintf(buf, sizeof(buf), "0x%02x", static_cast<int>(addr));
  return "hmc6343." + base + "." + buf;
}

rclcpp::Logger logger(const std::string & name) { return rclcpp::get_logger(name); }

/// @brief Decode a big-endian int16 (HMC6343 native byte order).
int16_t be16(const uint8_t * p)
{
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | p[1]);
}
}  // namespace

Hmc6343::Hmc6343(
  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id)
: i2c_interface_(require_iface(std::move(i2c_interface))),
  device_id_(device_id),
  log_name_(make_log_name(i2c_interface_->bus_name(), device_id)),
  initialized_(false)
{
}

int Hmc6343::post_and_read(uint8_t cmd, uint8_t * buffer, uint32_t count, uint32_t delay_us)
{
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(
        logger(log_name_), "%s: Failed to reserve bus for command 0x%02x", __func__, cmd);
      return -1;
    }
    if (i2c_transaction.write_cmd(cmd) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to post command 0x%02x", __func__, cmd);
      return -1;
    }
  }
  // Bus released so other drivers can proceed during the device's processing
  // time; the HMC6343 retains the posted output until the next command.
  std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to reserve bus for read", __func__);
      return -1;
    }
    if (i2c_transaction.read_raw(buffer, count) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to read %u bytes", __func__, count);
      return -1;
    }
  }
  return 0;
}

int Hmc6343::read_eeprom(uint8_t address, uint8_t & value)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to reserve bus for EEPROM read", __func__);
      return -1;
    }
    const uint8_t payload[2] = {HMC6343_CMD_READ_EEPROM, address};
    if (i2c_transaction.write_raw(payload, sizeof(payload)) < 0)
    {
      RCLCPP_ERROR(
        logger(log_name_), "%s: Failed to post EEPROM read of 0x%02x", __func__, address);
      return -1;
    }
  }
  std::this_thread::sleep_for(std::chrono::microseconds(HMC6343_EEPROM_READ_DELAY_US));
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to reserve bus for EEPROM read", __func__);
      return -1;
    }
    if (i2c_transaction.read_raw(&value, 1) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to read EEPROM byte 0x%02x", __func__, address);
      return -1;
    }
  }
  return 0;
}

int Hmc6343::initialize()
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  if (initialized_)
  {
    return 0;
  }

  RCLCPP_INFO(logger(log_name_), "%s: Starting initialization", __func__);

  uint8_t eeprom_addr_byte = 0;
  if (read_eeprom(HMC6343_EEPROM_SLAVE_ADDR, eeprom_addr_byte) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to read EEPROM slave-address byte", __func__);
    return -1;
  }
  // EEPROM stores the 8-bit (write-direction) address, i.e. 7-bit << 1.
  const uint8_t expected = static_cast<uint8_t>(device_id_ << 1);
  if (eeprom_addr_byte != expected)
  {
    RCLCPP_WARN(
      logger(log_name_),
      "%s: EEPROM slave address 0x%02x does not match expected 0x%02x (continuing)", __func__,
      static_cast<int>(eeprom_addr_byte), static_cast<int>(expected));
  }

  initialized_ = true;
  RCLCPP_INFO(logger(log_name_), "%s: Initialization done", __func__);
  return 0;
}

int Hmc6343::stop()
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
  if (!i2c_transaction.ok())
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to reserve bus", __func__);
    return -1;
  }
  if (i2c_transaction.write_cmd(HMC6343_CMD_ENTER_SLEEP) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to enter sleep", __func__);
    return -1;
  }
  initialized_ = false;
  return 0;
}

int Hmc6343::read_field_raw(int16_t & x, int16_t & y, int16_t & z)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  uint8_t buf[6];
  if (post_and_read(HMC6343_CMD_POST_MAG, buf, sizeof(buf), HMC6343_POST_DELAY_US) < 0)
  {
    return -1;
  }
  x = be16(&buf[0]);
  y = be16(&buf[2]);
  z = be16(&buf[4]);
  return 0;
}

int Hmc6343::read_field_ut(double & x_ut, double & y_ut, double & z_ut)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
  if (read_field_raw(x, y, z) < 0)
  {
    return -1;
  }
  x_ut = x * HMC6343_MAG_LSB_TO_UT;
  y_ut = y * HMC6343_MAG_LSB_TO_UT;
  z_ut = z * HMC6343_MAG_LSB_TO_UT;
  return 0;
}

int Hmc6343::read_heading_deg(double & heading_deg)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  uint8_t buf[6];
  if (post_and_read(HMC6343_CMD_POST_HEADING, buf, sizeof(buf), HMC6343_POST_DELAY_US) < 0)
  {
    return -1;
  }
  // Heading is reported in [0, 3599] tenths of a degree.
  heading_deg = be16(&buf[0]) * HMC6343_ANGLE_LSB_TO_DEG;
  return 0;
}

int Hmc6343::read_tilt_deg(double & pitch_deg, double & roll_deg)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  uint8_t buf[6];
  if (post_and_read(HMC6343_CMD_POST_TILT, buf, sizeof(buf), HMC6343_POST_DELAY_US) < 0)
  {
    return -1;
  }
  pitch_deg = be16(&buf[0]) * HMC6343_ANGLE_LSB_TO_DEG;
  roll_deg = be16(&buf[2]) * HMC6343_ANGLE_LSB_TO_DEG;
  return 0;
}

int Hmc6343::read_accel_g(double & x_g, double & y_g, double & z_g)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  uint8_t buf[6];
  if (post_and_read(HMC6343_CMD_POST_ACCEL, buf, sizeof(buf), HMC6343_POST_DELAY_US) < 0)
  {
    return -1;
  }
  // Datasheet: 1 g = 1024 LSB on the on-chip accelerometer.
  constexpr double k_g_per_lsb = 1.0 / 1024.0;
  x_g = be16(&buf[0]) * k_g_per_lsb;
  y_g = be16(&buf[2]) * k_g_per_lsb;
  z_g = be16(&buf[4]) * k_g_per_lsb;
  return 0;
}

}  // namespace linux_i2c_devices
