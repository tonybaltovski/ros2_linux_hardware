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

#include "linux_i2c_devices/bmi160.hpp"
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
    throw std::invalid_argument("Bmi160: i2c_interface must not be null");
  }
  return iface;
}

std::string make_log_name(const std::string & bus, uint8_t addr)
{
  const auto slash = bus.find_last_of('/');
  const std::string base = slash == std::string::npos ? bus : bus.substr(slash + 1);
  char buf[8];
  std::snprintf(buf, sizeof(buf), "0x%02x", static_cast<int>(addr));
  return "bmi160." + base + "." + buf;
}

rclcpp::Logger logger(const std::string & name) { return rclcpp::get_logger(name); }

/// @brief Decode a little-endian int16 (BMI160 native byte order).
int16_t le16(const uint8_t * p)
{
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

double accel_scale(Bmi160AccelRange range)
{
  switch (range)
  {
    case Bmi160AccelRange::RANGE_2G:
      return 1.0 / 16384.0;
    case Bmi160AccelRange::RANGE_4G:
      return 1.0 / 8192.0;
    case Bmi160AccelRange::RANGE_8G:
      return 1.0 / 4096.0;
    case Bmi160AccelRange::RANGE_16G:
      return 1.0 / 2048.0;
  }
  return 1.0 / 16384.0;
}

double gyro_scale(Bmi160GyroRange range)
{
  switch (range)
  {
    case Bmi160GyroRange::RANGE_2000_DPS:
      return 1.0 / 16.4;
    case Bmi160GyroRange::RANGE_1000_DPS:
      return 1.0 / 32.8;
    case Bmi160GyroRange::RANGE_500_DPS:
      return 1.0 / 65.6;
    case Bmi160GyroRange::RANGE_250_DPS:
      return 1.0 / 131.2;
    case Bmi160GyroRange::RANGE_125_DPS:
      return 1.0 / 262.4;
  }
  return 1.0 / 16.4;
}
}  // namespace

Bmi160::Bmi160(
  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id,
  Bmi160AccelRange accel_range, Bmi160GyroRange gyro_range)
: i2c_interface_(require_iface(std::move(i2c_interface))),
  device_id_(device_id),
  accel_range_(accel_range),
  gyro_range_(gyro_range),
  accel_lsb_to_g_(accel_scale(accel_range)),
  gyro_lsb_to_dps_(gyro_scale(gyro_range)),
  log_name_(make_log_name(i2c_interface_->bus_name(), device_id)),
  initialized_(false)
{
}

int Bmi160::initialize()
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  if (initialized_)
  {
    return 0;
  }

  RCLCPP_INFO(logger(log_name_), "%s: Starting initialization", __func__);

  // 1) Verify CHIP_ID.
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to reserve bus", __func__);
      return -1;
    }
    uint8_t chip_id = 0;
    if (i2c_transaction.read(BMI160_REG_CHIP_ID, &chip_id, 1) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to read CHIP_ID", __func__);
      return -1;
    }
    if (chip_id != BMI160_CHIP_ID_VALUE)
    {
      RCLCPP_ERROR(
        logger(log_name_), "%s: Unexpected CHIP_ID 0x%02x (expected 0x%02x)", __func__,
        static_cast<int>(chip_id), static_cast<int>(BMI160_CHIP_ID_VALUE));
      return -1;
    }
  }

  // 2) Soft reset.
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to reserve bus for reset", __func__);
      return -1;
    }
    const uint8_t v = BMI160_CMD_SOFT_RESET;
    if (i2c_transaction.write(BMI160_REG_CMD, &v, 1) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to soft-reset", __func__);
      return -1;
    }
  }
  std::this_thread::sleep_for(std::chrono::microseconds(BMI160_SOFT_RESET_DELAY_US));

  // 3) Bring accelerometer to normal power mode.
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to reserve bus for acc wake", __func__);
      return -1;
    }
    const uint8_t v = BMI160_CMD_ACC_NORMAL;
    if (i2c_transaction.write(BMI160_REG_CMD, &v, 1) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to set accel to normal mode", __func__);
      return -1;
    }
  }
  std::this_thread::sleep_for(std::chrono::microseconds(BMI160_ACC_WAKEUP_DELAY_US));

  // 4) Bring gyroscope to normal power mode.
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to reserve bus for gyr wake", __func__);
      return -1;
    }
    const uint8_t v = BMI160_CMD_GYR_NORMAL;
    if (i2c_transaction.write(BMI160_REG_CMD, &v, 1) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to set gyro to normal mode", __func__);
      return -1;
    }
  }
  std::this_thread::sleep_for(std::chrono::microseconds(BMI160_GYR_WAKEUP_DELAY_US));

  // 5) Configure full-scale ranges.
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to reserve bus for range config", __func__);
      return -1;
    }
    const uint8_t acc_range = static_cast<uint8_t>(accel_range_);
    if (i2c_transaction.write(BMI160_REG_ACC_RANGE, &acc_range, 1) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to write ACC_RANGE", __func__);
      return -1;
    }
    const uint8_t gyr_range = static_cast<uint8_t>(gyro_range_);
    if (i2c_transaction.write(BMI160_REG_GYR_RANGE, &gyr_range, 1) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to write GYR_RANGE", __func__);
      return -1;
    }
  }

  // 6) Sanity check: PMU_STATUS should show normal mode for both blocks.
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to reserve bus for PMU check", __func__);
      return -1;
    }
    uint8_t pmu = 0;
    if (i2c_transaction.read(BMI160_REG_PMU_STATUS, &pmu, 1) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to read PMU_STATUS", __func__);
      return -1;
    }
    // PMU_STATUS bits [5:4] = acc_pmu_status, [3:2] = gyr_pmu_status; 0b01 = normal.
    const uint8_t acc_pmu = (pmu >> 4) & 0x03;
    const uint8_t gyr_pmu = (pmu >> 2) & 0x03;
    if (acc_pmu != 0x01 || gyr_pmu != 0x01)
    {
      RCLCPP_WARN(
        logger(log_name_), "%s: PMU_STATUS=0x%02x (acc_pmu=%u gyr_pmu=%u; expected 1/1)", __func__,
        static_cast<int>(pmu), static_cast<unsigned>(acc_pmu), static_cast<unsigned>(gyr_pmu));
    }
  }

  initialized_ = true;
  RCLCPP_INFO(logger(log_name_), "%s: Initialization done", __func__);
  return 0;
}

int Bmi160::stop()
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
  if (!i2c_transaction.ok())
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to reserve bus", __func__);
    return -1;
  }
  const uint8_t gyr_suspend = BMI160_CMD_GYR_SUSPEND;
  if (i2c_transaction.write(BMI160_REG_CMD, &gyr_suspend, 1) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to suspend gyro", __func__);
    return -1;
  }
  const uint8_t acc_suspend = BMI160_CMD_ACC_SUSPEND;
  if (i2c_transaction.write(BMI160_REG_CMD, &acc_suspend, 1) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to suspend accel", __func__);
    return -1;
  }
  initialized_ = false;
  return 0;
}

int Bmi160::read_imu_raw(
  int16_t & ax, int16_t & ay, int16_t & az, int16_t & gx, int16_t & gy, int16_t & gz)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  uint8_t buf[12];  // gyro X/Y/Z, accel X/Y/Z; little-endian int16.
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to reserve bus", __func__);
      return -1;
    }
    if (i2c_transaction.read(BMI160_REG_DATA_GYRO_X_LSB, buf, sizeof(buf)) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to read sensor data", __func__);
      return -1;
    }
  }
  gx = le16(&buf[0]);
  gy = le16(&buf[2]);
  gz = le16(&buf[4]);
  ax = le16(&buf[6]);
  ay = le16(&buf[8]);
  az = le16(&buf[10]);
  return 0;
}

int Bmi160::read_imu(
  double & ax_g, double & ay_g, double & az_g, double & gx_dps, double & gy_dps, double & gz_dps)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;
  int16_t gx = 0;
  int16_t gy = 0;
  int16_t gz = 0;
  if (read_imu_raw(ax, ay, az, gx, gy, gz) < 0)
  {
    return -1;
  }
  ax_g = ax * accel_lsb_to_g_;
  ay_g = ay * accel_lsb_to_g_;
  az_g = az * accel_lsb_to_g_;
  gx_dps = gx * gyro_lsb_to_dps_;
  gy_dps = gy * gyro_lsb_to_dps_;
  gz_dps = gz * gyro_lsb_to_dps_;
  return 0;
}

int Bmi160::read_accel_g(double & x_g, double & y_g, double & z_g)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  double gx_dps = 0.0;
  double gy_dps = 0.0;
  double gz_dps = 0.0;
  return read_imu(x_g, y_g, z_g, gx_dps, gy_dps, gz_dps);
}

int Bmi160::read_gyro_dps(double & x_dps, double & y_dps, double & z_dps)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  double ax_g = 0.0;
  double ay_g = 0.0;
  double az_g = 0.0;
  return read_imu(ax_g, ay_g, az_g, x_dps, y_dps, z_dps);
}

int Bmi160::read_temperature_c(double & temperature_c)
{
  std::lock_guard<std::recursive_mutex> guard(device_mutex_);
  uint8_t buf[2];
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to reserve bus", __func__);
      return -1;
    }
    if (i2c_transaction.read(BMI160_REG_TEMPERATURE_LSB, buf, sizeof(buf)) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to read temperature", __func__);
      return -1;
    }
  }
  // Per datasheet: 0x0000 = 23 °C, slope = 1/2^9 K/LSB.
  const int16_t raw = le16(buf);
  temperature_c = 23.0 + raw / 512.0;
  return 0;
}

}  // namespace linux_i2c_devices
