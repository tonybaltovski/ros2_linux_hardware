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

#include "linux_i2c_ros2_control/bmi160_sensor_hardware.hpp"

#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace linux_i2c_ros2_control
{

namespace
{
constexpr double k_g_to_mps2 = 9.80665;
constexpr double k_deg_to_rad = M_PI / 180.0;

// Index layout of Bmi160SensorHardware::state_.
constexpr std::size_t kOrientationX = 0;
constexpr std::size_t kOrientationY = 1;
constexpr std::size_t kOrientationZ = 2;
constexpr std::size_t kOrientationW = 3;
constexpr std::size_t kAngularVelocityX = 4;
constexpr std::size_t kAngularVelocityY = 5;
constexpr std::size_t kAngularVelocityZ = 6;
constexpr std::size_t kLinearAccelerationX = 7;
constexpr std::size_t kLinearAccelerationY = 8;
constexpr std::size_t kLinearAccelerationZ = 9;

linux_i2c_devices::Bmi160AccelRange accel_range_from_g(int range_g)
{
  switch (range_g)
  {
    case 4:
      return linux_i2c_devices::Bmi160AccelRange::RANGE_4G;
    case 8:
      return linux_i2c_devices::Bmi160AccelRange::RANGE_8G;
    case 16:
      return linux_i2c_devices::Bmi160AccelRange::RANGE_16G;
    default:
      return linux_i2c_devices::Bmi160AccelRange::RANGE_2G;
  }
}

linux_i2c_devices::Bmi160GyroRange gyro_range_from_dps(int range_dps)
{
  switch (range_dps)
  {
    case 125:
      return linux_i2c_devices::Bmi160GyroRange::RANGE_125_DPS;
    case 250:
      return linux_i2c_devices::Bmi160GyroRange::RANGE_250_DPS;
    case 500:
      return linux_i2c_devices::Bmi160GyroRange::RANGE_500_DPS;
    case 1000:
      return linux_i2c_devices::Bmi160GyroRange::RANGE_1000_DPS;
    default:
      return linux_i2c_devices::Bmi160GyroRange::RANGE_2000_DPS;
  }
}

int get_int_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key, int default_value)
{
  auto it = params.find(key);
  if (it == params.end())
  {
    return default_value;
  }
  // Support 0x.. prefix for device_id.
  return static_cast<int>(std::stol(it->second, nullptr, 0));
}
}  // namespace

hardware_interface::CallbackReturn Bmi160SensorHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.sensors.size() != 1)
  {
    RCLCPP_ERROR(
      get_logger(),
      "Bmi160SensorHardware expects exactly one <sensor> in the URDF (got %zu).",
      info_.sensors.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  sensor_name_ = info_.sensors[0].name;

  try
  {
    i2c_bus_ = get_int_param(info_.hardware_parameters, "i2c_bus", 1);
    const int dev = get_int_param(
      info_.hardware_parameters, "device_id",
      static_cast<int>(linux_i2c_devices::BMI160_DEFAULT_ADDRESS));
    if (dev < 0 || dev > 0x7F)
    {
      RCLCPP_ERROR(get_logger(), "device_id %d out of 7-bit range", dev);
      return hardware_interface::CallbackReturn::ERROR;
    }
    device_id_ = static_cast<uint8_t>(dev);
    accel_range_ =
      accel_range_from_g(get_int_param(info_.hardware_parameters, "accel_range_g", 2));
    gyro_range_ =
      gyro_range_from_dps(get_int_param(info_.hardware_parameters, "gyro_range_dps", 2000));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to parse hardware parameters: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // No orientation fusion on chip -> publish identity quaternion every cycle.
  state_.fill(std::numeric_limits<double>::quiet_NaN());
  state_[kOrientationX] = 0.0;
  state_[kOrientationY] = 0.0;
  state_[kOrientationZ] = 0.0;
  state_[kOrientationW] = 1.0;

  RCLCPP_INFO(
    get_logger(),
    "Bmi160SensorHardware configured: sensor='%s' bus=%d addr=0x%02X",
    sensor_name_.c_str(), i2c_bus_, device_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Bmi160SensorHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    i2c_interface_ = linux_i2c_interface::I2cInterface::get_shared(static_cast<uint8_t>(i2c_bus_));
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to open I2C bus %d: %s", i2c_bus_, e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  imu_ = std::make_unique<linux_i2c_devices::Bmi160>(
    i2c_interface_, device_id_, accel_range_, gyro_range_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Bmi160SensorHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!imu_)
  {
    RCLCPP_ERROR(get_logger(), "BMI160 driver not constructed; was on_configure called?");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (imu_->initialize() < 0)
  {
    RCLCPP_ERROR(get_logger(), "Failed to initialize BMI160");
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Bmi160SensorHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (imu_ && imu_->stop() < 0)
  {
    RCLCPP_WARN(get_logger(), "Failed to stop BMI160");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Bmi160SensorHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  imu_.reset();
  i2c_interface_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Bmi160SensorHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  ifaces.reserve(10);

  const auto add = [&](const std::string & iface_name, std::size_t idx)
    {
      ifaces.emplace_back(sensor_name_, iface_name, &state_[idx]);
    };

  add("orientation.x", kOrientationX);
  add("orientation.y", kOrientationY);
  add("orientation.z", kOrientationZ);
  add("orientation.w", kOrientationW);
  add("angular_velocity.x", kAngularVelocityX);
  add("angular_velocity.y", kAngularVelocityY);
  add("angular_velocity.z", kAngularVelocityZ);
  add("linear_acceleration.x", kLinearAccelerationX);
  add("linear_acceleration.y", kLinearAccelerationY);
  add("linear_acceleration.z", kLinearAccelerationZ);

  return ifaces;
}

hardware_interface::return_type Bmi160SensorHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!imu_)
  {
    return hardware_interface::return_type::ERROR;
  }

  double ax_g = 0.0;
  double ay_g = 0.0;
  double az_g = 0.0;
  double gx_dps = 0.0;
  double gy_dps = 0.0;
  double gz_dps = 0.0;
  if (imu_->read_imu(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps) < 0)
  {
    // Don't propagate ERROR upward -- a transient bus glitch shouldn't deactivate
    // the controller. Throttled warn instead and reuse the previous sample.
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "BMI160 read failed; reusing previous sample");
    return hardware_interface::return_type::OK;
  }

  state_[kLinearAccelerationX] = ax_g * k_g_to_mps2;
  state_[kLinearAccelerationY] = ay_g * k_g_to_mps2;
  state_[kLinearAccelerationZ] = az_g * k_g_to_mps2;
  state_[kAngularVelocityX] = gx_dps * k_deg_to_rad;
  state_[kAngularVelocityY] = gy_dps * k_deg_to_rad;
  state_[kAngularVelocityZ] = gz_dps * k_deg_to_rad;

  return hardware_interface::return_type::OK;
}

}  // namespace linux_i2c_ros2_control

PLUGINLIB_EXPORT_CLASS(
  linux_i2c_ros2_control::Bmi160SensorHardware,
  hardware_interface::SensorInterface)
