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

/**
 * @file bmi160_sensor_hardware.hpp
 * @brief ros2_control SensorInterface for the Bosch BMI160 6-axis IMU.
 */

#pragma once

#include <array>
#include <memory>
#include <string>

#include "hardware_interface/sensor_interface.hpp"

#include "linux_i2c_devices/bmi160.hpp"
#include "linux_i2c_interface/i2c_interface.hpp"

namespace linux_i2c_ros2_control
{

/**
 * @class Bmi160SensorHardware
 * @brief ros2_control hardware interface for the BMI160 IMU.
 *
 * URDF `<hardware>` parameters (all optional except as noted):
 *   - `i2c_bus`         : int, default 1.
 *   - `device_id`       : int, default 0x68 (use 0x69 if SDO tied high).
 *   - `accel_range_g`   : one of {2, 4, 8, 16}, default 2.
 *   - `gyro_range_dps`  : one of {125, 250, 500, 1000, 2000}, default 2000.
 *   - `sensor_name`     : string, the `<sensor name=...>` inside the
 *                         `<ros2_control>` tag. Required to match.
 *
 * Exports the 10 standard IMU state interfaces under `<sensor_name>/`:
 *   orientation.{x,y,z,w} (fixed to identity; chip has no fusion),
 *   angular_velocity.{x,y,z}    (rad/s),
 *   linear_acceleration.{x,y,z} (m/s^2).
 */
class Bmi160SensorHardware : public hardware_interface::SensorInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Configuration (parsed in on_init).
  int i2c_bus_{1};
  uint8_t device_id_{linux_i2c_devices::BMI160_DEFAULT_ADDRESS};
  linux_i2c_devices::Bmi160AccelRange accel_range_{linux_i2c_devices::Bmi160AccelRange::RANGE_2G};
  linux_i2c_devices::Bmi160GyroRange gyro_range_{linux_i2c_devices::Bmi160GyroRange::RANGE_2000_DPS};
  std::string sensor_name_;

  // Runtime.
  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface_;
  std::unique_ptr<linux_i2c_devices::Bmi160> imu_;

  // State storage. Order matches export_state_interfaces below.
  //   [0..3] orientation x,y,z,w
  //   [4..6] angular_velocity x,y,z (rad/s)
  //   [7..9] linear_acceleration x,y,z (m/s^2)
  std::array<double, 10> state_{};
};

}  // namespace linux_i2c_ros2_control
