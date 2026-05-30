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

/**
 * @file gpio_system.hpp
 * @brief ros2_control SystemInterface plugin exposing Linux GPIO lines.
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "linux_gpio_interface/gpio_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace linux_i2c_ros2_control
{

/**
 * @class GpioSystem
 * @brief Exposes a set of `<gpio>` lines on a single `/dev/gpiochipN` as
 *        ros2_control command/state interfaces.
 *
 * URDF schema (per `<gpio>`):
 *   - `<param name="line">N</param>`         — line offset on the chip (required).
 *   - `<param name="direction">input|output</param>` — required.
 *   - `<param name="initial_value">0|1</param>` — output only, default 0.
 *   - `<param name="bias">as_is|disabled|pull_up|pull_down</param>` — input
 *      only, default `as_is`.
 *   - `<param name="drive">push_pull|open_drain|open_source</param>` — output
 *      only, default `push_pull`.
 *
 * Hardware-level params (under `<hardware>`):
 *   - `<param name="chip">/dev/gpiochip0</param>` — default `/dev/gpiochip0`.
 *
 * Every `<gpio>` exports one interface per declared `<state_interface>` and
 * `<command_interface>` (typically named `value`) carrying 0.0 or 1.0.  All
 * interfaces on a given gpio read/write the same line value.
 */
class GpioSystem : public hardware_interface::SystemInterface
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

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  enum class Direction
  {
    Input,
    Output,
  };

  struct GpioEntry
  {
    std::string name;
    Direction direction{Direction::Input};
    uint32_t line{0};
    linux_gpio_interface::GpioBias bias{linux_gpio_interface::GpioBias::AsIs};
    linux_gpio_interface::GpioDrive drive{linux_gpio_interface::GpioDrive::PushPull};
    bool initial_value{false};
    double state_value{0.0};
    double command_value{0.0};
    linux_gpio_interface::GpioInterface::Line line_handle;
  };

  std::string chip_path_{"/dev/gpiochip0"};
  std::shared_ptr<linux_gpio_interface::GpioInterface> chip_;
  std::vector<GpioEntry> entries_;
};

}  // namespace linux_i2c_ros2_control
