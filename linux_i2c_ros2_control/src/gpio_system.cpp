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

#include "linux_i2c_ros2_control/gpio_system.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace linux_i2c_ros2_control
{
namespace
{

constexpr const char * kLogger = "linux_i2c_ros2_control.GpioSystem";

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  return s;
}

bool parse_bool(const std::string & s)
{
  const std::string l = lower(s);
  return l == "1" || l == "true" || l == "high" || l == "on";
}

linux_gpio_interface::GpioBias parse_bias(const std::string & s)
{
  const std::string l = lower(s);
  if (l == "disabled" || l == "none") return linux_gpio_interface::GpioBias::Disabled;
  if (l == "pull_up" || l == "pullup" || l == "up") return linux_gpio_interface::GpioBias::PullUp;
  if (l == "pull_down" || l == "pulldown" || l == "down")
    return linux_gpio_interface::GpioBias::PullDown;
  return linux_gpio_interface::GpioBias::AsIs;
}

linux_gpio_interface::GpioDrive parse_drive(const std::string & s)
{
  const std::string l = lower(s);
  if (l == "open_drain" || l == "opendrain") return linux_gpio_interface::GpioDrive::OpenDrain;
  if (l == "open_source" || l == "opensource") return linux_gpio_interface::GpioDrive::OpenSource;
  return linux_gpio_interface::GpioDrive::PushPull;
}

std::string param_or(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & fallback)
{
  auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}

}  // namespace

hardware_interface::CallbackReturn GpioSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto it = info_.hardware_parameters.find("chip");
  if (it != info_.hardware_parameters.end() && !it->second.empty())
  {
    chip_path_ = it->second;
  }

  entries_.clear();
  entries_.reserve(info_.gpios.size());

  for (const auto & gpio : info_.gpios)
  {
    GpioEntry entry;
    entry.name = gpio.name;

    const auto direction_str = lower(param_or(gpio.parameters, "direction", ""));
    if (direction_str == "output" || direction_str == "out")
    {
      entry.direction = Direction::Output;
    }
    else if (direction_str == "input" || direction_str == "in")
    {
      entry.direction = Direction::Input;
    }
    else
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLogger),
        "GPIO '%s': required <param name=\"direction\"> missing or invalid ('%s')",
        gpio.name.c_str(), direction_str.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const auto line_it = gpio.parameters.find("line");
    if (line_it == gpio.parameters.end())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLogger), "GPIO '%s': required <param name=\"line\"> missing",
        gpio.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    try
    {
      entry.line = static_cast<uint32_t>(std::stoul(line_it->second));
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLogger), "GPIO '%s': invalid line '%s': %s", gpio.name.c_str(),
        line_it->second.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (entry.direction == Direction::Output)
    {
      entry.initial_value = parse_bool(param_or(gpio.parameters, "initial_value", "0"));
      entry.drive = parse_drive(param_or(gpio.parameters, "drive", "push_pull"));
      entry.state_value = entry.initial_value ? 1.0 : 0.0;
      entry.command_value = entry.state_value;
    }
    else
    {
      entry.bias = parse_bias(param_or(gpio.parameters, "bias", "as_is"));
    }

    entries_.push_back(std::move(entry));
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger), "Initialised %zu GPIO line(s) on %s", entries_.size(),
    chip_path_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GpioSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try
  {
    chip_ = linux_gpio_interface::GpioInterface::get_shared(chip_path_, /*eager_open=*/true);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogger), "Failed to open %s: %s", chip_path_.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GpioSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!chip_)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (auto & entry : entries_)
  {
    if (entry.direction == Direction::Output)
    {
      entry.line_handle = chip_->request_output(entry.line, entry.initial_value, entry.drive);
    }
    else
    {
      entry.line_handle = chip_->request_input(entry.line, entry.bias);
    }
    if (!entry.line_handle.ok())
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLogger), "Failed to request GPIO '%s' (line %u on %s): %s",
        entry.name.c_str(), entry.line, chip_path_.c_str(), std::strerror(errno));
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GpioSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // Releasing the Line handles releases the kernel-side line request.
  for (auto & entry : entries_)
  {
    entry.line_handle = linux_gpio_interface::GpioInterface::Line();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GpioSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> ifaces;
  for (size_t i = 0; i < entries_.size(); ++i)
  {
    auto & entry = entries_[i];
    const auto & info = info_.gpios[i];
    // Export one StateInterface per <state_interface> declared in the URDF.
    // Every interface for this gpio reads the same line value.
    for (const auto & state : info.state_interfaces)
    {
      ifaces.emplace_back(entry.name, state.name, &entry.state_value);
    }
  }
  return ifaces;
}

std::vector<hardware_interface::CommandInterface> GpioSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ifaces;
  for (size_t i = 0; i < entries_.size(); ++i)
  {
    auto & entry = entries_[i];
    if (entry.direction != Direction::Output)
    {
      continue;
    }
    const auto & info = info_.gpios[i];
    for (const auto & cmd : info.command_interfaces)
    {
      ifaces.emplace_back(entry.name, cmd.name, &entry.command_value);
    }
  }
  return ifaces;
}

hardware_interface::return_type GpioSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & entry : entries_)
  {
    if (!entry.line_handle.ok())
    {
      continue;
    }
    if (entry.direction == Direction::Input)
    {
      const int v = entry.line_handle.get_value();
      if (v < 0)
      {
        return hardware_interface::return_type::ERROR;
      }
      entry.state_value = v ? 1.0 : 0.0;
    }
    // Output state mirrors the last successfully written command (set in write()).
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GpioSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & entry : entries_)
  {
    if (entry.direction != Direction::Output || !entry.line_handle.ok())
    {
      continue;
    }
    const bool desired = entry.command_value >= 0.5;
    if (entry.line_handle.set_value(desired) < 0)
    {
      return hardware_interface::return_type::ERROR;
    }
    entry.state_value = desired ? 1.0 : 0.0;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace linux_i2c_ros2_control

PLUGINLIB_EXPORT_CLASS(linux_i2c_ros2_control::GpioSystem, hardware_interface::SystemInterface)
