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
 * @file screen_rosout_logger.cpp
 * @brief ROS 2 node that subscribes to /rosout and displays log messages on
 *        an I2C screen (LCM1602 LCD or SSD1306 OLED), filtered by severity.
 *
 * ROS parameters:
 *  - display_type      (string, default "lcm1602") : "lcm1602" or "ssd1306".
 *  - i2c_bus           (int, default 1)            : I2C bus number.
 *  - device_id         (int, default depends on display_type) : 7-bit I2C address.
 *  - min_logger_level  (int, default 20 = INFO)    : Minimum rcl_interfaces/Log level.
 */

#include <algorithm>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "linux_i2c_devices/lcm1602.hpp"
#include "linux_i2c_devices/screen.hpp"
#include "linux_i2c_devices/ssd1306.hpp"
#include "linux_i2c_interface/i2c_interface.hpp"

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @class ScreenRosoutLogger
 * @brief Displays /rosout log messages on an I2C screen, filtered by severity.
 *
 * New messages appear on the bottom row; older messages scroll upward.
 */
class ScreenRosoutLogger : public rclcpp::Node
{
public:
  ScreenRosoutLogger()
  : Node("screen_rosout_logger")
  {
    this->declare_parameter<std::string>("display_type", "lcm1602");
    this->declare_parameter<int>("i2c_bus", 1);
    this->declare_parameter<int>("min_logger_level", rcl_interfaces::msg::Log::INFO);

    std::string display_type = this->get_parameter("display_type").as_string();
    int i2c_bus = this->get_parameter("i2c_bus").as_int();
    min_logger_level_ =
      static_cast<uint8_t>(this->get_parameter("min_logger_level").as_int());

    auto i2c_interface =
      std::make_shared<linux_i2c_interface::I2cInterface>(static_cast<uint8_t>(i2c_bus));

    if (display_type == "ssd1306")
    {
      this->declare_parameter<int>(
        "device_id", linux_i2c_devices::SSD1306_DEFAULT_ADDRESS);
      int device_id = this->get_parameter("device_id").as_int();

      screen_ = std::make_unique<linux_i2c_devices::Ssd1306>(
        i2c_interface, static_cast<uint8_t>(device_id));

      RCLCPP_INFO(
        this->get_logger(), "SSD1306 OLED on bus %d at address 0x%02X",
        i2c_bus, device_id);
    }
    else if (display_type == "lcm1602")
    {
      this->declare_parameter<int>("device_id", 0x27);
      this->declare_parameter<int>("rows", 4);
      this->declare_parameter<int>("columns", 20);
      int device_id = this->get_parameter("device_id").as_int();
      int rows = this->get_parameter("rows").as_int();
      int columns = this->get_parameter("columns").as_int();

      screen_ = std::make_unique<linux_i2c_devices::Lcm1602>(
        i2c_interface, static_cast<uint8_t>(device_id),
        static_cast<uint8_t>(rows), static_cast<uint8_t>(columns));

      RCLCPP_INFO(
        this->get_logger(), "LCM1602 LCD on bus %d at address 0x%02X (%dx%d)",
        i2c_bus, device_id, columns, rows);
    }
    else
    {
      RCLCPP_FATAL(
        this->get_logger(),
        "Unknown display_type '%s'. Supported: lcm1602, ssd1306",
        display_type.c_str());
      rclcpp::shutdown();
      return;
    }

    if (screen_->initialize() < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize screen");
    }
    if (screen_->clear() < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to clear screen");
    }

    uint8_t rows = screen_->get_rows();
    uint8_t cols = screen_->get_columns();
    line_buffer_.assign(rows, std::string(cols, ' '));

    sub_log_ = this->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", 100,
      std::bind(&ScreenRosoutLogger::log_callback, this, std::placeholders::_1));
  }

  ~ScreenRosoutLogger()
  {
    if (screen_)
    {
      if (screen_->stop() < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop screen");
      }
    }
  }

private:
  static std::string log_to_string(const rcl_interfaces::msg::Log::SharedPtr & log_msg)
  {
    std::stringstream ss;
    switch (log_msg->level)
    {
      case rcl_interfaces::msg::Log::DEBUG:
        ss << "[DEBUG] ";
        break;
      case rcl_interfaces::msg::Log::INFO:
        ss << "[INFO] ";
        break;
      case rcl_interfaces::msg::Log::WARN:
        ss << "[WARN] ";
        break;
      case rcl_interfaces::msg::Log::ERROR:
        ss << "[ERROR] ";
        break;
      case rcl_interfaces::msg::Log::FATAL:
        ss << "[FATAL] ";
        break;
      default:
        ss << log_msg->level << " ";
    }
    ss << "[" << log_msg->name << "] " << log_msg->msg;
    return ss.str();
  }

  /// Pad or truncate a string to exactly @p cols characters.
  std::string fit_to_row(const std::string & text) const
  {
    uint8_t cols = screen_->get_columns();
    std::string row = text.substr(0, cols);
    row.resize(cols, ' ');
    return row;
  }

  void log_callback(const rcl_interfaces::msg::Log::SharedPtr log_msg)
  {
    if (log_msg->level < min_logger_level_)
    {
      return;
    }

    uint8_t rows = screen_->get_rows();

    // Scroll the buffer up by one row and place the new message on the last row.
    std::rotate(line_buffer_.begin(), line_buffer_.begin() + 1, line_buffer_.end());
    line_buffer_[rows - 1] = fit_to_row(log_to_string(log_msg));

    // Redraw every row.
    if (screen_->clear() < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to clear screen");
    }
    for (uint8_t r = 0; r < rows; ++r)
    {
      if (screen_->set_cursor(r, 0) < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to set cursor at row %d", r);
      }
      if (screen_->print_msg(line_buffer_[r]) < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to print message");
      }
    }
  }

  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr sub_log_;  ///< /rosout subscription.
  std::unique_ptr<linux_i2c_devices::Screen> screen_;  ///< Active screen device.
  uint8_t min_logger_level_;  ///< Minimum log severity to display.
  std::vector<std::string> line_buffer_;  ///< Circular line buffer for display rows.
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScreenRosoutLogger>());
  rclcpp::shutdown();
  return 0;
}
