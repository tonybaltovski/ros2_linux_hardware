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
 * @file screen_from_str_topic.cpp
 * @brief ROS 2 node that subscribes to a std_msgs/String topic and displays
 *        the message on an I2C screen (LCM1602 LCD or SSD1306 OLED).
 *
 * If the message is longer than the screen capacity the text scrolls
 * upward automatically at a configurable interval.
 *
 * ROS parameters:
 *  - display_type  (string, default "lcm1602") : "lcm1602" or "ssd1306".
 *  - i2c_bus       (int, default 1)            : I2C bus number.
 *  - device_id     (int, default depends on display_type) : 7-bit I2C address.
 *  - scroll_period (double, default 1.0)       : Seconds between scroll steps.
 */

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "linux_i2c_devices/lcm1602.hpp"
#include "linux_i2c_devices/screen.hpp"
#include "linux_i2c_devices/ssd1306.hpp"
#include "linux_i2c_interface/i2c_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @class ScreenFromStrTopic
 * @brief Displays std_msgs/String messages on an I2C screen.
 *
 * The display_type parameter selects between an LCM1602 character LCD and an
 * SSD1306 128×64 OLED.  Messages that exceed the visible area scroll upward
 * at scroll_period intervals.
 */
class ScreenFromStrTopic : public rclcpp::Node
{
public:
  ScreenFromStrTopic()
  : Node("screen_from_str_topic")
  {
    this->declare_parameter<std::string>("display_type", "lcm1602");
    this->declare_parameter<int>("i2c_bus", 1);
    this->declare_parameter<double>("scroll_period", 1.0);

    std::string display_type = this->get_parameter("display_type").as_string();
    int i2c_bus = this->get_parameter("i2c_bus").as_int();
    double scroll_period = this->get_parameter("scroll_period").as_double();

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

    scroll_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(scroll_period),
      std::bind(&ScreenFromStrTopic::scroll_callback, this));
    scroll_timer_->cancel();

    sub_str_ = this->create_subscription<std_msgs::msg::String>(
      "~/input", 100,
      std::bind(&ScreenFromStrTopic::str_callback, this, std::placeholders::_1));
  }

  ~ScreenFromStrTopic()
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
  /// Break a message into display-width lines, wrapping on column boundaries.
  std::vector<std::string> wrap_text(const std::string & text, uint8_t cols)
  {
    std::vector<std::string> lines;
    for (size_t i = 0; i < text.size(); i += cols)
    {
      std::string line = text.substr(i, cols);
      line.resize(cols, ' ');
      lines.push_back(line);
    }
    if (lines.empty())
    {
      lines.push_back(std::string(cols, ' '));
    }
    return lines;
  }

  /// Redraw the visible window starting at scroll_offset_.
  void redraw()
  {
    uint8_t rows = screen_->get_rows();
    if (screen_->clear() < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to clear screen");
    }
    for (uint8_t r = 0; r < rows && (scroll_offset_ + r) < lines_.size(); ++r)
    {
      if (screen_->set_cursor(r, 0) < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to set cursor at row %d", r);
      }
      if (screen_->print_msg(lines_[scroll_offset_ + r]) < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to print message");
      }
    }
  }

  void str_callback(const std_msgs::msg::String::SharedPtr str_msg)
  {
    lines_ = wrap_text(str_msg->data, screen_->get_columns());
    scroll_offset_ = 0;
    redraw();

    if (lines_.size() > screen_->get_rows())
    {
      scroll_timer_->reset();
    }
    else
    {
      scroll_timer_->cancel();
    }
  }

  void scroll_callback()
  {
    if (lines_.empty())
    {
      return;
    }

    uint8_t rows = screen_->get_rows();
    size_t max_offset = lines_.size() - rows;

    ++scroll_offset_;
    if (scroll_offset_ > max_offset)
    {
      scroll_offset_ = 0;
    }

    redraw();
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_str_;  ///< String subscription.
  rclcpp::TimerBase::SharedPtr scroll_timer_;  ///< Periodic scroll timer.
  std::unique_ptr<linux_i2c_devices::Screen> screen_;  ///< Active screen device.
  std::vector<std::string> lines_;  ///< Wrapped text lines from the last message.
  size_t scroll_offset_{0};  ///< Index of the first visible line.
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScreenFromStrTopic>());
  rclcpp::shutdown();
  return 0;
}
