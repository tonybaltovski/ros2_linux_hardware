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
 * Parameters are declared via generate_parameter_library; see
 * screen_from_str_topic_parameters.yaml for the schema.
 */

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "linux_i2c_demos/screen_from_str_topic_parameters.hpp"
#include "linux_i2c_devices/lcm1602.hpp"
#include "linux_i2c_devices/screen.hpp"
#include "linux_i2c_devices/ssd1306.hpp"
#include "linux_i2c_interface/i2c_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ScreenFromStrTopic : public rclcpp::Node
{
public:
  ScreenFromStrTopic() : Node("screen_from_str_topic")
  {
    param_listener_ =
      std::make_shared<screen_from_str_topic::ParamListener>(this->get_node_parameters_interface());
    const auto params = param_listener_->get_params();

    auto i2c_interface =
      std::make_shared<linux_i2c_interface::I2cInterface>(static_cast<uint8_t>(params.i2c_bus));

    if (params.display_type == "ssd1306")
    {
      screen_ = std::make_unique<linux_i2c_devices::Ssd1306>(
        i2c_interface, static_cast<uint8_t>(params.device_id));
      RCLCPP_INFO(
        this->get_logger(), "SSD1306 OLED on bus %ld at address 0x%02lX", params.i2c_bus,
        params.device_id);
    }
    else
    {
      screen_ = std::make_unique<linux_i2c_devices::Lcm1602>(
        i2c_interface, static_cast<uint8_t>(params.device_id), static_cast<uint8_t>(params.rows),
        static_cast<uint8_t>(params.columns));
      RCLCPP_INFO(
        this->get_logger(), "LCM1602 LCD on bus %ld at address 0x%02lX (%ldx%ld)", params.i2c_bus,
        params.device_id, params.columns, params.rows);
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
      std::chrono::duration<double>(params.scroll_period),
      std::bind(&ScreenFromStrTopic::scroll_callback, this));
    scroll_timer_->cancel();

    sub_str_ = this->create_subscription<std_msgs::msg::String>(
      "~/input", 100, std::bind(&ScreenFromStrTopic::str_callback, this, std::placeholders::_1));
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

  std::shared_ptr<screen_from_str_topic::ParamListener> param_listener_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_str_;
  rclcpp::TimerBase::SharedPtr scroll_timer_;
  std::unique_ptr<linux_i2c_devices::Screen> screen_;
  std::vector<std::string> lines_;
  size_t scroll_offset_{0};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScreenFromStrTopic>());
  rclcpp::shutdown();
  return 0;
}
