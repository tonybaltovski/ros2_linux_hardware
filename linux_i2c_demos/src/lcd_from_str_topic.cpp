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

#include <unistd.h>

#include <memory>
#include <string>

#include "linux_i2c_devices/lcm1602.hpp"
#include "linux_i2c_interface/i2c_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class LcdFromStrTopic : public rclcpp::Node
{
public:
  LcdFromStrTopic()
  : Node("lcd_from_str_topic"),
    lcd_(std::make_shared<linux_i2c_interface::I2cInterface>(1), 0x27, 4, 20)
  {
    sub_str_ = this->create_subscription<std_msgs::msg::String>(
      "/lcd_from_str_topic", 100,
      std::bind(&LcdFromStrTopic::str_callback, this, std::placeholders::_1));
    lcd_.initialize();
    lcd_.clear();
  }

  ~LcdFromStrTopic() { lcd_.stop(); }

private:
  void str_callback(const std_msgs::msg::String::SharedPtr str_msg)
  {
    lcd_.clear();
    lcd_.set_cursor(0, 0);
    lcd_.print_msg(str_msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_str_;
  linux_i2c_devices::Lcm1602 lcd_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LcdFromStrTopic>());
  rclcpp::shutdown();
  return 0;
}
