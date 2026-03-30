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
 * @file pca9685_servo_control.cpp
 * @brief ROS 2 node that drives servos on a PCA9685 board via
 *        std_msgs/Float64MultiArray messages.
 *
 * The node subscribes to ~/servo_commands and expects an array of duty-cycle
 * values (0.0-1.0), one per channel.  For standard hobby servos at 50 Hz the
 * useful range is roughly 0.025 (1 ms pulse / full-left) to 0.125 (2.5 ms
 * pulse / full-right).
 *
 * ROS parameters:
 *  - i2c_bus      (int, default 1)    : I2C bus number.
 *  - device_id    (int, default 0x40) : 7-bit I2C address.
 *  - pwm_frequency (double, default 50.0) : PWM frequency in Hz.
 */

#include <memory>
#include <vector>

#include "linux_i2c_devices/pca9685.hpp"
#include "linux_i2c_interface/i2c_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

/**
 * @class Pca9685ServoControl
 * @brief Drives PCA9685 PWM channels from a Float64MultiArray topic.
 */
class Pca9685ServoControl : public rclcpp::Node
{
public:
  Pca9685ServoControl()
  : Node("pca9685_servo_control")
  {
    this->declare_parameter<int>("i2c_bus", 1);
    this->declare_parameter<int>("device_id", linux_i2c_devices::PCA9685_DEFAULT_ADDRESS);
    this->declare_parameter<double>("pwm_frequency", linux_i2c_devices::PCA9685_DEFAULT_FREQ);

    int i2c_bus = this->get_parameter("i2c_bus").as_int();
    int device_id = this->get_parameter("device_id").as_int();
    double pwm_freq = this->get_parameter("pwm_frequency").as_double();

    auto i2c_interface = std::make_shared<linux_i2c_interface::I2cInterface>(
      static_cast<uint8_t>(i2c_bus));

    pca9685_ = std::make_unique<linux_i2c_devices::Pca9685>(
      i2c_interface, static_cast<uint8_t>(device_id));

    if (pca9685_->initialize() < 0)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize PCA9685");
      rclcpp::shutdown();
      return;
    }

    if (pca9685_->set_pwm_frequency(pwm_freq) < 0)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to set PWM frequency");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(
      this->get_logger(), "PCA9685 ready on bus %d at address 0x%02X (%.1f Hz)",
      i2c_bus, device_id, pwm_freq);

    sub_servo_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/servo_commands", 10,
      std::bind(&Pca9685ServoControl::servo_callback, this, std::placeholders::_1));
  }

  ~Pca9685ServoControl()
  {
    if (pca9685_)
    {
      pca9685_->stop();
    }
  }

private:
  void servo_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->data.size() && i < linux_i2c_devices::PCA9685_NUM_CHANNELS; ++i)
    {
      if (pca9685_->set_duty_cycle(static_cast<uint8_t>(i), msg->data[i]) < 0)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to set duty cycle on channel %zu", i);
      }
    }
  }

  std::unique_ptr<linux_i2c_devices::Pca9685> pca9685_;  ///< PCA9685 driver instance.
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_servo_;  ///< Servo command subscription.
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pca9685ServoControl>());
  rclcpp::shutdown();
  return 0;
}
