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
 * @brief ROS 2 node that drives servos / ESCs on a PCA9685 board via
 *        std_msgs/Float64MultiArray messages.
 *
 * Each element of the incoming array is a normalised throttle command in
 * [-1.0, +1.0] for the corresponding PCA9685 channel:
 *   - -1.0 -> minimum pulse width (full reverse / one extreme).
 *   -  0.0 -> midpoint (servo centre / ESC stop).
 *   - +1.0 -> maximum pulse width (full forward / other extreme).
 * Values are clamped, and channels past the array length are left untouched.
 *
 * Parameters are declared via generate_parameter_library; see
 * pca9685_servo_control_parameters.yaml for the schema.
 */

#include <memory>
#include <vector>

#include "linux_i2c_demos/pca9685_servo_control_parameters.hpp"
#include "linux_i2c_devices/pca9685.hpp"
#include "linux_i2c_devices/pca9685_servo.hpp"
#include "linux_i2c_interface/i2c_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class Pca9685ServoControl : public rclcpp::Node
{
public:
  Pca9685ServoControl() : Node("pca9685_servo_control")
  {
    param_listener_ =
      std::make_shared<pca9685_servo_control::ParamListener>(this->get_node_parameters_interface());
    const auto params = param_listener_->get_params();

    auto i2c_interface =
      linux_i2c_interface::I2cInterface::get_shared(static_cast<uint8_t>(params.i2c_bus));

    pca9685_ = std::make_unique<linux_i2c_devices::Pca9685>(
      i2c_interface, static_cast<uint8_t>(params.device_id));

    if (pca9685_->initialize() < 0)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to initialize PCA9685");
      rclcpp::shutdown();
      return;
    }

    if (pca9685_->set_pwm_frequency(params.pwm_frequency) < 0)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to set PWM frequency");
      rclcpp::shutdown();
      return;
    }

    servos_.reserve(linux_i2c_devices::PCA9685_NUM_CHANNELS);
    for (uint8_t ch = 0; ch < linux_i2c_devices::PCA9685_NUM_CHANNELS; ++ch)
    {
      servos_.emplace_back(*pca9685_, ch);
    }

    RCLCPP_INFO(
      this->get_logger(), "PCA9685 ready on bus %ld at address 0x%02lX (%.1f Hz)", params.i2c_bus,
      params.device_id, params.pwm_frequency);

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
    if (param_listener_->is_old(param_listener_->get_params()))
    {
      const auto params = param_listener_->get_params();
      pca9685_->set_pwm_frequency(params.pwm_frequency);
    }
    for (size_t i = 0; i < msg->data.size() && i < servos_.size(); ++i)
    {
      if (servos_[i].set_throttle(msg->data[i]) < 0)
      {
        RCLCPP_WARN(this->get_logger(), "Failed to set throttle on channel %zu", i);
      }
    }
  }

  std::shared_ptr<pca9685_servo_control::ParamListener> param_listener_;
  std::unique_ptr<linux_i2c_devices::Pca9685> pca9685_;
  std::vector<linux_i2c_devices::Pca9685Servo> servos_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_servo_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pca9685ServoControl>());
  rclcpp::shutdown();
  return 0;
}
