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
 * @file hmc6343_publisher.cpp
 * @brief ROS 2 node that samples the HMC6343 tilt-compensated compass over
 *        I2C and publishes:
 *          - ~/magnetic_field  sensor_msgs/MagneticField  (Tesla, SI)
 *          - ~/heading         std_msgs/Float64           (degrees, [0, 360))
 *
 * Parameters are declared via generate_parameter_library; see
 * hmc6343_publisher_parameters.yaml for the schema.
 */

#include <chrono>
#include <memory>

#include "linux_i2c_demos/hmc6343_publisher_parameters.hpp"
#include "linux_i2c_devices/hmc6343.hpp"
#include "linux_i2c_interface/i2c_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <std_msgs/msg/float64.hpp>

class Hmc6343Publisher : public rclcpp::Node
{
public:
  Hmc6343Publisher() : Node("hmc6343_publisher")
  {
    param_listener_ =
      std::make_shared<hmc6343_publisher::ParamListener>(this->get_node_parameters_interface());
    const auto params = param_listener_->get_params();

    frame_id_ = params.frame_id;

    auto i2c_interface =
      linux_i2c_interface::I2cInterface::get_shared(static_cast<uint8_t>(params.i2c_bus));

    sensor_ = std::make_unique<linux_i2c_devices::Hmc6343>(
      i2c_interface, static_cast<uint8_t>(params.device_id));

    RCLCPP_INFO(
      this->get_logger(), "HMC6343 on bus %ld at address 0x%02lX (frame_id='%s', %.2f Hz)",
      params.i2c_bus, params.device_id, frame_id_.c_str(), params.publish_rate);

    if (sensor_->initialize() < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize HMC6343");
    }

    pub_field_ = this->create_publisher<sensor_msgs::msg::MagneticField>("~/magnetic_field", 10);
    pub_heading_ = this->create_publisher<std_msgs::msg::Float64>("~/heading", 10);

    const auto period = std::chrono::duration<double>(1.0 / params.publish_rate);
    sample_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Hmc6343Publisher::sample_callback, this));
  }

  ~Hmc6343Publisher()
  {
    if (sensor_)
    {
      if (sensor_->stop() < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop HMC6343");
      }
    }
  }

private:
  void sample_callback()
  {
    const auto stamp = this->now();

    double x_ut = 0.0;
    double y_ut = 0.0;
    double z_ut = 0.0;
    if (sensor_->read_field_ut(x_ut, y_ut, z_ut) == 0)
    {
      sensor_msgs::msg::MagneticField msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = frame_id_;
      // sensor_msgs/MagneticField is in Tesla; convert from µT.
      constexpr double k_ut_to_t = 1.0e-6;
      msg.magnetic_field.x = x_ut * k_ut_to_t;
      msg.magnetic_field.y = y_ut * k_ut_to_t;
      msg.magnetic_field.z = z_ut * k_ut_to_t;
      // Unknown covariance: per the msg convention, set the first element to -1.
      msg.magnetic_field_covariance[0] = -1.0;
      pub_field_->publish(msg);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Failed to read magnetic field");
    }

    double heading_deg = 0.0;
    if (sensor_->read_heading_deg(heading_deg) == 0)
    {
      std_msgs::msg::Float64 msg;
      msg.data = heading_deg;
      pub_heading_->publish(msg);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Failed to read heading");
    }
  }

  std::shared_ptr<hmc6343_publisher::ParamListener> param_listener_;
  std::unique_ptr<linux_i2c_devices::Hmc6343> sensor_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_field_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_heading_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  std::string frame_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Hmc6343Publisher>());
  rclcpp::shutdown();
  return 0;
}
