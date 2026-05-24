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
 * @file bmi160_publisher.cpp
 * @brief ROS 2 node that samples the BMI160 6-axis IMU over I2C and publishes:
 *          - ~/imu          sensor_msgs/Imu (linear accel m/s^2, angular rate rad/s)
 *          - ~/temperature  sensor_msgs/Temperature (degrees Celsius)
 *
 * Parameters are declared via generate_parameter_library; see
 * bmi160_publisher_parameters.yaml for the schema.
 */

#include <chrono>
#include <cmath>
#include <memory>

#include "linux_i2c_demos/bmi160_publisher_parameters.hpp"
#include "linux_i2c_devices/bmi160.hpp"
#include "linux_i2c_interface/i2c_interface.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace
{
linux_i2c_devices::Bmi160AccelRange accel_range_from_g(int range_g)
{
  switch (range_g)
  {
    case 4:
      return linux_i2c_devices::Bmi160AccelRange::RANGE_4G;
    case 8:
      return linux_i2c_devices::Bmi160AccelRange::RANGE_8G;
    case 16:
      return linux_i2c_devices::Bmi160AccelRange::RANGE_16G;
    default:
      return linux_i2c_devices::Bmi160AccelRange::RANGE_2G;
  }
}

linux_i2c_devices::Bmi160GyroRange gyro_range_from_dps(int range_dps)
{
  switch (range_dps)
  {
    case 125:
      return linux_i2c_devices::Bmi160GyroRange::RANGE_125_DPS;
    case 250:
      return linux_i2c_devices::Bmi160GyroRange::RANGE_250_DPS;
    case 500:
      return linux_i2c_devices::Bmi160GyroRange::RANGE_500_DPS;
    case 1000:
      return linux_i2c_devices::Bmi160GyroRange::RANGE_1000_DPS;
    default:
      return linux_i2c_devices::Bmi160GyroRange::RANGE_2000_DPS;
  }
}
}  // namespace

class Bmi160Publisher : public rclcpp::Node
{
public:
  Bmi160Publisher() : Node("bmi160_publisher")
  {
    param_listener_ =
      std::make_shared<bmi160_publisher::ParamListener>(this->get_node_parameters_interface());
    const auto params = param_listener_->get_params();

    frame_id_ = params.frame_id;

    auto i2c_interface = linux_i2c_interface::I2cInterface::get_shared(
      static_cast<uint8_t>(params.i2c_bus));

    sensor_ = std::make_unique<linux_i2c_devices::Bmi160>(
      i2c_interface, static_cast<uint8_t>(params.device_id),
      accel_range_from_g(static_cast<int>(params.accel_range_g)),
      gyro_range_from_dps(static_cast<int>(params.gyro_range_dps)));

    RCLCPP_INFO(
      this->get_logger(),
      "BMI160 on bus %ld at address 0x%02lX (frame_id='%s', %.2f Hz, ±%ld g, ±%ld dps)",
      params.i2c_bus, params.device_id, frame_id_.c_str(), params.publish_rate,
      params.accel_range_g, params.gyro_range_dps);

    if (sensor_->initialize() < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize BMI160");
    }

    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu", 10);
    pub_temp_ = this->create_publisher<sensor_msgs::msg::Temperature>("~/temperature", 10);

    const auto period = std::chrono::duration<double>(1.0 / params.publish_rate);
    sample_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Bmi160Publisher::sample_callback, this));
  }

  ~Bmi160Publisher()
  {
    if (sensor_)
    {
      if (sensor_->stop() < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop BMI160");
      }
    }
  }

private:
  void sample_callback()
  {
    const auto stamp = this->now();

    double ax_g = 0.0;
    double ay_g = 0.0;
    double az_g = 0.0;
    double gx_dps = 0.0;
    double gy_dps = 0.0;
    double gz_dps = 0.0;
    if (sensor_->read_imu(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps) == 0)
    {
      sensor_msgs::msg::Imu msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = frame_id_;
      // sensor_msgs/Imu: linear_acceleration in m/s^2, angular_velocity in rad/s.
      constexpr double k_g_to_mps2 = 9.80665;
      constexpr double k_deg_to_rad = M_PI / 180.0;
      msg.linear_acceleration.x = ax_g * k_g_to_mps2;
      msg.linear_acceleration.y = ay_g * k_g_to_mps2;
      msg.linear_acceleration.z = az_g * k_g_to_mps2;
      msg.angular_velocity.x = gx_dps * k_deg_to_rad;
      msg.angular_velocity.y = gy_dps * k_deg_to_rad;
      msg.angular_velocity.z = gz_dps * k_deg_to_rad;
      // No on-chip orientation fusion; mark orientation invalid per msg convention.
      msg.orientation_covariance[0] = -1.0;
      msg.angular_velocity_covariance[0] = -1.0;
      msg.linear_acceleration_covariance[0] = -1.0;
      pub_imu_->publish(msg);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Failed to read IMU");
    }

    double temperature_c = 0.0;
    if (sensor_->read_temperature_c(temperature_c) == 0)
    {
      sensor_msgs::msg::Temperature msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = frame_id_;
      msg.temperature = temperature_c;
      msg.variance = 0.0;  // Unknown.
      pub_temp_->publish(msg);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000, "Failed to read temperature");
    }
  }

  std::shared_ptr<bmi160_publisher::ParamListener> param_listener_;
  std::unique_ptr<linux_i2c_devices::Bmi160> sensor_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_;
  rclcpp::TimerBase::SharedPtr sample_timer_;
  std::string frame_id_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Bmi160Publisher>());
  rclcpp::shutdown();
  return 0;
}
