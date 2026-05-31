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
 * @file iio_imu_publisher.cpp
 * @brief ROS 2 node that reads a 6-axis IMU via the Linux IIO subsystem and publishes:
 *          - ~/imu          sensor_msgs/Imu (linear accel m/s^2, angular rate rad/s)
 *          - ~/temperature  sensor_msgs/Temperature (degrees Celsius, when available)
 *
 * Works with any IIO IMU that exposes the standard in_accel_{x,y,z} and
 * in_anglvel_{x,y,z} channels (BMI160, BMI270, MPU6050, LSM6DSx, ICM426xx, ...).
 *
 * Two read modes (see iio_imu_publisher_parameters.yaml):
 *   - Polled sysfs reads on a ROS timer.  Works without any kernel trigger
 *     configuration; usable on any IIO device.
 *   - Buffered reads from /dev/iio:deviceN with poll().  Requires a configured
 *     hardware trigger (e.g. data-ready INT1 wired up in the device tree).
 *     The buffered path assumes the common scan layout of six little-endian
 *     int16 channels (gyro X/Y/Z then accel X/Y/Z) plus an 8-byte timestamp,
 *     which matches BMI160/BMI270/MPU6050/LSM6DSx.  Devices with a different
 *     layout would need per-device demuxing.
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "linux_iio_devices/iio_imu_publisher_parameters.hpp"
#include "linux_iio_devices/iio_device.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace
{
// Common 6-axis IIO scan layout used by BMI160/BMI270/MPU6050/LSM6DSx:
//   bytes 0..1   anglvel_x (le:s16)
//   bytes 2..3   anglvel_y
//   bytes 4..5   anglvel_z
//   bytes 6..7   accel_x
//   bytes 8..9   accel_y
//   bytes 10..11 accel_z
//   bytes 12..15 padding to 8-byte alignment
//   bytes 16..23 timestamp (s64 ns)
constexpr std::size_t COMMON_RECORD_SIZE = 24;

int16_t le16(const uint8_t * p)
{
  return static_cast<int16_t>(p[0] | (static_cast<uint16_t>(p[1]) << 8));
}
}  // namespace

class IioImuPublisher : public rclcpp::Node
{
public:
  IioImuPublisher() : Node("iio_imu_publisher")
  {
    param_listener_ =
      std::make_shared<iio_imu_publisher::ParamListener>(this->get_node_parameters_interface());
    const auto params = param_listener_->get_params();

    frame_id_ = params.frame_id;

    if (!params.device_path.empty())
    {
      device_path_ = params.device_path;
    }
    else
    {
      auto resolved = linux_iio_devices::resolve_by_name(params.iio_name);
      if (!resolved)
      {
        RCLCPP_FATAL(
          this->get_logger(), "No IIO device with name='%s' found under /sys/bus/iio/devices",
          params.iio_name.c_str());
        rclcpp::shutdown();
        return;
      }
      device_path_ = *resolved;
    }

    // Cache scales once.  IIO contract: physical = (raw + offset) * scale.
    accel_scale_ = linux_iio_devices::read_double(device_path_ + "/in_accel_scale").value_or(0.0);
    gyro_scale_ = linux_iio_devices::read_double(device_path_ + "/in_anglvel_scale").value_or(0.0);
    if (accel_scale_ == 0.0 || gyro_scale_ == 0.0)
    {
      RCLCPP_FATAL(
        this->get_logger(),
        "Failed to read in_accel_scale / in_anglvel_scale from %s; is this an IIO IMU?",
        device_path_.c_str());
      rclcpp::shutdown();
      return;
    }

    temp_scale_ = linux_iio_devices::read_double(device_path_ + "/in_temp_scale").value_or(0.0);
    temp_offset_ = linux_iio_devices::read_double(device_path_ + "/in_temp_offset").value_or(0.0);
    publish_temperature_ = params.publish_temperature && temp_scale_ != 0.0;

    if (params.sampling_frequency_hz > 0.0)
    {
      const std::string s = std::to_string(params.sampling_frequency_hz);
      linux_iio_devices::write_string(device_path_ + "/in_accel_sampling_frequency", s);
      linux_iio_devices::write_string(device_path_ + "/in_anglvel_sampling_frequency", s);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "IIO IMU '%s' at %s (accel_scale=%.6g m/s^2/LSB, gyro_scale=%.6g rad/s/LSB, "
      "temp=%s, mode=%s, frame_id='%s')",
      params.iio_name.c_str(), device_path_.c_str(), accel_scale_, gyro_scale_,
      publish_temperature_ ? "yes" : "no", params.use_buffered_reads ? "buffered" : "polled",
      frame_id_.c_str());

    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu", 10);
    if (publish_temperature_)
    {
      pub_temp_ = this->create_publisher<sensor_msgs::msg::Temperature>("~/temperature", 10);
    }

    if (params.use_buffered_reads)
    {
      start_buffered_mode();
    }
    else
    {
      start_polled_mode(params.publish_rate);
    }
  }

  ~IioImuPublisher() override
  {
    stop_buffered_thread_ = true;
    if (buffered_thread_.joinable())
    {
      buffered_thread_.join();
    }
  }

private:
  void start_polled_mode(double rate_hz)
  {
    const auto period = std::chrono::duration<double>(1.0 / rate_hz);
    sample_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&IioImuPublisher::polled_callback, this));
  }

  void polled_callback()
  {
    const auto stamp = this->now();
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;

    auto ax = linux_iio_devices::read_int(device_path_ + "/in_accel_x_raw");
    auto ay = linux_iio_devices::read_int(device_path_ + "/in_accel_y_raw");
    auto az = linux_iio_devices::read_int(device_path_ + "/in_accel_z_raw");
    auto gx = linux_iio_devices::read_int(device_path_ + "/in_anglvel_x_raw");
    auto gy = linux_iio_devices::read_int(device_path_ + "/in_anglvel_y_raw");
    auto gz = linux_iio_devices::read_int(device_path_ + "/in_anglvel_z_raw");
    if (!ax || !ay || !az || !gx || !gy || !gz)
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Failed to read one or more IMU channels from %s", device_path_.c_str());
      return;
    }
    msg.linear_acceleration.x = *ax * accel_scale_;
    msg.linear_acceleration.y = *ay * accel_scale_;
    msg.linear_acceleration.z = *az * accel_scale_;
    msg.angular_velocity.x = *gx * gyro_scale_;
    msg.angular_velocity.y = *gy * gyro_scale_;
    msg.angular_velocity.z = *gz * gyro_scale_;
    msg.orientation_covariance[0] = -1.0;
    msg.angular_velocity_covariance[0] = -1.0;
    msg.linear_acceleration_covariance[0] = -1.0;
    pub_imu_->publish(msg);

    if (publish_temperature_)
    {
      if (auto t = linux_iio_devices::read_int(device_path_ + "/in_temp_raw"))
      {
        sensor_msgs::msg::Temperature tmsg;
        tmsg.header.stamp = stamp;
        tmsg.header.frame_id = frame_id_;
        // IIO temp convention: (raw + offset) * scale in milli-degrees C.
        tmsg.temperature = (*t + temp_offset_) * temp_scale_ / 1000.0;
        tmsg.variance = 0.0;
        pub_temp_->publish(tmsg);
      }
    }
  }

  void start_buffered_mode()
  {
    try
    {
      buffered_ =
        std::make_unique<linux_iio_devices::BufferedReader>(device_path_, COMMON_RECORD_SIZE);
    }
    catch (const std::exception & e)
    {
      RCLCPP_FATAL(this->get_logger(), "Failed to open buffered reader: %s", e.what());
      rclcpp::shutdown();
      return;
    }
    buffered_thread_ = std::thread([this]() { buffered_loop(); });
  }

  void buffered_loop()
  {
    std::vector<uint8_t> record;
    while (rclcpp::ok() && !stop_buffered_thread_)
    {
      if (!buffered_->wait_record(record, 500))
      {
        continue;  // timeout or short read; loop and re-check stop flag.
      }
      sensor_msgs::msg::Imu msg;
      // Kernel-supplied nanosecond timestamp; prefer it over now() for jitter.
      int64_t ts_ns = 0;
      std::memcpy(&ts_ns, record.data() + 16, sizeof(ts_ns));
      msg.header.stamp = rclcpp::Time(ts_ns);
      msg.header.frame_id = frame_id_;
      const int16_t gx = le16(record.data() + 0);
      const int16_t gy = le16(record.data() + 2);
      const int16_t gz = le16(record.data() + 4);
      const int16_t ax = le16(record.data() + 6);
      const int16_t ay = le16(record.data() + 8);
      const int16_t az = le16(record.data() + 10);
      msg.linear_acceleration.x = ax * accel_scale_;
      msg.linear_acceleration.y = ay * accel_scale_;
      msg.linear_acceleration.z = az * accel_scale_;
      msg.angular_velocity.x = gx * gyro_scale_;
      msg.angular_velocity.y = gy * gyro_scale_;
      msg.angular_velocity.z = gz * gyro_scale_;
      msg.orientation_covariance[0] = -1.0;
      msg.angular_velocity_covariance[0] = -1.0;
      msg.linear_acceleration_covariance[0] = -1.0;
      pub_imu_->publish(msg);
    }
  }

  std::shared_ptr<iio_imu_publisher::ParamListener> param_listener_;
  std::string device_path_;
  std::string frame_id_;
  double accel_scale_{0.0};
  double gyro_scale_{0.0};
  double temp_scale_{0.0};
  double temp_offset_{0.0};
  bool publish_temperature_{false};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr pub_temp_;
  rclcpp::TimerBase::SharedPtr sample_timer_;

  std::unique_ptr<linux_iio_devices::BufferedReader> buffered_;
  std::thread buffered_thread_;
  std::atomic<bool> stop_buffered_thread_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IioImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
