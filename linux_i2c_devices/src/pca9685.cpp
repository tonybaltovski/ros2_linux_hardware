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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "linux_i2c_devices/pca9685.hpp"
#include "rclcpp/logging.hpp"

namespace linux_i2c_devices
{
namespace
{
std::shared_ptr<linux_i2c_interface::I2cInterface> require_iface(
  std::shared_ptr<linux_i2c_interface::I2cInterface> iface)
{
  if (!iface)
  {
    throw std::invalid_argument("Pca9685: i2c_interface must not be null");
  }
  return iface;
}

std::string make_log_name(const std::string & bus, uint8_t addr)
{
  const auto slash = bus.find_last_of('/');
  const std::string base = slash == std::string::npos ? bus : bus.substr(slash + 1);
  char buf[8];
  std::snprintf(buf, sizeof(buf), "0x%02x", static_cast<int>(addr));
  return "pca9685." + base + "." + buf;
}
rclcpp::Logger logger(const std::string & name) { return rclcpp::get_logger(name); }
}  // namespace

Pca9685::Pca9685(
  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id)
: i2c_interface_(require_iface(std::move(i2c_interface))),
  device_id_(device_id),
  log_name_(make_log_name(i2c_interface_->bus_name(), device_id))
{
}

int Pca9685::initialize()
{
  if (initialized_)
  {
    return 0;
  }

  RCLCPP_INFO(logger(log_name_), "%s: Starting initialization", __func__);

  auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
  if (!i2c_transaction.ok())
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to start transaction", __func__);
    return -1;
  }

  uint8_t value;

  // Reset: put device to sleep so we can configure the prescaler.
  value = PCA9685_MODE1_SLEEP;
  if (i2c_transaction.write(PCA9685_MODE1, &value, 1) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to set sleep mode", __func__);
    return -1;
  }

  if (set_pwm_frequency_unlocked(i2c_transaction, PCA9685_DEFAULT_FREQ) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to set default PWM frequency", __func__);
    return -1;
  }

  // Configure MODE2: totem-pole outputs, outputs change on STOP.
  value = PCA9685_MODE2_OUTDRV;
  if (i2c_transaction.write(PCA9685_MODE2, &value, 1) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to configure MODE2", __func__);
    return -1;
  }

  // Wake up: enable auto-increment, clear sleep bit.
  value = PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL;
  if (i2c_transaction.write(PCA9685_MODE1, &value, 1) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to wake device", __func__);
    return -1;
  }

  // The oscillator needs 500 us to stabilize after wake-up.
  std::this_thread::sleep_for(std::chrono::microseconds(500));

  // Turn off all channels.
  uint8_t zero = 0;
  if (
    i2c_transaction.write(PCA9685_ALL_LED_ON_L, &zero, 1) < 0 ||
    i2c_transaction.write(PCA9685_ALL_LED_ON_H, &zero, 1) < 0 ||
    i2c_transaction.write(PCA9685_ALL_LED_OFF_L, &zero, 1) < 0 ||
    i2c_transaction.write(PCA9685_ALL_LED_OFF_H, &zero, 1) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to turn off all channels", __func__);
    return -1;
  }

  initialized_ = true;
  RCLCPP_INFO(logger(log_name_), "%s: Initialization done", __func__);
  return 0;
}

int Pca9685::set_pwm_frequency_unlocked(Transaction & i2c_transaction, double freq_hz)
{
  // Clamp to the valid range (datasheet: 24 Hz – 1526 Hz).
  freq_hz = std::clamp(freq_hz, 24.0, 1526.0);

  // Prescaler formula from the PCA9685 datasheet:
  //   prescale = round(osc_clock / (4096 * freq)) - 1
  double prescale_val = std::round(PCA9685_INTERNAL_CLK / (PCA9685_MAX_COUNT * freq_hz)) - 1.0;
  uint8_t prescale = static_cast<uint8_t>(std::clamp(prescale_val, 3.0, 255.0));

  // The prescaler can only be set while the device is in sleep mode.
  uint8_t old_mode = 0;
  if (i2c_transaction.read(PCA9685_MODE1, &old_mode, 1) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to read MODE1", __func__);
    return -1;
  }

  uint8_t sleep_mode = (old_mode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
  if (i2c_transaction.write(PCA9685_MODE1, &sleep_mode, 1) < 0)
  {
    RCLCPP_ERROR(
      logger(log_name_), "%s: Failed to enter sleep mode for prescaler change", __func__);
    return -1;
  }

  if (i2c_transaction.write(PCA9685_PRE_SCALE, &prescale, 1) < 0)
  {
    RCLCPP_ERROR(
      logger(log_name_), "%s: Failed to set prescaler to %d", __func__, static_cast<int>(prescale));
    return -1;
  }

  // Restore previous mode (wake up).
  if (i2c_transaction.write(PCA9685_MODE1, &old_mode, 1) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to restore MODE1 after prescaler change", __func__);
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(500));

  // Set RESTART bit to re-enable any previously active PWM channels.
  uint8_t restart_mode = old_mode | PCA9685_MODE1_RESTART;
  if (i2c_transaction.write(PCA9685_MODE1, &restart_mode, 1) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to set RESTART bit", __func__);
    return -1;
  }

  pwm_frequency_ = freq_hz;
  RCLCPP_INFO(
    logger(log_name_), "%s: Set frequency to %.2f Hz (prescale=%d)", __func__, freq_hz,
    static_cast<int>(prescale));
  return 0;
}

int Pca9685::set_pwm_frequency(double freq_hz)
{
  auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
  if (!i2c_transaction.ok())
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to start transaction", __func__);
    return -1;
  }
  return set_pwm_frequency_unlocked(i2c_transaction, freq_hz);
}

int Pca9685::set_pwm(uint8_t channel, uint16_t on, uint16_t off)
{
  if (channel >= PCA9685_NUM_CHANNELS)
  {
    RCLCPP_ERROR(
      logger(log_name_), "%s: Channel %d out of range (0-15)", __func__, static_cast<int>(channel));
    return -1;
  }

  // Send all four bytes in one auto-increment burst (MODE1_AI is set in init).
  uint8_t payload[4] = {
    static_cast<uint8_t>(on & 0xFF),
    static_cast<uint8_t>((on >> 8) & 0x0F),
    static_cast<uint8_t>(off & 0xFF),
    static_cast<uint8_t>((off >> 8) & 0x0F),
  };

  auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
  if (!i2c_transaction.ok())
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to start transaction", __func__);
    return -1;
  }
  if (i2c_transaction.write(pca9685_led_reg(channel, 0), payload, sizeof(payload)) < 0)
  {
    RCLCPP_ERROR(
      logger(log_name_), "%s: Failed to set PWM for channel %d", __func__,
      static_cast<int>(channel));
    return -1;
  }
  return 0;
}

int Pca9685::set_duty_cycle(uint8_t channel, double duty_cycle)
{
  duty_cycle = std::clamp(duty_cycle, 0.0, 1.0);
  uint16_t off_count = static_cast<uint16_t>(duty_cycle * (PCA9685_MAX_COUNT - 1));
  return set_pwm(channel, 0, off_count);
}

int Pca9685::set_all_pwm(uint16_t on, uint16_t off)
{
  uint8_t payload[4] = {
    static_cast<uint8_t>(on & 0xFF),
    static_cast<uint8_t>((on >> 8) & 0x0F),
    static_cast<uint8_t>(off & 0xFF),
    static_cast<uint8_t>((off >> 8) & 0x0F),
  };

  auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
  if (!i2c_transaction.ok())
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to start transaction", __func__);
    return -1;
  }
  if (i2c_transaction.write(PCA9685_ALL_LED_ON_L, payload, sizeof(payload)) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to set all PWM channels", __func__);
    return -1;
  }
  return 0;
}

int Pca9685::sleep()
{
  auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
  if (!i2c_transaction.ok())
  {
    return -1;
  }
  uint8_t mode = 0;
  if (i2c_transaction.read(PCA9685_MODE1, &mode, 1) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to read MODE1", __func__);
    return -1;
  }
  uint8_t new_mode = mode | PCA9685_MODE1_SLEEP;
  return i2c_transaction.write(PCA9685_MODE1, &new_mode, 1);
}

int Pca9685::wake_up()
{
  auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
  if (!i2c_transaction.ok())
  {
    return -1;
  }
  uint8_t mode = 0;
  if (i2c_transaction.read(PCA9685_MODE1, &mode, 1) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to read MODE1", __func__);
    return -1;
  }
  uint8_t new_mode = mode & ~PCA9685_MODE1_SLEEP;
  if (i2c_transaction.write(PCA9685_MODE1, &new_mode, 1) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to clear sleep bit", __func__);
    return -1;
  }
  std::this_thread::sleep_for(std::chrono::microseconds(500));
  // Re-enable any previously active PWM channels.
  uint8_t restart_mode = new_mode | PCA9685_MODE1_RESTART;
  return i2c_transaction.write(PCA9685_MODE1, &restart_mode, 1);
}

void Pca9685::stop()
{
  set_all_pwm(0, 0);
  sleep();
}

}  // namespace linux_i2c_devices
