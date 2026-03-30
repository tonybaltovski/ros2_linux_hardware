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
#include <cmath>
#include <iostream>

#include "linux_i2c_devices/pca9685.hpp"

namespace linux_i2c_devices
{

Pca9685::Pca9685(
  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id)
: i2c_interface_(i2c_interface), device_id_(device_id)
{
}

int Pca9685::write_register(uint8_t reg, uint8_t value)
{
  return i2c_interface_->write_to_bus(reg, &value, 1);
}

int Pca9685::read_register(uint8_t reg, uint8_t & value)
{
  return i2c_interface_->read_from_bus(reg, &value, 1);
}

int Pca9685::initialize()
{
  if (initialized_)
  {
    return 0;
  }

  std::cout << __PRETTY_FUNCTION__ << ": Starting initialization" << std::endl;

  // Open bus and select device.
  if (i2c_interface_->open_bus() < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to open I2C bus" << std::endl;
    return -1;
  }
  if (i2c_interface_->set_device_id(device_id_) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to set device ID" << std::endl;
    return -1;
  }

  // Reset: put device to sleep so we can configure the prescaler.
  if (write_register(PCA9685_MODE1, PCA9685_MODE1_SLEEP) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to set sleep mode" << std::endl;
    return -1;
  }

  // Set default PWM frequency.
  if (set_pwm_frequency(PCA9685_DEFAULT_FREQ) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to set default PWM frequency" << std::endl;
    return -1;
  }

  // Configure MODE2: totem-pole outputs, outputs change on STOP.
  if (write_register(PCA9685_MODE2, PCA9685_MODE2_OUTDRV) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to configure MODE2" << std::endl;
    return -1;
  }

  // Wake up: enable auto-increment, clear sleep bit.
  if (write_register(PCA9685_MODE1, PCA9685_MODE1_AI | PCA9685_MODE1_ALLCALL) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to wake device" << std::endl;
    return -1;
  }

  // The oscillator needs 500 us to stabilize after wake-up.
  usleep(500);

  // Turn off all channels.
  set_all_pwm(0, 0);

  initialized_ = true;
  std::cout << __PRETTY_FUNCTION__ << ": Initialization done" << std::endl;
  return 0;
}

int Pca9685::set_pwm_frequency(double freq_hz)
{
  // Clamp to the valid range (datasheet: 24 Hz – 1526 Hz).
  freq_hz = std::clamp(freq_hz, 24.0, 1526.0);

  // Prescaler formula from the PCA9685 datasheet:
  //   prescale = round(osc_clock / (4096 * freq)) - 1
  double prescale_val = std::round(PCA9685_INTERNAL_CLK / (PCA9685_MAX_COUNT * freq_hz)) - 1.0;
  uint8_t prescale = static_cast<uint8_t>(std::clamp(prescale_val, 3.0, 255.0));

  // The prescaler can only be set while the device is in sleep mode.
  uint8_t old_mode = 0;
  if (read_register(PCA9685_MODE1, old_mode) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to read MODE1" << std::endl;
    return -1;
  }

  uint8_t sleep_mode = (old_mode & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
  if (write_register(PCA9685_MODE1, sleep_mode) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to enter sleep mode for prescaler change" << std::endl;
    return -1;
  }

  if (write_register(PCA9685_PRE_SCALE, prescale) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to set prescaler to "
              << static_cast<int>(prescale) << std::endl;
    return -1;
  }

  // Restore previous mode (wake up).
  if (write_register(PCA9685_MODE1, old_mode) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to restore MODE1 after prescaler change" << std::endl;
    return -1;
  }
  usleep(500);

  // Set RESTART bit to re-enable any previously active PWM channels.
  if (write_register(PCA9685_MODE1, old_mode | PCA9685_MODE1_RESTART) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to set RESTART bit" << std::endl;
    return -1;
  }

  pwm_frequency_ = freq_hz;
  std::cout << __PRETTY_FUNCTION__ << ": Set frequency to " << freq_hz
            << " Hz (prescale=" << static_cast<int>(prescale) << ")" << std::endl;
  return 0;
}

int Pca9685::set_pwm(uint8_t channel, uint16_t on, uint16_t off)
{
  if (channel >= PCA9685_NUM_CHANNELS)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Channel " << static_cast<int>(channel)
              << " out of range (0-15)" << std::endl;
    return -1;
  }

  uint8_t on_l = static_cast<uint8_t>(on & 0xFF);
  uint8_t on_h = static_cast<uint8_t>((on >> 8) & 0x0F);
  uint8_t off_l = static_cast<uint8_t>(off & 0xFF);
  uint8_t off_h = static_cast<uint8_t>((off >> 8) & 0x0F);

  if (
    write_register(pca9685_led_reg(channel, 0), on_l) < 0 ||
    write_register(pca9685_led_reg(channel, 1), on_h) < 0 ||
    write_register(pca9685_led_reg(channel, 2), off_l) < 0 ||
    write_register(pca9685_led_reg(channel, 3), off_h) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to set PWM for channel "
              << static_cast<int>(channel) << std::endl;
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
  uint8_t on_l = static_cast<uint8_t>(on & 0xFF);
  uint8_t on_h = static_cast<uint8_t>((on >> 8) & 0x0F);
  uint8_t off_l = static_cast<uint8_t>(off & 0xFF);
  uint8_t off_h = static_cast<uint8_t>((off >> 8) & 0x0F);

  if (
    write_register(PCA9685_ALL_LED_ON_L, on_l) < 0 ||
    write_register(PCA9685_ALL_LED_ON_H, on_h) < 0 ||
    write_register(PCA9685_ALL_LED_OFF_L, off_l) < 0 ||
    write_register(PCA9685_ALL_LED_OFF_H, off_h) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to set all PWM channels" << std::endl;
    return -1;
  }
  return 0;
}

int Pca9685::sleep()
{
  uint8_t mode = 0;
  if (read_register(PCA9685_MODE1, mode) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to read MODE1" << std::endl;
    return -1;
  }
  return write_register(PCA9685_MODE1, mode | PCA9685_MODE1_SLEEP);
}

int Pca9685::wake_up()
{
  uint8_t mode = 0;
  if (read_register(PCA9685_MODE1, mode) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to read MODE1" << std::endl;
    return -1;
  }
  uint8_t new_mode = mode & ~PCA9685_MODE1_SLEEP;
  if (write_register(PCA9685_MODE1, new_mode) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to clear sleep bit" << std::endl;
    return -1;
  }
  usleep(500);
  // Re-enable any previously active PWM channels.
  return write_register(PCA9685_MODE1, new_mode | PCA9685_MODE1_RESTART);
}

void Pca9685::stop()
{
  set_all_pwm(0, 0);
  sleep();
  if (i2c_interface_->is_connected())
  {
    i2c_interface_->close_bus();
  }
}

}  // namespace linux_i2c_devices
