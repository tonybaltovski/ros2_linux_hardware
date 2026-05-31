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
 * @file pca9685.hpp
 * @brief Driver for the PCA9685 16-channel, 12-bit PWM/LED controller over I2C.
 */

#pragma once

#include <unistd.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "linux_i2c_interface/i2c_interface.hpp"

namespace linux_i2c_devices
{

/** @name PCA9685 register addresses */
///@{
constexpr uint8_t PCA9685_DEFAULT_ADDRESS = 0x40;

constexpr uint8_t PCA9685_MODE1 = 0x00;
constexpr uint8_t PCA9685_MODE2 = 0x01;
constexpr uint8_t PCA9685_SUBADR1 = 0x02;
constexpr uint8_t PCA9685_SUBADR2 = 0x03;
constexpr uint8_t PCA9685_SUBADR3 = 0x04;
constexpr uint8_t PCA9685_ALLCALLADR = 0x05;
///@}

/** @name MODE1 register bits */
///@{
constexpr uint8_t PCA9685_MODE1_ALLCALL = 0x01;  ///< Respond to LED all-call address.
constexpr uint8_t PCA9685_MODE1_SUB3 = 0x02;     ///< Respond to sub-address 3.
constexpr uint8_t PCA9685_MODE1_SUB2 = 0x04;     ///< Respond to sub-address 2.
constexpr uint8_t PCA9685_MODE1_SUB1 = 0x08;     ///< Respond to sub-address 1.
constexpr uint8_t PCA9685_MODE1_SLEEP = 0x10;    ///< Low-power / oscillator off.
constexpr uint8_t PCA9685_MODE1_AI = 0x20;       ///< Auto-increment register pointer.
constexpr uint8_t PCA9685_MODE1_EXTCLK = 0x40;   ///< Use external clock.
constexpr uint8_t PCA9685_MODE1_RESTART = 0x80;  ///< Restart enabled PWM channels.
///@}

/** @name MODE2 register bits */
///@{
constexpr uint8_t PCA9685_MODE2_OUTNE_MASK = 0x03;  ///< Output-not-enabled control.
constexpr uint8_t PCA9685_MODE2_OUTDRV = 0x04;      ///< Totem-pole (1) vs open-drain (0).
constexpr uint8_t PCA9685_MODE2_OCH = 0x08;         ///< Outputs change on ACK (1) vs STOP (0).
constexpr uint8_t PCA9685_MODE2_INVRT = 0x10;       ///< Invert output logic.
///@}

/**
 * @brief Register address for a per-channel LED register.
 *
 * Each of the 16 channels occupies four consecutive bytes starting at 0x06:
 * ON_L, ON_H, OFF_L, OFF_H.  @p offset selects one of these (0..3).
 */
constexpr uint8_t pca9685_led_reg(uint8_t channel, uint8_t offset)
{
  return static_cast<uint8_t>(0x06 + channel * 4 + offset);
}

/** @name "All LED" broadcast registers */
///@{
constexpr uint8_t PCA9685_ALL_LED_ON_L = 0xFA;
constexpr uint8_t PCA9685_ALL_LED_ON_H = 0xFB;
constexpr uint8_t PCA9685_ALL_LED_OFF_L = 0xFC;
constexpr uint8_t PCA9685_ALL_LED_OFF_H = 0xFD;
///@}

/** @name Miscellaneous registers */
///@{
constexpr uint8_t PCA9685_PRE_SCALE = 0xFE;
constexpr uint8_t PCA9685_TEST_MODE = 0xFF;
///@}

/** @name Driver constants */
///@{
constexpr uint16_t PCA9685_MAX_COUNT = 4096;         ///< 12-bit counter range.
constexpr double PCA9685_INTERNAL_CLK = 25000000.0;  ///< 25 MHz internal oscillator.
constexpr double PCA9685_DEFAULT_FREQ = 50.0;        ///< Default PWM frequency (Hz).
constexpr uint8_t PCA9685_NUM_CHANNELS = 16;         ///< Number of PWM channels.
///@}

/**
 * @class Pca9685
 * @brief Driver for the PCA9685 16-channel, 12-bit PWM controller.
 *
 * Multi-register operations (e.g. set_pwm, set_pwm_frequency) are issued
 * inside a single I2C `Transaction` so they remain atomic on a shared bus.
 *
 * Typical usage:
 * @code
 * auto i2c = linux_i2c_interface::I2cInterface::get_shared(1);
 * linux_i2c_devices::Pca9685 pwm(i2c, 0x40);
 * pwm.initialize();
 * pwm.set_pwm_frequency(50.0);   // 50 Hz for servos
 * pwm.set_duty_cycle(0, 0.075);  // ~1.5 ms pulse → servo centre
 * @endcode
 */
class Pca9685
{
public:
  /**
   * @brief Construct a Pca9685 driver.
   * @param i2c_interface Shared I2C bus interface (must not be null).
   * @param device_id 7-bit I2C address (default 0x40).
   * @throws std::invalid_argument if @p i2c_interface is null.
   */
  Pca9685(std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id);

  /**
   * @brief Run the power-on sequence and program the default PWM frequency.
   * @return 0 on success, -1 on transfer failure (errno set).
   */
  int initialize();

  /**
   * @brief Set the PWM frequency for all 16 channels.
   * @param freq_hz Target frequency in Hz; clamped to the 24..1526 Hz range
   *        supported by the PCA9685's prescaler.
   * @return 0 on success, -1 on transfer failure.
   */
  int set_pwm_frequency(double freq_hz);

  /**
   * @brief Set raw ON/OFF tick counts for one channel.
   * @param channel 0..15.
   * @param on  Tick (0..4095) at which the output rises.
   * @param off Tick (0..4095) at which the output falls.
   * @return 0 on success, -1 on transfer or argument failure.
   */
  int set_pwm(uint8_t channel, uint16_t on, uint16_t off);

  /**
   * @brief Set the duty cycle of a channel as a fraction of the PWM period.
   * @param channel 0..15.
   * @param duty_cycle 0.0 (always off) .. 1.0 (always on); clamped.
   * @return 0 on success, -1 on transfer or argument failure.
   */
  int set_duty_cycle(uint8_t channel, double duty_cycle);

  /**
   * @brief Set raw ON/OFF tick counts for all 16 channels in one I2C burst.
   * @param on  Tick (0..4095) at which all outputs rise.
   * @param off Tick (0..4095) at which all outputs fall.
   * @return 0 on success, -1 on transfer failure.
   */
  int set_all_pwm(uint16_t on, uint16_t off);

  /// @brief Put the device into sleep mode (oscillator off).
  int sleep();

  /// @brief Wake the device from sleep mode and wait for the oscillator to settle.
  int wake_up();

  /// @brief Drive all outputs to 0.
  void stop();

  /// @brief Currently-programmed PWM frequency in Hz (last value set; default
  /// PCA9685_DEFAULT_FREQ).
  double frequency_hz() const { return pwm_frequency_; }

  /// @brief Underlying I2C address of this PCA9685.
  uint8_t device_id() const { return device_id_; }

private:
  using Transaction = linux_i2c_interface::I2cInterface::Transaction;

  /// @brief Set the PWM prescaler (caller already holds the transaction lock).
  int set_pwm_frequency_unlocked(Transaction & i2c_transaction, double freq_hz);

  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface_;
  uint8_t device_id_;
  std::string log_name_;
  double pwm_frequency_{PCA9685_DEFAULT_FREQ};
  bool initialized_{false};
  // Serialises public entry points so concurrent callers cannot interleave
  // multi-register writes (set_pwm) or the read-modify-write in sleep()/
  // wake_up().  Recursive because stop() and set_duty_cycle() chain through
  // other public methods.
  mutable std::recursive_mutex device_mutex_;
};

}  // namespace linux_i2c_devices
