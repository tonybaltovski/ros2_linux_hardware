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
 * @brief Compute the register address for a per-channel LED register.
 *
 * Each of the 16 channels (0-15) occupies four consecutive bytes starting
 * at 0x06: ON_L, ON_H, OFF_L, OFF_H.
 *
 * @param channel LED channel number (0-15).
 * @param offset  Register offset within the channel group (0 = ON_L, 1 = ON_H,
 *                2 = OFF_L, 3 = OFF_H).
 * @return Register address.
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
 * Typical usage:
 * @code
 * auto i2c = std::make_shared<linux_i2c_interface::I2cInterface>(1);
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
   * @param i2c_interface Shared I2C bus interface.
   * @param device_id 7-bit I2C address (default 0x40).
   */
  Pca9685(std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id);

  /**
   * @brief Initialise the PCA9685.
   *
   * Resets the device, enables auto-increment, sets totem-pole output,
   * configures the default PWM frequency, and wakes the oscillator.
   *
   * @return 0 on success, -1 on failure.
   */
  int initialize();

  /**
   * @brief Set the PWM frequency for all channels.
   * @param freq_hz Desired frequency in Hz (24-1526 Hz range).
   * @return 0 on success, -1 on failure.
   */
  int set_pwm_frequency(double freq_hz);

  /**
   * @brief Set raw ON/OFF tick counts for a single channel.
   * @param channel Channel number (0-15).
   * @param on  12-bit tick count at which the output turns ON.
   * @param off 12-bit tick count at which the output turns OFF.
   * @return 0 on success, -1 on failure.
   */
  int set_pwm(uint8_t channel, uint16_t on, uint16_t off);

  /**
   * @brief Set the duty cycle for a single channel.
   * @param channel    Channel number (0-15).
   * @param duty_cycle Duty cycle in the range [0.0, 1.0].
   * @return 0 on success, -1 on failure.
   */
  int set_duty_cycle(uint8_t channel, double duty_cycle);

  /**
   * @brief Set raw ON/OFF tick counts for all 16 channels simultaneously.
   * @param on  12-bit ON tick count.
   * @param off 12-bit OFF tick count.
   * @return 0 on success, -1 on failure.
   */
  int set_all_pwm(uint16_t on, uint16_t off);

  /**
   * @brief Put the device into sleep mode (oscillator off).
   * @return 0 on success, -1 on failure.
   */
  int sleep();

  /**
   * @brief Wake the device from sleep mode.
   * @return 0 on success, -1 on failure.
   */
  int wake_up();

  /** @brief Turn off all channels and close the I2C bus connection. */
  void stop();

private:
  /**
   * @brief Write a single byte to a register.
   * @param reg  Register address.
   * @param value Byte to write.
   * @return 0 on success, -1 on failure.
   */
  int write_register(uint8_t reg, uint8_t value);

  /**
   * @brief Read a single byte from a register.
   * @param reg Register address.
   * @param[out] value Byte read.
   * @return 0 on success, -1 on failure.
   */
  int read_register(uint8_t reg, uint8_t & value);

  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface_;
  uint8_t device_id_;
  double pwm_frequency_{PCA9685_DEFAULT_FREQ};  ///< Current PWM frequency (Hz).
  bool initialized_{false};
};

}  // namespace linux_i2c_devices
