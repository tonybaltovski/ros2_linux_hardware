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
 * @file hmc6343.hpp
 * @brief Driver for the Honeywell HMC6343 tilt-compensated I2C compass.
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "linux_i2c_devices/magnetometer.hpp"
#include "linux_i2c_interface/i2c_interface.hpp"

namespace linux_i2c_devices
{

/** @name HMC6343 default I2C address (7-bit). */
///@{
constexpr uint8_t HMC6343_DEFAULT_ADDRESS = 0x19;
///@}

/** @name HMC6343 command set (single-byte commands posted to the device). */
///@{
constexpr uint8_t HMC6343_CMD_POST_ACCEL = 0x40;    ///< Post 6 bytes accel  (Ax,Ay,Az  int16).
constexpr uint8_t HMC6343_CMD_POST_MAG = 0x45;      ///< Post 6 bytes mag    (Bx,By,Bz  int16).
constexpr uint8_t HMC6343_CMD_POST_HEADING = 0x50;  ///< Post 6 bytes (heading,pitch,roll int16).
constexpr uint8_t HMC6343_CMD_POST_TILT = 0x55;     ///< Post 6 bytes (pitch,roll,temperature).
constexpr uint8_t HMC6343_CMD_READ_EEPROM = 0xE1;   ///< Followed by 1 byte EEPROM address.
constexpr uint8_t HMC6343_CMD_WRITE_EEPROM = 0xF1;  ///< Followed by 2 bytes: address, value.
constexpr uint8_t HMC6343_CMD_ENTER_STANDBY = 0x76;
constexpr uint8_t HMC6343_CMD_ENTER_RUN = 0x77;
constexpr uint8_t HMC6343_CMD_RESET = 0x82;
constexpr uint8_t HMC6343_CMD_ENTER_SLEEP = 0x83;
constexpr uint8_t HMC6343_CMD_EXIT_SLEEP = 0x84;
///@}

/** @name HMC6343 EEPROM addresses. */
///@{
constexpr uint8_t HMC6343_EEPROM_SLAVE_ADDR = 0x00;  ///< 8-bit (write-direction) slave address.
constexpr uint8_t HMC6343_EEPROM_SW_VERSION = 0x02;
constexpr uint8_t HMC6343_EEPROM_OPMODE1 = 0x04;
constexpr uint8_t HMC6343_EEPROM_OPMODE2 = 0x05;
///@}

/** @name Datasheet timings (microseconds). */
///@{
constexpr uint32_t HMC6343_POST_DELAY_US = 1000;          ///< Wait after a POST_* command.
constexpr uint32_t HMC6343_EEPROM_READ_DELAY_US = 10000;  ///< Wait after READ_EEPROM command.
constexpr uint32_t HMC6343_EEPROM_WRITE_DELAY_US = 10000;
///@}

/**
 * @name Output scaling constants.
 *
 * Heading / pitch / roll are reported in tenths of a degree (signed int16).
 *
 * The raw magnetic output is the chip's on-board compensated value that it
 * uses internally for heading; the datasheet does not specify an SI scale.
 * 1 LSB ≈ 0.1 µT (≈ 1 mGauss) is a commonly used nominal conversion — treat
 * the µT output as approximate and recalibrate per device when absolute field
 * magnitude matters.
 */
///@{
constexpr double HMC6343_ANGLE_LSB_TO_DEG = 0.1;
constexpr double HMC6343_MAG_LSB_TO_UT = 0.1;
///@}

/**
 * @class Hmc6343
 * @brief Driver for the Honeywell HMC6343 tilt-compensated digital compass.
 *
 * The device follows a "post and read" protocol rather than the more common
 * register-pointer one: the host writes a single command byte (e.g.
 * @c HMC6343_CMD_POST_HEADING) and then, after ≥1 ms, performs a plain I²C
 * read of N bytes.  This driver implements that pattern by issuing the
 * command and the read in two separate I²C transactions with a sleep in
 * between, so the bus mutex is released during the wait and other drivers
 * on the same bus may proceed.
 *
 * All @c int-returning methods return 0 on success, -1 on failure.
 */
class Hmc6343 : public Magnetometer
{
public:
  /**
   * @brief Construct a Hmc6343 driver.
   * @param i2c_interface Shared I2C bus interface (must not be null).
   * @param device_id 7-bit I2C address (default 0x19).
   * @throws std::invalid_argument if @p i2c_interface is null.
   */
  Hmc6343(
    std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface,
    uint8_t device_id = HMC6343_DEFAULT_ADDRESS);

  /// @copydoc Magnetometer::initialize
  int initialize() override;
  /// @copydoc Magnetometer::stop
  int stop() override;
  /// @copydoc Magnetometer::read_field_ut
  int read_field_ut(double & x_ut, double & y_ut, double & z_ut) override;
  /// @copydoc Magnetometer::read_heading_deg
  int read_heading_deg(double & heading_deg) override;
  /// @copydoc Magnetometer::read_tilt_deg
  int read_tilt_deg(double & pitch_deg, double & roll_deg) override;

  /**
   * @brief Read the raw int16 magnetic-field counts in the chip's native units.
   *        Useful if you want to apply your own calibration / scale.
   */
  int read_field_raw(int16_t & x, int16_t & y, int16_t & z);

  /**
   * @brief Read the on-chip accelerometer in g.  1024 LSB = 1 g per datasheet.
   */
  int read_accel_g(double & x_g, double & y_g, double & z_g);

  /**
   * @brief Read a single byte from the device EEPROM.
   * @param address EEPROM byte address (see HMC6343_EEPROM_* constants).
   */
  int read_eeprom(uint8_t address, uint8_t & value);

private:
  /**
   * @brief Post @p cmd, wait @p delay_us, then read @p count bytes back.
   *        The bus is released between the command and the read.
   */
  int post_and_read(uint8_t cmd, uint8_t * buffer, uint32_t count, uint32_t delay_us);

  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface_;
  uint8_t device_id_;
  std::string log_name_;
  bool initialized_;
};

}  // namespace linux_i2c_devices
