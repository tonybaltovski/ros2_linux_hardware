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
 * @file lcm1602.hpp
 * @brief Driver for the LCM1602 I2C LCD display (HD44780-compatible via PCF8574 expander).
 */

#pragma once

#include <unistd.h>

#include <cstdint>
#include <memory>
#include <string>

#include "linux_i2c_devices/screen.hpp"
#include "linux_i2c_interface/i2c_interface.hpp"

namespace linux_i2c_devices
{

/** @name HD44780 instruction set constants */
///@{
constexpr uint8_t LCM1602_CLEARDISPLAY = 0x01;
constexpr uint8_t LCM1602_RETURNHOME = 0x02;
constexpr uint8_t LCM1602_ENTRYMODESET = 0x04;
constexpr uint8_t LCM1602_DISPLAYCONTROL = 0x08;
constexpr uint8_t LCM1602_CURSORSHIFT = 0x10;
constexpr uint8_t LCM1602_FUNCTIONSET = 0x20;
constexpr uint8_t LCM1602_SETCGRAMADDR = 0x40;
constexpr uint8_t LCM1602_SETDDRAMADDR = 0x80;
constexpr uint8_t LCM1602_BACKLIGHT_ON = 0x08;
constexpr uint8_t LCM1602_BACKLIGHT_OFF = 0x00;
///@}

/** @name Entry-mode flags */
///@{
constexpr uint8_t LCM1602_ENTRYSHIFTINCREMENT = 0x01;
constexpr uint8_t LCM1602_ENTRYLEFT = 0x02;
///@}

/** @name Display-control flags */
///@{
constexpr uint8_t LCM1602_BLINKON = 0x01;
constexpr uint8_t LCM1602_CURSORON = 0x02;
constexpr uint8_t LCM1602_DISPLAYON = 0x04;
///@}

/** @name Function-set flags */
///@{
constexpr uint8_t LCM1602_4BITMODE = 0x00;
constexpr uint8_t LCM1602_8BITMODE = 0x10;
constexpr uint8_t LCM1602_1LINE = 0x00;
constexpr uint8_t LCM1602_2LINE = 0x08;
///@}

/** @name Data / command mode selectors */
///@{
constexpr uint8_t LCM1602_CMD = 0x00;
constexpr uint8_t LCM1602_DATA = 0x01;
constexpr uint8_t LCM1602_4BITS = 0x02;
///@}

/** @name PCF8574 control-pin masks */
///@{
constexpr uint8_t LCM1602_RS = 0x01;
constexpr uint8_t LCM1602_RW = 0x02;
constexpr uint8_t LCM1602_EN = 0x04;
///@}

/** @name Timing delays (microseconds) */
///@{
constexpr uint32_t LCM1602_DELAY_100_US = 100;
constexpr uint32_t LCM1602_DELAY_4000_US = 4000;
constexpr uint32_t LCM1602_DELAY_40000_US = 40000;
///@}

/**
 * @class Lcm1602
 * @brief Driver for the LCM1602 I2C character LCD (HD44780 via PCF8574, 4-bit mode).
 *
 * @note Each byte is issued as a one-shot `I2cInterface::write_to_bus()` rather than
 *       under a held `Transaction`: print_msg() and initialize() interleave many writes
 *       with `usleep()` delays, and per-byte release minimises bus latency for other
 *       drivers.  The LCD's sticky state (cursor) is only touched by this driver, so
 *       device-level atomicity is not required.
 */
class Lcm1602 : public Screen
{
public:
  /**
   * @brief Construct an Lcm1602 driver.
   * @param i2c_interface Shared I2C bus interface (must not be null).
   * @param device_id 7-bit I2C address of the LCD (typically 0x27).
   * @param rows Number of display rows.
   * @param columns Number of display columns.
   * @throws std::invalid_argument if @p i2c_interface is null.
   */
  Lcm1602(
    std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id,
    uint8_t rows, uint8_t columns);

  /// @copydoc Screen::clear
  int clear() override;
  /// @copydoc Screen::set_cursor
  int set_cursor(uint8_t row, uint8_t column) override;
  /// @copydoc Screen::print_msg
  int print_msg(const std::string & msg) override;
  /// @copydoc Screen::print_char
  int print_char(char c) override;
  /// @copydoc Screen::initialize
  int initialize() override;
  /// @copydoc Screen::stop
  int stop() override;
  /// @copydoc Screen::get_rows
  uint8_t get_rows() const override { return rows_; }
  /// @copydoc Screen::get_columns
  uint8_t get_columns() const override { return columns_; }

  /// @brief Return the cursor to the home (0,0) position and reset shift.
  int home();

  /// @brief Enable the display output (does not affect the backlight).
  int turn_on();

  /// @brief Disable the display output (does not affect the backlight).
  int turn_off();

  /**
   * @brief Send a single HD44780 command byte.
   * @param value Command byte (e.g. LCM1602_CLEARDISPLAY).
   * @param delay_us Microseconds to sleep after the command completes
   *        (allows the controller's internal settling time).
   * @return 0 on success, -1 on transfer failure.
   */
  int command(uint8_t value, uint32_t delay_us = 0);

  /**
   * @brief Write a byte to the LCD in either command or data mode.
   * @param value Byte to write.
   * @param mode LCM1602_CMD for instruction, LCM1602_RS for character data.
   * @return 0 on success, -1 on transfer failure.
   */
  int write(uint8_t value, uint8_t mode);

  /**
   * @brief Send the upper nibble of @p value via the PCF8574 expander.
   *
   * Used during 4-bit-mode initialisation before the full @c write() path is
   * available.  Callers normally use @c write() instead.
   */
  int write_4bits(uint8_t value);

  /**
   * @brief Write a raw expander byte (OR'd with the current backlight bit).
   * @param value Pre-formed PCF8574 byte (RS / RW / EN / data nibble).
   * @param delay_us Microseconds to sleep after the write.
   * @return 0 on success, -1 on transfer failure.
   */
  int send(uint8_t value, uint32_t delay_us = 0);

  /// @brief Pulse the LCD enable line to latch the current nibble (E high, E low).
  int pulse_enable(uint8_t value);

private:
  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface_;
  uint8_t device_id_;
  std::string log_name_;
  uint8_t rows_;
  uint8_t columns_;
  uint8_t backlight_;
  bool initialized_;
};

}  // namespace linux_i2c_devices
