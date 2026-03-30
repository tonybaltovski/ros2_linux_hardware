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
 * @brief Driver for the LCM1602 I2C character LCD module.
 *
 * The display is assumed to be wired through a PCF8574 I/O expander
 * and driven in 4-bit mode.
 */
class Lcm1602 : public Screen
{
public:
  /**
   * @brief Construct an Lcm1602 driver.
   * @param i2c_interface Shared I2C bus interface.
   * @param device_id 7-bit I2C address of the LCD (typically 0x27).
   * @param rows Number of display rows.
   * @param columns Number of display columns.
   */
  Lcm1602(
    std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id,
    uint8_t rows, uint8_t columns);

  /**
   * @brief Clear the display.
   * @return 0 on success, negative on failure.
   */
  int8_t clear() override;

  /**
   * @brief Return the cursor to the home position.
   * @return 0 on success, negative on failure.
   */
  int8_t home();

  /**
   * @brief Move the cursor to a specific position.
   * @param row    Row index (0-based).
   * @param column Column index (0-based).
   * @return 0 on success, negative on failure.
   */
  int8_t set_cursor(uint8_t row, uint8_t column) override;

  /**
   * @brief Print a string, wrapping across rows automatically.
   * @param msg The message to display.
   * @return 0 on success, negative on failure.
   */
  int8_t print_msg(const std::string & msg) override;

  /**
   * @brief Print a single character at the current cursor position.
   * @param c Character to print.
   * @return 0 on success, negative on failure.
   */
  int8_t print_char(char c) override;

  /**
   * @brief Perform the power-on initialisation sequence.
   * @return 0 on success, negative on failure.
   */
  int8_t initialize() override;

  /**
   * @brief Turn the display on.
   * @return 0 on success, negative on failure.
   */
  int8_t turn_on();

  /**
   * @brief Turn the display off.
   * @return 0 on success, negative on failure.
   */
  int8_t turn_off();

  /**
   * @brief Clear the display, turn off the backlight, and close the bus.
   * @return 0 on success, negative on failure.
   */
  int8_t stop() override;

  /** @brief Number of text rows the display can show. */
  uint8_t get_rows() const override { return rows_; }

  /** @brief Number of text columns the display can show. */
  uint8_t get_columns() const override { return columns_; }

  /**
   * @brief Send a command byte to the display.
   * @param value    Command byte.
   * @param delay_us Delay in microseconds after the command.
   * @return 0 on success, negative on failure.
   */
  int8_t command(uint8_t value, uint32_t delay_us = 0);

  /**
   * @brief Write a byte in command or data mode.
   * @param value Byte to send.
   * @param mode  LCM1602_CMD or LCM1602_RS.
   * @return 0 on success, negative on failure.
   */
  int8_t write(uint8_t value, uint8_t mode);

  /**
   * @brief Write the upper nibble to the bus.
   * @param value Upper 4 bits to send.
   * @return 0 on success, negative on failure.
   */
  int8_t write_4bits(uint8_t value);

  /**
   * @brief Send a raw byte to the I2C expander.
   * @param value    Byte (combined with backlight state).
   * @param delay_us Delay in microseconds after the send.
   * @return 0 on success, negative on failure.
   */
  int8_t send(uint8_t value, uint32_t delay_us = 0);

  /**
   * @brief Pulse the enable pin to latch data.
   * @param value Current data nibble on the bus.
   * @return 0 on success, negative on failure.
   */
  int8_t pulse_enable(uint8_t value);

private:
  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface_;  ///< Shared I2C bus.
  uint8_t device_id_;   ///< 7-bit I2C slave address.
  uint8_t rows_;        ///< Number of display rows.
  uint8_t columns_;     ///< Number of display columns.
  uint8_t backlight_;   ///< Current backlight state.
  bool initialized_;    ///< Whether initialize() has completed.
};

}  // namespace linux_i2c_devices
