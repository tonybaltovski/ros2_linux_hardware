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
 * @file screen.hpp
 * @brief Abstract base class for I2C text-capable screen devices.
 */

#pragma once

#include <cstdint>
#include <string>

namespace linux_i2c_devices
{

/**
 * @class Screen
 * @brief Interface for I2C screen devices that can display text.
 *
 * Both the LCM1602 character LCD and SSD1306 OLED implement this interface,
 * allowing application code to be written against a single type.
 */
class Screen
{
public:
  virtual ~Screen() = default;

  /**
   * @brief Perform the power-on initialisation sequence.
   * @return 0 on success, negative on failure.
   */
  virtual int8_t initialize() = 0;

  /**
   * @brief Clear the display contents.
   * @return 0 on success, negative on failure.
   */
  virtual int8_t clear() = 0;

  /**
   * @brief Move the text cursor to a character-grid position.
   * @param row    Row index (0-based).
   * @param column Column index (0-based).
   * @return 0 on success, negative on failure.
   */
  virtual int8_t set_cursor(uint8_t row, uint8_t column) = 0;

  /**
   * @brief Print a single character at the current cursor position.
   * @param c Character to print.
   * @return 0 on success, negative on failure.
   */
  virtual int8_t print_char(char c) = 0;

  /**
   * @brief Print a string starting at the current cursor, wrapping across rows.
   * @param msg The message to display.
   * @return 0 on success, negative on failure.
   */
  virtual int8_t print_msg(const std::string & msg) = 0;

  /**
   * @brief Shut down the display and release the I2C bus.
   * @return 0 on success, negative on failure.
   */
  virtual int8_t stop() = 0;

  /** @brief Number of text rows the display can show. */
  virtual uint8_t get_rows() const = 0;

  /** @brief Number of text columns the display can show. */
  virtual uint8_t get_columns() const = 0;
};

}  // namespace linux_i2c_devices
