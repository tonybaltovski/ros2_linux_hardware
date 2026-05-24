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
 * @brief Common interface for I2C text-capable screens (LCM1602, SSD1306).
 *
 * All @c int-returning methods return 0 on success and a negative value on failure.
 */
class Screen
{
public:
  virtual ~Screen() = default;

  /// @brief Run the power-on initialisation sequence.
  virtual int initialize() = 0;

  /// @brief Clear the display contents.
  virtual int clear() = 0;

  /// @brief Move the text cursor to character-grid position (@p row, @p column), 0-based.
  virtual int set_cursor(uint8_t row, uint8_t column) = 0;

  /// @brief Print one character at the current cursor.
  virtual int print_char(char c) = 0;

  /// @brief Print a string from the cursor, wrapping across rows.
  virtual int print_msg(const std::string & msg) = 0;

  /// @brief Put the display into a safe shutdown state.
  virtual int stop() = 0;

  /// @brief Number of text rows the display can show.
  virtual uint8_t get_rows() const = 0;

  /// @brief Number of text columns the display can show.
  virtual uint8_t get_columns() const = 0;
};

}  // namespace linux_i2c_devices
