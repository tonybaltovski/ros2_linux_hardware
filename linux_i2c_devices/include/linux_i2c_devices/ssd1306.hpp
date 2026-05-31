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
 * @file ssd1306.hpp
 * @brief Driver for the SSD1306 128×64 OLED I2C display.
 */

#pragma once

#include <unistd.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "linux_i2c_devices/screen.hpp"
#include "linux_i2c_interface/i2c_interface.hpp"

namespace linux_i2c_devices
{

/** @name SSD1306 default I2C address */
///@{
constexpr uint8_t SSD1306_DEFAULT_ADDRESS = 0x3C;
///@}

/** @name Display dimensions */
///@{
constexpr uint8_t SSD1306_WIDTH = 128;
constexpr uint8_t SSD1306_HEIGHT = 64;
constexpr uint16_t SSD1306_BUFFER_SIZE = SSD1306_WIDTH * SSD1306_HEIGHT / 8;  // 1024
///@}

/** @name I2C control bytes */
///@{
constexpr uint8_t SSD1306_CONTROL_COMMAND = 0x00;
constexpr uint8_t SSD1306_CONTROL_DATA = 0x40;
///@}

/** @name Fundamental commands */
///@{
constexpr uint8_t SSD1306_SET_CONTRAST = 0x81;
constexpr uint8_t SSD1306_DISPLAY_ALL_ON_RESUME = 0xA4;
constexpr uint8_t SSD1306_NORMAL_DISPLAY = 0xA6;
constexpr uint8_t SSD1306_DISPLAY_OFF = 0xAE;
constexpr uint8_t SSD1306_DISPLAY_ON = 0xAF;
///@}

/** @name Scrolling commands */
///@{
constexpr uint8_t SSD1306_DEACTIVATE_SCROLL = 0x2E;
///@}

/** @name Addressing-mode commands */
///@{
constexpr uint8_t SSD1306_SET_MEMORY_MODE = 0x20;
constexpr uint8_t SSD1306_SET_COLUMN_ADDR = 0x21;
constexpr uint8_t SSD1306_SET_PAGE_ADDR = 0x22;
///@}

/** @name Hardware-configuration commands */
///@{
constexpr uint8_t SSD1306_SET_START_LINE = 0x40;
constexpr uint8_t SSD1306_SEG_REMAP = 0xA0;
constexpr uint8_t SSD1306_SET_MULTIPLEX = 0xA8;
constexpr uint8_t SSD1306_COM_SCAN_DEC = 0xC8;
constexpr uint8_t SSD1306_SET_DISPLAY_OFFSET = 0xD3;
constexpr uint8_t SSD1306_SET_COM_PINS = 0xDA;
///@}

/** @name Timing and driving-scheme commands */
///@{
constexpr uint8_t SSD1306_SET_DISPLAY_CLOCK_DIV = 0xD5;
constexpr uint8_t SSD1306_SET_PRECHARGE = 0xD9;
constexpr uint8_t SSD1306_SET_VCOM_DETECT = 0xDB;
///@}

/** @name Charge-pump command */
///@{
constexpr uint8_t SSD1306_CHARGE_PUMP = 0x8D;
///@}

/** @name Font metrics */
///@{
constexpr uint8_t SSD1306_FONT_WIDTH = 5;
constexpr uint8_t SSD1306_FONT_HEIGHT = 8;
constexpr uint8_t SSD1306_FONT_SPACING = 1;
constexpr uint8_t SSD1306_TEXT_COLS = SSD1306_WIDTH / (SSD1306_FONT_WIDTH + SSD1306_FONT_SPACING);
constexpr uint8_t SSD1306_TEXT_ROWS = SSD1306_HEIGHT / SSD1306_FONT_HEIGHT;
///@}

/**
 * @class Ssd1306
 * @brief Driver for the SSD1306 128×64 OLED display over I2C.
 *
 * Maintains an in-memory framebuffer that is flushed to the display
 * with display().  Text is rendered using a built-in 5×7 pixel font.
 *
 * Typical usage:
 * @code
 * auto i2c = linux_i2c_interface::I2cInterface::get_shared(1);
 * linux_i2c_devices::Ssd1306 oled(i2c, 0x3C);
 * oled.initialize();
 * oled.clear();
 * oled.print_msg("Hello, world!");
 * oled.display();
 * @endcode
 */
class Ssd1306 : public Screen
{
public:
  /**
   * @brief Construct an Ssd1306 driver.
   * @param i2c_interface Shared I2C bus interface (must not be null).
   * @param device_id 7-bit I2C address (typically 0x3C or 0x3D).
   * @throws std::invalid_argument if @p i2c_interface is null.
   */
  Ssd1306(std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id);

  /// @copydoc Screen::initialize
  int initialize() override;
  /// @copydoc Screen::clear
  int clear() override;
  /// @copydoc Screen::set_cursor
  int set_cursor(uint8_t row, uint8_t column) override;
  /// @copydoc Screen::print_char
  int print_char(char c) override;
  /// @copydoc Screen::print_msg
  int print_msg(const std::string & msg) override;
  /// @copydoc Screen::stop
  int stop() override;
  /// @copydoc Screen::get_rows
  uint8_t get_rows() const override { return SSD1306_TEXT_ROWS; }
  /// @copydoc Screen::get_columns
  uint8_t get_columns() const override { return SSD1306_TEXT_COLS; }

  /**
   * @brief Flush the in-memory framebuffer to the OLED.
   *
   * Sets the GDDRAM window in a short transaction, then streams the 1024-byte
   * buffer as ~16-byte chunks, each in its own transaction.  Other drivers on
   * the same bus may interleave between chunks.
   *
   * @return 0 on success, -1 on transfer failure (errno set).
   */
  int display();

  /**
   * @brief Set or clear a single pixel in the framebuffer (not flushed until display()).
   * @param x Column (0..127).  Out-of-range coordinates are ignored.
   * @param y Row (0..63).      Out-of-range coordinates are ignored.
   * @param on True to light the pixel, false to clear.
   */
  void set_pixel(uint8_t x, uint8_t y, bool on = true);

  /**
   * @brief Draw a single 5x7 glyph into the framebuffer at pixel (@p x, @p y).
   * @param x Left edge column (top of the cell).
   * @param y Top edge row.
   * @param c ASCII character in 0x20..0x7E; out-of-range chars render blank.
   */
  void draw_char(uint8_t x, uint8_t y, char c);

private:
  using Transaction = linux_i2c_interface::I2cInterface::Transaction;

  /// @brief Send a single command byte (within an open transaction).
  int send_command(Transaction & i2c_transaction, uint8_t cmd);

  /// @brief Send a sequence of command bytes (within an open transaction).
  int send_commands(Transaction & i2c_transaction, const uint8_t * cmds, uint16_t count);

  /// @brief Send a block of pixel data to GDDRAM (within an open transaction).
  int send_data(Transaction & i2c_transaction, const uint8_t * data, uint16_t size);

  /// @brief Flush the framebuffer using an already-open transaction.
  int display(Transaction & i2c_transaction);

  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface_;
  uint8_t device_id_;
  std::string log_name_;
  uint8_t buffer_[SSD1306_BUFFER_SIZE]{};  ///< In-memory framebuffer (1024 bytes).
  uint8_t cursor_row_{0};
  uint8_t cursor_col_{0};
  bool initialized_{false};
  // Serialises public entry points so concurrent callers cannot corrupt the
  // framebuffer or interleave a partially-written GDDRAM window.  Recursive
  // because public methods chain (e.g. print_msg -> print_char -> draw_char).
  mutable std::recursive_mutex device_mutex_;
};

}  // namespace linux_i2c_devices
