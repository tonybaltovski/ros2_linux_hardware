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
 * auto i2c = std::make_shared<linux_i2c_interface::I2cInterface>(1);
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
   * @param i2c_interface Shared I2C bus interface.
   * @param device_id 7-bit I2C address (typically 0x3C or 0x3D).
   */
  Ssd1306(std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id);

  /**
   * @brief Run the power-on initialisation sequence.
   * @return 0 on success, negative on failure.
   */
  int8_t initialize() override;

  /**
   * @brief Clear the framebuffer (call display() to update the screen).
   * @return 0 on success, negative on failure.
   */
  int8_t clear() override;

  /**
   * @brief Flush the framebuffer to the OLED.
   * @return 0 on success, -1 on failure.
   */
  int display();

  /**
   * @brief Set or clear a single pixel in the framebuffer.
   * @param x  Column (0–127).
   * @param y  Row (0–63).
   * @param on true to turn the pixel on, false to turn it off.
   */
  void set_pixel(uint8_t x, uint8_t y, bool on = true);

  /**
   * @brief Draw a single character at a pixel position.
   * @param x Column pixel coordinate.
   * @param y Row pixel coordinate (top of the character cell).
   * @param c ASCII character to draw.
   */
  void draw_char(uint8_t x, uint8_t y, char c);

  /**
   * @brief Move the text cursor to a character-grid position.
   * @param row    Row index (0-based, in character rows).
   * @param column Column index (0-based, in character columns).
   * @return 0 on success, -1 if row or column is out of range.
   */
  int8_t set_cursor(uint8_t row, uint8_t column) override;

  /**
   * @brief Print a single character at the current text cursor and advance it.
   * @param c Character to print.
   * @return 0 on success, negative on failure.
   */
  int8_t print_char(char c) override;

  /**
   * @brief Print a string starting at the current cursor, wrapping across rows.
   *
   * Calls display() automatically after rendering.
   *
   * @param msg The message to display.
   * @return 0 on success, negative on failure.
   */
  int8_t print_msg(const std::string & msg) override;

  /**
   * @brief Turn the display off and close the I2C bus.
   * @return 0 on success, negative on failure.
   */
  int8_t stop() override;

  /** @brief Number of text rows the display can show. */
  uint8_t get_rows() const override { return SSD1306_TEXT_ROWS; }

  /** @brief Number of text columns the display can show. */
  uint8_t get_columns() const override { return SSD1306_TEXT_COLS; }

private:
  /**
   * @brief Send a single command byte to the SSD1306.
   * @param cmd Command byte.
   * @return 0 on success, -1 on failure.
   */
  int send_command(uint8_t cmd);

  /**
   * @brief Send a sequence of command bytes.
   * @param cmds Pointer to the command byte array.
   * @param count Number of bytes to send.
   * @return 0 on success, -1 on failure.
   */
  int send_commands(const uint8_t * cmds, uint16_t count);

  /**
   * @brief Send a block of pixel data to the SSD1306 GDDRAM.
   * @param data Pointer to the data bytes.
   * @param size Number of bytes to send.
   * @return 0 on success, -1 on failure.
   */
  int send_data(const uint8_t * data, uint16_t size);

  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface_;  ///< Shared I2C bus.
  uint8_t device_id_;           ///< 7-bit I2C slave address.
  uint8_t buffer_[SSD1306_BUFFER_SIZE]{};  ///< In-memory framebuffer (1024 bytes).
  uint8_t cursor_row_{0};       ///< Current text cursor row.
  uint8_t cursor_col_{0};       ///< Current text cursor column.
  bool initialized_{false};     ///< Whether initialize() has completed.
};

}  // namespace linux_i2c_devices
