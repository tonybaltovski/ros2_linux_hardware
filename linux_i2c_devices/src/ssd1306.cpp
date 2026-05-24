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

#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

#include "linux_i2c_devices/ssd1306.hpp"
#include "rclcpp/logging.hpp"

namespace linux_i2c_devices
{
namespace
{
std::shared_ptr<linux_i2c_interface::I2cInterface> require_iface(
  std::shared_ptr<linux_i2c_interface::I2cInterface> iface)
{
  if (!iface)
  {
    throw std::invalid_argument("Ssd1306: i2c_interface must not be null");
  }
  return iface;
}

std::string make_log_name(const std::string & bus, uint8_t addr)
{
  const auto slash = bus.find_last_of('/');
  const std::string base = slash == std::string::npos ? bus : bus.substr(slash + 1);
  char buf[8];
  std::snprintf(buf, sizeof(buf), "0x%02x", static_cast<int>(addr));
  return "ssd1306." + base + "." + buf;
}
rclcpp::Logger logger(const std::string & name) { return rclcpp::get_logger(name); }
}  // namespace

// Basic 5×7 font for printable ASCII characters 0x20–0x7E for front.
// Each character is 5 bytes, one per column, with bit 0 at the top.
// clang-format off
static constexpr uint8_t FONT_5X7[][SSD1306_FONT_WIDTH] = {
  {0x00, 0x00, 0x00, 0x00, 0x00},  // ' '
  {0x00, 0x00, 0x5F, 0x00, 0x00},  // '!'
  {0x00, 0x07, 0x00, 0x07, 0x00},  // '"'
  {0x14, 0x7F, 0x14, 0x7F, 0x14},  // '#'
  {0x24, 0x2A, 0x7F, 0x2A, 0x12},  // '$'
  {0x23, 0x13, 0x08, 0x64, 0x62},  // '%'
  {0x36, 0x49, 0x55, 0x22, 0x50},  // '&'
  {0x00, 0x05, 0x03, 0x00, 0x00},  // '''
  {0x00, 0x1C, 0x22, 0x41, 0x00},  // '('
  {0x00, 0x41, 0x22, 0x1C, 0x00},  // ')'
  {0x08, 0x2A, 0x1C, 0x2A, 0x08},  // '*'
  {0x08, 0x08, 0x3E, 0x08, 0x08},  // '+'
  {0x00, 0x50, 0x30, 0x00, 0x00},  // ','
  {0x08, 0x08, 0x08, 0x08, 0x08},  // '-'
  {0x00, 0x60, 0x60, 0x00, 0x00},  // '.'
  {0x20, 0x10, 0x08, 0x04, 0x02},  // '/'
  {0x3E, 0x51, 0x49, 0x45, 0x3E},  // '0'
  {0x00, 0x42, 0x7F, 0x40, 0x00},  // '1'
  {0x42, 0x61, 0x51, 0x49, 0x46},  // '2'
  {0x21, 0x41, 0x45, 0x4B, 0x31},  // '3'
  {0x18, 0x14, 0x12, 0x7F, 0x10},  // '4'
  {0x27, 0x45, 0x45, 0x45, 0x39},  // '5'
  {0x3C, 0x4A, 0x49, 0x49, 0x30},  // '6'
  {0x01, 0x71, 0x09, 0x05, 0x03},  // '7'
  {0x36, 0x49, 0x49, 0x49, 0x36},  // '8'
  {0x06, 0x49, 0x49, 0x29, 0x1E},  // '9'
  {0x00, 0x36, 0x36, 0x00, 0x00},  // ':'
  {0x00, 0x56, 0x36, 0x00, 0x00},  // ';'
  {0x00, 0x08, 0x14, 0x22, 0x41},  // '<'
  {0x14, 0x14, 0x14, 0x14, 0x14},  // '='
  {0x41, 0x22, 0x14, 0x08, 0x00},  // '>'
  {0x02, 0x01, 0x51, 0x09, 0x06},  // '?'
  {0x32, 0x49, 0x79, 0x41, 0x3E},  // '@'
  {0x7E, 0x11, 0x11, 0x11, 0x7E},  // 'A'
  {0x7F, 0x49, 0x49, 0x49, 0x36},  // 'B'
  {0x3E, 0x41, 0x41, 0x41, 0x22},  // 'C'
  {0x7F, 0x41, 0x41, 0x22, 0x1C},  // 'D'
  {0x7F, 0x49, 0x49, 0x49, 0x41},  // 'E'
  {0x7F, 0x09, 0x09, 0x01, 0x01},  // 'F'
  {0x3E, 0x41, 0x41, 0x51, 0x32},  // 'G'
  {0x7F, 0x08, 0x08, 0x08, 0x7F},  // 'H'
  {0x00, 0x41, 0x7F, 0x41, 0x00},  // 'I'
  {0x20, 0x40, 0x41, 0x3F, 0x01},  // 'J'
  {0x7F, 0x08, 0x14, 0x22, 0x41},  // 'K'
  {0x7F, 0x40, 0x40, 0x40, 0x40},  // 'L'
  {0x7F, 0x02, 0x04, 0x02, 0x7F},  // 'M'
  {0x7F, 0x04, 0x08, 0x10, 0x7F},  // 'N'
  {0x3E, 0x41, 0x41, 0x41, 0x3E},  // 'O'
  {0x7F, 0x09, 0x09, 0x09, 0x06},  // 'P'
  {0x3E, 0x41, 0x51, 0x21, 0x5E},  // 'Q'
  {0x7F, 0x09, 0x19, 0x29, 0x46},  // 'R'
  {0x46, 0x49, 0x49, 0x49, 0x31},  // 'S'
  {0x01, 0x01, 0x7F, 0x01, 0x01},  // 'T'
  {0x3F, 0x40, 0x40, 0x40, 0x3F},  // 'U'
  {0x1F, 0x20, 0x40, 0x20, 0x1F},  // 'V'
  {0x7F, 0x20, 0x18, 0x20, 0x7F},  // 'W'
  {0x63, 0x14, 0x08, 0x14, 0x63},  // 'X'
  {0x03, 0x04, 0x78, 0x04, 0x03},  // 'Y'
  {0x61, 0x51, 0x49, 0x45, 0x43},  // 'Z'
  {0x00, 0x00, 0x7F, 0x41, 0x41},  // '['
  {0x02, 0x04, 0x08, 0x10, 0x20},  // '\'
  {0x41, 0x41, 0x7F, 0x00, 0x00},  // ']'
  {0x04, 0x02, 0x01, 0x02, 0x04},  // '^'
  {0x40, 0x40, 0x40, 0x40, 0x40},  // '_'
  {0x00, 0x01, 0x02, 0x04, 0x00},  // '`'
  {0x20, 0x54, 0x54, 0x54, 0x78},  // 'a'
  {0x7F, 0x48, 0x44, 0x44, 0x38},  // 'b'
  {0x38, 0x44, 0x44, 0x44, 0x20},  // 'c'
  {0x38, 0x44, 0x44, 0x48, 0x7F},  // 'd'
  {0x38, 0x54, 0x54, 0x54, 0x18},  // 'e'
  {0x08, 0x7E, 0x09, 0x01, 0x02},  // 'f'
  {0x08, 0x14, 0x54, 0x54, 0x3C},  // 'g'
  {0x7F, 0x08, 0x04, 0x04, 0x78},  // 'h'
  {0x00, 0x44, 0x7D, 0x40, 0x00},  // 'i'
  {0x20, 0x40, 0x44, 0x3D, 0x00},  // 'j'
  {0x00, 0x7F, 0x10, 0x28, 0x44},  // 'k'
  {0x00, 0x41, 0x7F, 0x40, 0x00},  // 'l'
  {0x7C, 0x04, 0x18, 0x04, 0x78},  // 'm'
  {0x7C, 0x08, 0x04, 0x04, 0x78},  // 'n'
  {0x38, 0x44, 0x44, 0x44, 0x38},  // 'o'
  {0x7C, 0x14, 0x14, 0x14, 0x08},  // 'p'
  {0x08, 0x14, 0x14, 0x18, 0x7C},  // 'q'
  {0x7C, 0x08, 0x04, 0x04, 0x08},  // 'r'
  {0x48, 0x54, 0x54, 0x54, 0x20},  // 's'
  {0x04, 0x3F, 0x44, 0x40, 0x20},  // 't'
  {0x3C, 0x40, 0x40, 0x20, 0x7C},  // 'u'
  {0x1C, 0x20, 0x40, 0x20, 0x1C},  // 'v'
  {0x3C, 0x40, 0x30, 0x40, 0x3C},  // 'w'
  {0x44, 0x28, 0x10, 0x28, 0x44},  // 'x'
  {0x0C, 0x50, 0x50, 0x50, 0x3C},  // 'y'
  {0x44, 0x64, 0x54, 0x4C, 0x44},  // 'z'
  {0x00, 0x08, 0x36, 0x41, 0x00},  // '{'
  {0x00, 0x00, 0x7F, 0x00, 0x00},  // '|'
  {0x00, 0x41, 0x36, 0x08, 0x00},  // '}'
  {0x08, 0x08, 0x2A, 0x1C, 0x08},  // '~'
};
// clang-format on

Ssd1306::Ssd1306(
  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id)
: i2c_interface_(require_iface(std::move(i2c_interface))),
  device_id_(device_id),
  log_name_(make_log_name(i2c_interface_->bus_name(), device_id))
{
}

int Ssd1306::send_command(Transaction & i2c_transaction, uint8_t cmd)
{
  uint8_t buf[2] = {SSD1306_CONTROL_COMMAND, cmd};
  return i2c_transaction.write_raw(buf, sizeof(buf));
}

int Ssd1306::send_commands(Transaction & i2c_transaction, const uint8_t * cmds, uint16_t count)
{
  for (uint16_t i = 0; i < count; ++i)
  {
    if (send_command(i2c_transaction, cmds[i]) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to send command at index %u", __func__, i);
      return -1;
    }
  }
  return 0;
}

int Ssd1306::send_data(Transaction & i2c_transaction, const uint8_t * data, uint16_t size)
{
  // Prepend the data control byte so everything goes in a single I2C transaction.
  // Send in chunks to respect I2C buffer limits (typically 32 bytes on many adapters).
  static constexpr uint16_t CHUNK_SIZE = 16;
  uint8_t buf[CHUNK_SIZE + 1];
  buf[0] = SSD1306_CONTROL_DATA;

  uint16_t offset = 0;
  while (offset < size)
  {
    uint16_t remaining = size - offset;
    uint16_t len = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;
    std::memcpy(&buf[1], data + offset, len);
    if (i2c_transaction.write_raw(buf, len + 1) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to write data at offset %u", __func__, offset);
      return -1;
    }
    offset += len;
  }
  return 0;
}

int Ssd1306::initialize()
{
  if (initialized_)
  {
    return 0;
  }

  RCLCPP_INFO(logger(log_name_), "%s: Starting initialization", __func__);

  auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
  if (!i2c_transaction.ok())
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to start transaction", __func__);
    return -1;
  }

  // Initialisation command sequence for a 128×64 SSD1306 with internal charge pump.
  const uint8_t init_cmds[] = {
    SSD1306_DISPLAY_OFF,
    SSD1306_SET_DISPLAY_CLOCK_DIV,
    0x80,
    SSD1306_SET_MULTIPLEX,
    0x3F,
    SSD1306_SET_DISPLAY_OFFSET,
    0x00,
    SSD1306_SET_START_LINE | 0x00,
    SSD1306_CHARGE_PUMP,
    0x14,
    SSD1306_SET_MEMORY_MODE,
    0x00,
    SSD1306_SEG_REMAP | 0x01,
    SSD1306_COM_SCAN_DEC,
    SSD1306_SET_COM_PINS,
    0x12,
    SSD1306_SET_CONTRAST,
    0xCF,
    SSD1306_SET_PRECHARGE,
    0xF1,
    SSD1306_SET_VCOM_DETECT,
    0x40,
    SSD1306_DISPLAY_ALL_ON_RESUME,
    SSD1306_NORMAL_DISPLAY,
    SSD1306_DEACTIVATE_SCROLL,
    SSD1306_DISPLAY_ON,
  };

  if (send_commands(i2c_transaction, init_cmds, sizeof(init_cmds)) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Initialization failed", __func__);
    return -1;
  }

  std::memset(buffer_, 0, SSD1306_BUFFER_SIZE);
  if (display(i2c_transaction) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to flush initial framebuffer", __func__);
    return -1;
  }

  initialized_ = true;
  RCLCPP_INFO(logger(log_name_), "%s: Initialization done", __func__);
  return 0;
}

int Ssd1306::clear()
{
  std::memset(buffer_, 0, SSD1306_BUFFER_SIZE);
  cursor_row_ = 0;
  cursor_col_ = 0;
  return 0;
}

int Ssd1306::display(Transaction & i2c_transaction)
{
  // Reset the column and page pointers to the beginning of GDDRAM.
  const uint8_t addressing_cmds[] = {
    SSD1306_SET_COLUMN_ADDR, 0, static_cast<uint8_t>(SSD1306_WIDTH - 1),
    SSD1306_SET_PAGE_ADDR,   0, 7,
  };
  if (send_commands(i2c_transaction, addressing_cmds, sizeof(addressing_cmds)) < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to set GDDRAM window", __func__);
    return -1;
  }

  return send_data(i2c_transaction, buffer_, SSD1306_BUFFER_SIZE);
}

int Ssd1306::display()
{
  // The GDDRAM cursor lives in the chip and auto-advances across separate I2C
  // transactions, so we release the bus between chunks: other drivers on the
  // same bus can interleave instead of waiting for the full ~1 KB flush.
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to start window transaction", __func__);
      return -1;
    }
    const uint8_t addressing_cmds[] = {
      SSD1306_SET_COLUMN_ADDR, 0, static_cast<uint8_t>(SSD1306_WIDTH - 1),
      SSD1306_SET_PAGE_ADDR,   0, 7,
    };
    if (send_commands(i2c_transaction, addressing_cmds, sizeof(addressing_cmds)) < 0)
    {
      return -1;
    }
  }

  static constexpr uint16_t CHUNK_SIZE = 16;
  uint8_t buf[CHUNK_SIZE + 1];
  buf[0] = SSD1306_CONTROL_DATA;
  for (uint16_t offset = 0; offset < SSD1306_BUFFER_SIZE; offset += CHUNK_SIZE)
  {
    const uint16_t remaining = SSD1306_BUFFER_SIZE - offset;
    const uint16_t len = (remaining < CHUNK_SIZE) ? remaining : CHUNK_SIZE;
    std::memcpy(&buf[1], buffer_ + offset, len);

    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      RCLCPP_ERROR(
        logger(log_name_), "%s: Failed to start data transaction at offset %u", __func__, offset);
      return -1;
    }
    if (i2c_transaction.write_raw(buf, len + 1) < 0)
    {
      RCLCPP_ERROR(logger(log_name_), "%s: Failed to write data at offset %u", __func__, offset);
      return -1;
    }
  }
  return 0;
}

void Ssd1306::set_pixel(uint8_t x, uint8_t y, bool on)
{
  if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT)
  {
    return;
  }

  uint16_t index = x + (y / 8) * SSD1306_WIDTH;
  uint8_t bit = 1 << (y & 7);

  if (on)
  {
    buffer_[index] |= bit;
  }
  else
  {
    buffer_[index] &= ~bit;
  }
}

void Ssd1306::draw_char(uint8_t x, uint8_t y, char c)
{
  if (c < 0x20 || c > 0x7E)
  {
    c = '?';
  }

  const uint8_t * glyph = FONT_5X7[c - 0x20];
  for (uint8_t col = 0; col < SSD1306_FONT_WIDTH; ++col)
  {
    for (uint8_t row = 0; row < 7; ++row)
    {
      set_pixel(x + col, y + row, (glyph[col] >> row) & 1);
    }
  }
}

int Ssd1306::set_cursor(uint8_t row, uint8_t column)
{
  if (row >= SSD1306_TEXT_ROWS)
  {
    RCLCPP_ERROR(
      logger(log_name_), "%s: Row %d out of range (max %d)", __func__, static_cast<int>(row),
      static_cast<int>(SSD1306_TEXT_ROWS - 1));
    return -1;
  }
  if (column >= SSD1306_TEXT_COLS)
  {
    RCLCPP_ERROR(
      logger(log_name_), "%s: Column %d out of range (max %d)", __func__, static_cast<int>(column),
      static_cast<int>(SSD1306_TEXT_COLS - 1));
    return -1;
  }
  cursor_row_ = row;
  cursor_col_ = column;
  return 0;
}

int Ssd1306::print_char(char c)
{
  uint8_t px = cursor_col_ * (SSD1306_FONT_WIDTH + SSD1306_FONT_SPACING);
  uint8_t py = cursor_row_ * SSD1306_FONT_HEIGHT;
  draw_char(px, py, c);

  ++cursor_col_;
  if (cursor_col_ >= SSD1306_TEXT_COLS)
  {
    cursor_col_ = 0;
    ++cursor_row_;
    if (cursor_row_ >= SSD1306_TEXT_ROWS)
    {
      cursor_row_ = 0;
    }
  }
  return 0;
}

int Ssd1306::print_msg(const std::string & msg)
{
  for (char c : msg)
  {
    print_char(c);
  }
  if (display() < 0)
  {
    RCLCPP_ERROR(logger(log_name_), "%s: Failed to flush framebuffer", __func__);
    return -1;
  }
  return 0;
}

int Ssd1306::stop()
{
  int ret = 0;
  if (initialized_)
  {
    auto i2c_transaction = i2c_interface_->begin_transaction(device_id_);
    if (!i2c_transaction.ok())
    {
      ret = -1;
    }
    else
    {
      clear();
      if (display(i2c_transaction) < 0)
      {
        RCLCPP_ERROR(logger(log_name_), "%s: Failed to flush framebuffer", __func__);
        ret = -1;
      }
      if (send_command(i2c_transaction, SSD1306_DISPLAY_OFF) < 0)
      {
        RCLCPP_ERROR(logger(log_name_), "%s: Failed to send display-off command", __func__);
        ret = -1;
      }
    }
  }
  return ret;
}

}  // namespace linux_i2c_devices
