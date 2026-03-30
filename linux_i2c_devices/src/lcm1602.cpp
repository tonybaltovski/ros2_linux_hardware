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

#include <iostream>

#include "linux_i2c_devices/lcm1602.hpp"

namespace linux_i2c_devices
{

Lcm1602::Lcm1602(
  std::shared_ptr<linux_i2c_interface::I2cInterface> i2c_interface, uint8_t device_id, uint8_t rows,
  uint8_t columns)
: i2c_interface_(i2c_interface),
  device_id_(device_id),
  rows_(rows),
  columns_(columns),
  backlight_(LCM1602_BACKLIGHT_OFF),
  initialized_(false)
{
  i2c_interface_->open_bus();
}

int8_t Lcm1602::send(uint8_t value, uint32_t delay_us)
{
  uint8_t buffer = value | backlight_;
  if (i2c_interface_->is_connected())
  {
    if (i2c_interface_->write_to_bus(device_id_, buffer) < 0)
    {
      std::cerr << __PRETTY_FUNCTION__ << ": Failed to write to I2C bus" << std::endl;
      return -1;
    }
  }
  else
  {
    std::cerr << __PRETTY_FUNCTION__ << ": I2C interface not connected" << std::endl;
    return -1;
  }
  usleep(delay_us);
  return 0;
}

int8_t Lcm1602::pulse_enable(uint8_t value)
{
  if (send(value | LCM1602_EN, 1) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to send enable pulse" << std::endl;
    return -1;
  }
  return send(value & ~LCM1602_EN, 50);
}

int8_t Lcm1602::write_4bits(uint8_t value)
{
  if (send(value) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to send nibble" << std::endl;
    return -1;
  }
  return pulse_enable(value);
}

int8_t Lcm1602::write(uint8_t value, uint8_t mode)
{
  if (write_4bits((value & 0xF0) | mode) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to write high nibble" << std::endl;
    return -1;
  }
  return write_4bits(((value << 4) & 0xF0) | mode);
}

int8_t Lcm1602::command(uint8_t value, uint32_t delay_us)
{
  if (write(value, LCM1602_CMD) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to write command 0x"
              << std::hex << static_cast<int>(value) << std::dec << std::endl;
    return -1;
  }
  usleep(delay_us);
  return 0;
}

int8_t Lcm1602::initialize()
{
  if (initialized_)
  {
    return 0;
  }

  std::cout << __PRETTY_FUNCTION__ << ": Starting initialization" << std::endl;
  usleep(400000);
  backlight_ = LCM1602_BACKLIGHT_ON;
  if (send(backlight_, 2000) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to enable backlight" << std::endl;
    return -1;
  }

  if (write_4bits(0x03 << 4) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed during 4-bit init sequence" << std::endl;
    return -1;
  }
  usleep(LCM1602_DELAY_4000_US);
  if (write_4bits(0x30) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed during 4-bit init sequence" << std::endl;
    return -1;
  }
  usleep(LCM1602_DELAY_4000_US);
  if (write_4bits(0x30) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed during 4-bit init sequence" << std::endl;
    return -1;
  }
  usleep(LCM1602_DELAY_100_US);
  if (write_4bits(0x20) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to switch to 4-bit mode" << std::endl;
    return -1;
  }

  if (command(LCM1602_FUNCTIONSET | LCM1602_2LINE) < 0 ||
      command(LCM1602_DISPLAYCONTROL | LCM1602_DISPLAYON) < 0 ||
      command(LCM1602_ENTRYMODESET | LCM1602_ENTRYLEFT) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to configure display" << std::endl;
    return -1;
  }
  usleep(LCM1602_DELAY_40000_US);
  if (clear() < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to clear display" << std::endl;
    return -1;
  }
  if (home() < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to home cursor" << std::endl;
    return -1;
  }
  std::cout << __PRETTY_FUNCTION__ << ": Initialization done" << std::endl;
  initialized_ = true;
  return 0;
}

int8_t Lcm1602::stop()
{
  int8_t ret = 0;
  if (clear() < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to clear display" << std::endl;
    ret = -1;
  }
  backlight_ = LCM1602_BACKLIGHT_OFF;
  if (send(backlight_, LCM1602_DELAY_100_US) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to turn off backlight" << std::endl;
    ret = -1;
  }
  if (i2c_interface_->is_connected())
  {
    i2c_interface_->close_bus();
  }
  return ret;
}

int8_t Lcm1602::clear()
{
  if (command(LCM1602_CLEARDISPLAY) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to clear display" << std::endl;
    return -1;
  }
  usleep(LCM1602_DELAY_4000_US);
  return 0;
}

int8_t Lcm1602::home()
{
  if (command(LCM1602_RETURNHOME) < 0)
  {
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to return home" << std::endl;
    return -1;
  }
  usleep(LCM1602_DELAY_4000_US);
  return 0;
}

int8_t Lcm1602::set_cursor(const uint8_t row, const uint8_t column)
{
  static constexpr uint8_t row_offsets[] = {0, 64, 20, 84};
  uint8_t offset = (row < 4) ? row_offsets[row] : 0;
  return command(LCM1602_SETDDRAMADDR | ((column % columns_) + offset));
}

int8_t Lcm1602::print_char(char c) { return write(c, LCM1602_RS); }

int8_t Lcm1602::print_msg(const std::string & msg)
{
  for (size_t i = 0; i < msg.length(); ++i)
  {
    uint8_t row = static_cast<uint8_t>(i / columns_);
    uint8_t col = static_cast<uint8_t>(i % columns_);
    if (set_cursor(row, col) < 0)
    {
      std::cerr << __PRETTY_FUNCTION__ << ": Failed to set cursor at row "
                << static_cast<int>(row) << " col " << static_cast<int>(col) << std::endl;
      return -1;
    }
    if (print_char(msg[i]) < 0)
    {
      std::cerr << __PRETTY_FUNCTION__ << ": Failed to print character" << std::endl;
      return -1;
    }
  }
  return 0;
}

int8_t Lcm1602::turn_on()
{
  return command(LCM1602_DISPLAYCONTROL | LCM1602_DISPLAYON);
}

int8_t Lcm1602::turn_off()
{
  return command(LCM1602_DISPLAYCONTROL);
}

}  // namespace linux_i2c_devices
