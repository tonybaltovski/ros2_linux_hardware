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

#include <cstring>
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
  blacklight_(LCM1602_BACKLIGHT_OFF),
  initialized_(false)
{
  i2c_interface_->open_bus();
}

void Lcm1602::send(uint8_t value, uint32_t delay_us)
{
  uint8_t buffer = value | blacklight_;
  if (i2c_interface_->is_connected())
  {
    i2c_interface_->write_to_bus(device_id_, buffer);
  }
  else
  {
    std::cerr << __PRETTY_FUNCTION__ << ": i2c interface not connected." << std::endl;
  }
  usleep(delay_us);
}

void Lcm1602::pulse_enable(uint8_t value)
{
  this->send(value | LCM1602_EN, 1);
  this->send(value & ~LCM1602_EN, 50);
}

void Lcm1602::write_4bits(uint8_t value)
{
  this->send(value);
  this->pulse_enable(value);
}

void Lcm1602::write(uint8_t value, uint8_t mode)
{
  this->write_4bits((value & 0xF0) | mode);
  this->write_4bits(((value << 4) & 0xF0) | mode);
}

void Lcm1602::command(uint8_t value, uint32_t delay_us)
{
  this->write(value, LCM1602_CMD);
  usleep(delay_us);
}

void Lcm1602::initialize()
{
  if (!initialized_)
  {
    // Sleep to let the device initialize.
    std::cout << __PRETTY_FUNCTION__ << ": Starting initialization" << std::endl;
    usleep(400000);
    blacklight_ = LCM1602_BACKLIGHT_ON;
    this->send(blacklight_, 2000);

    this->write_4bits(0x03 << 4);
    usleep(LCM1602_DELAY_4000_US);
    this->write_4bits(0x30);
    usleep(LCM1602_DELAY_4000_US);
    this->write_4bits(0x30);
    usleep(LCM1602_DELAY_100_US);
    this->write_4bits(0x20);

    command(LCM1602_FUNCTIONSET | LCM1602_2LINE);
    command(LCM1602_DISPLAYCONTROL | LCM1602_DISPLAYON);
    command(LCM1602_ENTRYMODESET | LCM1602_ENTRYLEFT);
    usleep(LCM1602_DELAY_40000_US);
    this->clear();
    this->home();
    std::cout << __PRETTY_FUNCTION__ << ": Initialization done" << std::endl;
    initialized_ = true;
  }
}

void Lcm1602::stop()
{
  this->clear();
  blacklight_ = LCM1602_BACKLIGHT_OFF;
  this->send(blacklight_, LCM1602_DELAY_100_US);
  if (i2c_interface_->is_connected())
  {
    i2c_interface_->close_bus();
  }
}

void Lcm1602::clear()
{
  this->command(LCM1602_CLEARDISPLAY);
  usleep(LCM1602_DELAY_4000_US);
}

void Lcm1602::home()
{
  this->command(LCM1602_RETURNHOME);
  usleep(LCM1602_DELAY_4000_US);
}

void Lcm1602::set_cursor(const uint8_t row, const uint8_t column)
{
  uint8_t offset = 0;
  switch (row)
  {
    case 0:
      offset = 0;
      break;
    case 1:
      offset = 64;
      break;
    case 2:
      offset = 20;
      break;
    case 3:
      offset = 84;
      break;
  }
  this->command(LCM1602_SETDDRAMADDR | ((column % columns_) + offset));
}

void Lcm1602::print_char(char i) { this->write(i, LCM1602_RS); }

void Lcm1602::print_msg(const std::string & msg)
{
  for (size_t i = 0; i < msg.length(); i++)
  {
    if (i >= (3 * columns_))
    {
      this->set_cursor(3, (i - 2 * columns_));
    }
    else if (i >= (2 * columns_))
    {
      this->set_cursor(2, i - columns_);
    }
    else if (i >= (1 * columns_))
    {
      this->set_cursor(1, i - columns_);
    }
    else
    {
      this->set_cursor(0, i);
    }
    this->print_char(msg[i]);
  }
}

}  // namespace linux_i2c_devices
