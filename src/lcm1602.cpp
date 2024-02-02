#include <cstring>
#include <iostream>

#include "ros2_firmware/lcm1602.hpp"

namespace ros2_firmware
{

Lcm1602::Lcm1602(std::shared_ptr<ros2_firmware::I2cInterface> i2c_interface, uint8_t device_id, uint8_t rows, uint8_t columns) :
  i2c_interface_(i2c_interface),
  device_id_(device_id),
  rows_(rows),
  columns_(columns),
  blacklight_(LCM1602_BACKLIGHT_OFF),
  initialized_(false)
{
  i2c_interface_->open_bus();
}

void Lcm1602::send(uint8_t value, uint32_t dely_us)
{
  uint8_t buffer = value | blacklight_;
  if (i2c_interface_->is_connected())
    i2c_interface_->write_to_bus(device_id_, buffer);
  else
    std::cerr << __PRETTY_FUNCTION__ << ": i2c interface not connected." << std::endl;
  usleep(dely_us);
}

void Lcm1602::pulse_enable(uint8_t value)
{
  send(value | LCM1602_EN, 1);
  send(value & ~LCM1602_EN, 50);
}

void Lcm1602::write_4bits(uint8_t value)
{
  send(value);
  pulse_enable(value);
}

void Lcm1602::write(uint8_t value, uint8_t mode)
{
  write_4bits((value & 0xF0) | mode);
  write_4bits(((value << 4) & 0xF0) | mode);
}

void Lcm1602::command(uint8_t value, uint32_t dely_us)
{
  write(value, LCM1602_CMD);
  usleep(dely_us);
}

void Lcm1602::initialize()
{
  if (!initialized_)
  {
    // Sleep to let the device initialize.
    std::cout << __PRETTY_FUNCTION__ << ": Starting initialization" << std::endl;
    usleep(400000);
    blacklight_ = LCM1602_BACKLIGHT_ON;
    send(blacklight_, 2000);

    write_4bits(0x03 << 4);
    usleep(LCM1602_DELAY_4000_US);
    write_4bits(0x30);
    usleep(LCM1602_DELAY_4000_US);
    write_4bits(0x30);
    usleep(LCM1602_DELAY_100_US);
    write_4bits(0x20);

    command(LCM1602_FUNCTIONSET    | LCM1602_2LINE);
    command(LCM1602_DISPLAYCONTROL | LCM1602_DISPLAYON);
    command(LCM1602_ENTRYMODESET   | LCM1602_ENTRYLEFT);
    usleep(LCM1602_DELAY_40000_US);
    clear();
    home();
    std::cout << __PRETTY_FUNCTION__ << ": Initialization done" << std::endl;
    initialized_ = true;
  }
}

void Lcm1602::stop()
{
  clear();
  blacklight_ = LCM1602_BACKLIGHT_OFF;
  send(blacklight_, LCM1602_DELAY_100_US);
  if (i2c_interface_->is_connected())
    i2c_interface_->close_bus();
}

void Lcm1602::clear()
{
  command(LCM1602_CLEARDISPLAY);
  usleep(LCM1602_DELAY_4000_US);
}

void Lcm1602::home()
{
  command(LCM1602_RETURNHOME);
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
  command(LCM1602_SETDDRAMADDR | ((column % columns_) + offset));
}

void Lcm1602::print_char(char i)
{
  write(i, LCM1602_RS);
}

void Lcm1602::print_msg(const std::string& msg)
{
  for (size_t i = 0; i < msg.length(); i++)
  {
    if (i >= (3 * columns_))
    {
      set_cursor(3, (i - 2 * columns_));
    }
    else if (i >= (2 * columns_))
    {
      set_cursor(2, i - columns_);
    }
    else if (i >= (1 * columns_))
    {
      set_cursor(1, i - columns_);
    }
    else
    {
      set_cursor(0, i);
    }
    print_char(msg[i]);
  }
}

}  // namespace ros2_firmware
