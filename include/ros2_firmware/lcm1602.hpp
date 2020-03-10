#pragma once

#include <unistd.h>

#include "ros2_firmware/i2c_interface.hpp"

#define LCM1602_CLEARDISPLAY   0x01
#define LCM1602_RETURNHOME     0x02
#define LCM1602_ENTRYMODESET   0x04
#define LCM1602_DISPLAYCONTROL 0x08
#define LCM1602_CURSORSHIFT    0x10
#define LCM1602_FUNCTIONSET    0x20
#define LCM1602_SETCGRAMADDR   0x40
#define LCM1602_SETDDRAMADDR   0x80
#define LCM1602_BACKLIGHT_ON   0x08
#define LCM1602_BACKLIGHT_OFF  0x00

#define LCM1602_ENTRYSHIFTINCREMENT 0x01
#define LCM1602_ENTRYLEFT           0x02

#define LCM1602_BLINKON   0x01
#define LCM1602_CURSORON  0x02
#define LCM1602_DISPLAYON 0x04

#define LCM1602_4BITMODE 0x00
#define LCM1602_8BITMODE 0x10

#define LCM1602_1LINE 0x00
#define LCM1602_2LINE 0x08

#define LCM1602_CMD   0x00
#define LCM1602_DATA  0x01
#define LCM1602_4BITS 0x02

#define LCM1602_RS 0x01
#define LCM1602_RW 0x02
#define LCM1602_EN 0x04

#define LCM1602_DELAY_100_US   100
#define LCM1602_DELAY_4000_US  4000
#define LCM1602_DELAY_40000_US 40000

namespace ros2_firmware
{

class Lcm1602
{
public:
  Lcm1602(I2cInterface& i2c_interface, uint8_t device_id,
          uint8_t rows, uint8_t columns);
  void clear();
  void home();
  void set_cursor(uint8_t row, uint8_t column);
  void print_msg(const std::string& msg);
  void print_char(uint8_t i);
  int initialize();
  void stop();
  void command(uint8_t value, uint32_t dely_us = 0);
  void write(uint8_t value, uint8_t mode);
  void write_4bits(uint8_t value);
  void send(uint8_t value, uint32_t dely_us = 0);
  void pulse_enable(uint8_t value);

private:
  I2cInterface i2c_interface_;
  uint8_t device_id_;
  uint8_t rows_, columns_;
  uint8_t blacklight_;
};

}  // namespace ros2_firmware
