#include <memory>
#include <string>

#include <unistd.h>

#include "ros2_firmware/i2c_interface.hpp"
#include "ros2_firmware/lcm1602.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class LcdScreenLogger : public rclcpp::Node
{
public:
  LcdScreenLogger(uint8_t min_logger_level)
  : Node("lcd_loger"),
    min_logger_level_(min_logger_level),
    i2c_interface_(1),
    lcd_(i2c_interface_, 0x27, 4, 20)
  {
    sub_log_ = this->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", 100, std::bind(&LcdScreenLogger::logCallback, this, _1));
    lcd_.initialize();
    lcd_.clear();
  }

  ~LcdScreenLogger()
  {
    lcd_.stop();
  }

private:
  const std::string log_to_string(const rcl_interfaces::msg::Log::SharedPtr log_msg)
  {
    std::stringstream ss;
    switch (log_msg->level)
    {
      case rcl_interfaces::msg::Log::DEBUG:
        ss << "[DEBUG] ";
        break;
      case rcl_interfaces::msg::Log::INFO:
        ss << "[INFO] ";
        break;
      case rcl_interfaces::msg::Log::WARN:
        ss << "[WARN] ";
        break;
      case rcl_interfaces::msg::Log::ERROR:
        ss << "[ERROR] ";
        break;
      case rcl_interfaces::msg::Log::FATAL:
        ss << "[FATAL] ";
        break;
      default:
        ss << log_msg->level << " ";
    }
    ss << "[" << log_msg->name << "] ";

    ss << log_msg->msg;

    return ss.str();
  }
  void logCallback(const rcl_interfaces::msg::Log::SharedPtr log_msg)
  {
    lcd_.clear();
    lcd_.set_cursor(0, 0);
    if (log_msg->level >= min_logger_level_)
    {
      lcd_.print_msg(log_to_string(log_msg));
    }
  }

  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr sub_log_;
  ros2_firmware::I2cInterface i2c_interface_;
  ros2_firmware::Lcm1602 lcd_;
  uint8_t min_logger_level_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LcdScreenLogger>(rcl_interfaces::msg::Log::INFO));
  rclcpp::shutdown();
  return 0;
}
