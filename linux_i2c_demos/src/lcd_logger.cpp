#include <memory>
#include <string>

#include <unistd.h>

#include "linux_i2c_interface/i2c_interface.hpp"
#include "linux_i2c_devices/lcm1602.hpp"

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
    lcd_(std::make_shared<linux_i2c_interface::I2cInterface>(1), 0x27, 4, 20)
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
  linux_i2c_devices::Lcm1602 lcd_;
  uint8_t min_logger_level_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LcdScreenLogger>(rcl_interfaces::msg::Log::INFO));
  rclcpp::shutdown();
  return 0;
}
