#include <memory>
#include <string>

#include <unistd.h>

#include "ros2_firmware/i2c_interface.hpp"
#include "ros2_firmware/lcm1602.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class LcdScreenSubscriber : public rclcpp::Node
{
public:
  LcdScreenSubscriber()
  : Node("lcd_screen_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "to_screen", 10, std::bind(&LcdScreenSubscriber::callback, this, _1));
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    static int i;
    i++;

    ros2_firmware::I2cInterface i2c_interface(1);
    // i2c_interface.open_bus();
    ros2_firmware::Lcm1602 lcm1602(i2c_interface, 0x27, 4, 20);
    lcm1602.initialize();
    lcm1602.clear();
    std::string str = msg->data;
    str += " ";
    str += std::to_string(i);
    RCLCPP_INFO(this->get_logger(), "%s", str.c_str());
    lcm1602.set_cursor(0, 0);
    lcm1602.print_msg(str);
    usleep(100000);
    lcm1602.stop();
    // i2c_interface.close_bus();
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  usleep(400000);
  rclcpp::spin(std::make_shared<LcdScreenSubscriber>());
  rclcpp::shutdown();
  return 0;
}
