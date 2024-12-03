#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

void sub_callback(std::shared_ptr<std_msgs::msg::String> msg)
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got msg: %s", msg->data.c_str());
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("hello_launch_sub");
  auto sub = node->create_subscription<std_msgs::msg::String>("listener", rclcpp::QoS(10), sub_callback); 

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
