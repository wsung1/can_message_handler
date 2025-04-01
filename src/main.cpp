#include "can_message_handler/can_message_publisher_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<can_message_handler::CanMessagePublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 