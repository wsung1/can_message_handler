#include "can_message_handler/can_message_publisher_node.hpp"
#include <thread>
#include <chrono>

namespace can_message_handler
{

CanMessagePublisherNode::CanMessagePublisherNode(const rclcpp::NodeOptions & options)
: Node("can_message_publisher", options)
{
  publisher_ = this->create_publisher<can_msgs::msg::Frame>(
    "to_can_bus", 10);

  startPublishing();

  RCLCPP_INFO(this->get_logger(), "CanMessagePublisherNode initialized");
}

CanMessagePublisherNode::~CanMessagePublisherNode()
{
  is_running_ = false;
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

void CanMessagePublisherNode::startPublishing()
{
  is_running_ = true;
  publish_thread_ = std::thread(&CanMessagePublisherNode::publishLoop, this);
}

void CanMessagePublisherNode::publishLoop()
{
  while (is_running_) {
    can_msgs::msg::Frame message;
    message.id = 0x001;
    message.is_rtr = false;
    message.is_error = false;
    message.is_extended = false;
    message.dlc = 8;

    // Initialize all data bytes to 0
    for (int i = 0; i < 8; ++i) {
      message.data[i] = 0;
    }

    publisher_->publish(message);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

}  // namespace can_message_handler 