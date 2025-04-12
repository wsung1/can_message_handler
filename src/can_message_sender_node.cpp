#include "can_message_handler/can_message_sender_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <atomic>

namespace can_message_handler
{

CanMessageSenderNode::CanMessageSenderNode(const rclcpp::NodeOptions & options)
: Node("can_message_sender", options)
{
  publisher_ = this->create_publisher<can_msgs::msg::Frame>(
    "to_can_bus", 10);

  can_subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus", 10,
    std::bind(&CanMessageSenderNode::canMessageCallback, this, std::placeholders::_1));

  std::string can_ids_str = this->declare_parameter<std::string>("can_ids", "0x001");
  std::vector<uint32_t> can_ids;
  std::stringstream ss(can_ids_str);
  std::string id;
  while (std::getline(ss, id, ',')) {
    // Convert hex string to integer
    // Test
    uint32_t can_id;
    std::stringstream hex_ss;
    hex_ss << std::hex << id;
    hex_ss >> can_id;
    can_ids.push_back(can_id);
    RCLCPP_INFO(this->get_logger(), "Handling CAN ID: 0x%03x", can_id);
  }

  state_machine_.setCanIds(can_ids);  // New method to pass CAN IDs to state machine
  startPublishing();

  RCLCPP_INFO(this->get_logger(), "CanMessageSenderNode initialized");
}

CanMessageSenderNode::~CanMessageSenderNode()
{
  stopPublishing();
  RCLCPP_INFO(this->get_logger(), "CanMessageSenderNode destroyed");
}

void CanMessageSenderNode::startPublishing()
{
  is_running_ = true;
  publish_thread_ = std::thread(&CanMessageSenderNode::publishLoop, this);
}

void CanMessageSenderNode::stopPublishing()
{
  is_running_ = false;
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

void CanMessageSenderNode::publishLoop()
{
  while (is_running_) {
    auto messages = state_machine_.generateOutputMessages();
    for (const auto& message : messages) {
      publisher_->publish(message);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void CanMessageSenderNode::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  state_machine_.updateState(msg);
}

}  // namespace can_message_handler 
