#include "can_message_handler/can_message_publisher_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace can_message_handler
{

CanMessagePublisherNode::CanMessagePublisherNode(const rclcpp::NodeOptions & options)
: Node("can_message_publisher", options)
{
  // Declare and get parameters
  this->declare_parameter("can_ids", std::vector<int64_t>{0x001, 0x002, 0x003});
  auto can_ids_param = this->get_parameter("can_ids").as_integer_array();
  
  // Convert int64_t to uint32_t
  can_ids_.reserve(can_ids_param.size());
  for (const auto& id : can_ids_param) {
    can_ids_.push_back(static_cast<uint32_t>(id));
  }
  
  // Set CAN IDs in state machine
  state_machine_.setCanIds(can_ids_);

  // Create publisher
  can_publisher_ = this->create_publisher<can_msgs::msg::Frame>("can_tx", 10);

  // Create subscription
  can_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_rx", 10,
    std::bind(&CanMessagePublisherNode::canMessageCallback, this, std::placeholders::_1));

  // Create timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10),
    std::bind(&CanMessagePublisherNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "CanMessagePublisherNode has been initialized");
}

void CanMessagePublisherNode::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  state_machine_.updateState(msg);
}

void CanMessagePublisherNode::timerCallback()
{
  auto messages = state_machine_.generateOutputMessages();
  for (const auto& message : messages) {
    can_publisher_->publish(message);
  }
}

}  // namespace can_message_handler 