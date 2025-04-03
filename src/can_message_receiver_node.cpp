#include "can_message_handler/can_message_receiver_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace can_message_handler
{

CanMessageReceiverNode::CanMessageReceiverNode(const rclcpp::NodeOptions & options)
: Node("can_message_receiver", options)
{
  // Declare and get parameters
  this->declare_parameter("target_can_id", 0x211);
  this->declare_parameter("target_byte", 0);
  this->declare_parameter("target_bit", 0);

  target_can_id_ = static_cast<uint32_t>(this->get_parameter("target_can_id").as_int());
  target_byte_ = static_cast<uint8_t>(this->get_parameter("target_byte").as_int());
  target_bit_ = static_cast<uint8_t>(this->get_parameter("target_bit").as_int());

  // Create subscription
  can_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
    "can_rx", 10,
    std::bind(&CanMessageReceiverNode::canMessageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "CanMessageReceiverNode has been initialized");
}

bool CanMessageReceiverNode::checkBit(uint8_t byte, uint8_t bit) const
{
  return (byte & (1 << bit)) != 0;
}

void CanMessageReceiverNode::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (msg->id != target_can_id_) {
    return;
  }

  if (target_byte_ >= msg->dlc) {
    RCLCPP_WARN(this->get_logger(), "Target byte %d is out of range (DLC: %d)", target_byte_, msg->dlc);
    return;
  }

  bool bit_value = checkBit(msg->data[target_byte_], target_bit_);
  RCLCPP_INFO(this->get_logger(), "CAN ID: 0x%X, Byte %d, Bit %d: %s",
    msg->id, target_byte_, target_bit_, bit_value ? "true" : "false");
}

}  // namespace can_message_handler 