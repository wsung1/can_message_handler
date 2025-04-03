#include "can_message_handler/can_message_receiver_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace can_message_handler
{

CanMessageReceiverNode::CanMessageReceiverNode(const rclcpp::NodeOptions & options)
: Node("can_message_receiver", options)
{
  declareParameters();
  target_can_id_ = this->get_parameter("target_can_id").as_int();
  target_byte_index_ = this->get_parameter("target_byte_index").as_int();
  target_bit_index_ = this->get_parameter("target_bit_index").as_int();

  can_subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus", 10,
    std::bind(&CanMessageReceiverNode::canMessageCallback, this, std::placeholders::_1));

  vcu_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
    "vcu_asdrv_disable", 10);

  RCLCPP_INFO(this->get_logger(), "CanMessageReceiverNode initialized");
}

CanMessageReceiverNode::~CanMessageReceiverNode()
{
  RCLCPP_INFO(this->get_logger(), "CanMessageReceiverNode destroyed");
}

void CanMessageReceiverNode::declareParameters()
{
  this->declare_parameter("target_can_id", 0x211);
  this->declare_parameter("target_byte_index", 1);
  this->declare_parameter("target_bit_index", 0);
}

void CanMessageReceiverNode::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (msg->id != target_can_id_) {
    return;
  }

  if (msg->dlc > target_byte_index_) {
    uint8_t target_byte = msg->data[target_byte_index_];
    bool bit_value = (target_byte & (1 << target_bit_index_)) == 0;

    auto vcu_status = std_msgs::msg::Bool();
    vcu_status.data = bit_value;
    vcu_status_publisher_->publish(vcu_status);

    RCLCPP_DEBUG(this->get_logger(), "VCU_ASDrvDisable status: %s", bit_value ? "true" : "false");
  }
}

}  // namespace can_message_handler 