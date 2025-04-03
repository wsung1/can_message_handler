#ifndef CAN_MESSAGE_HANDLER__CAN_MESSAGE_RECEIVER_NODE_HPP_
#define CAN_MESSAGE_HANDLER__CAN_MESSAGE_RECEIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/bool.hpp>
#include <memory>

namespace can_message_handler
{

class CanMessageReceiverNode : public rclcpp::Node
{
public:
  explicit CanMessageReceiverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CanMessageReceiverNode();

private:
  // CAN message parameters
  uint32_t target_can_id_;
  uint8_t target_byte_index_;
  uint8_t target_bit_index_;

  // Node components
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vcu_status_publisher_;

  // Methods
  void declareParameters();
  void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg);
};

}  // namespace can_message_handler

#endif  // CAN_MESSAGE_HANDLER__CAN_MESSAGE_RECEIVER_NODE_HPP_ 