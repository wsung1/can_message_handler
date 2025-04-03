#ifndef CAN_MESSAGE_HANDLER__CAN_MESSAGE_PUBLISHER_NODE_HPP_
#define CAN_MESSAGE_HANDLER__CAN_MESSAGE_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include "can_message_handler/aspc_state_machine.hpp"

namespace can_message_handler
{

class CanMessagePublisherNode : public rclcpp::Node
{
public:
  explicit CanMessagePublisherNode(const rclcpp::NodeOptions & options);

private:
  void timerCallback();
  void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;
  ASPCStateMachine state_machine_;
  std::vector<uint32_t> can_ids_;
};

}  // namespace can_message_handler

#endif  // CAN_MESSAGE_HANDLER__CAN_MESSAGE_PUBLISHER_NODE_HPP_ 