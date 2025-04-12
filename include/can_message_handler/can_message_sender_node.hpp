#ifndef CAN_MESSAGE_HANDLER__CAN_MESSAGE_SENDER_NODE_HPP_
#define CAN_MESSAGE_HANDLER__CAN_MESSAGE_SENDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include "can_message_handler/aspc_state_machine.hpp"
#include <thread>
#include <atomic>
#include <memory>

namespace can_message_handler
{

class CanMessageSenderNode : public rclcpp::Node
{
public:
  explicit CanMessageSenderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CanMessageSenderNode();

private:
  // CAN message parameters
  uint32_t can_id_;
  bool is_extended_;
  uint8_t dlc_;
  std::vector<uint8_t> data_pattern_;
  std::chrono::milliseconds period_ms_;

  // Node components
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscriber_;
  std::thread publish_thread_;
  std::atomic<bool> is_running_;
  ASPCStateMachine state_machine_;

  // Methods
  void declareParameters();
  void startPublishing();
  void stopPublishing();
  void publishLoop();
  void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg);
};

}  // namespace can_message_handler

#endif  // CAN_MESSAGE_HANDLER__CAN_MESSAGE_SENDER_NODE_HPP_