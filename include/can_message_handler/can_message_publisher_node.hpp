#ifndef CAN_MESSAGE_HANDLER__CAN_MESSAGE_PUBLISHER_NODE_HPP_
#define CAN_MESSAGE_HANDLER__CAN_MESSAGE_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include "can_message_handler/aspc_state_machine.hpp"

namespace can_message_handler
{

class CanMessagePublisherNode : public rclcpp::Node
{
public:
  explicit CanMessagePublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CanMessagePublisherNode();

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

#endif  // CAN_MESSAGE_HANDLER__CAN_MESSAGE_PUBLISHER_NODE_HPP_ 