#ifndef CAN_MESSAGE_HANDLER__CAN_MESSAGE_PUBLISHER_NODE_HPP_
#define CAN_MESSAGE_HANDLER__CAN_MESSAGE_PUBLISHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <memory>
#include <thread>
#include <atomic>

namespace can_message_handler
{

class CanMessagePublisherNode : public rclcpp::Node
{
public:
  explicit CanMessagePublisherNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CanMessagePublisherNode();

private:
  // Node components
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;

  // Thread control
  std::thread publish_thread_;
  std::atomic<bool> is_running_{false};

  // Methods
  void startPublishing();
  void publishLoop();
};

}  // namespace can_message_handler

#endif  // CAN_MESSAGE_HANDLER__CAN_MESSAGE_PUBLISHER_NODE_HPP_ 