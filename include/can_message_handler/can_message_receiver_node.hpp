#ifndef CAN_MESSAGE_HANDLER__CAN_MESSAGE_RECEIVER_NODE_HPP_
#define CAN_MESSAGE_HANDLER__CAN_MESSAGE_RECEIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/float64.hpp>
#include <memory>

namespace can_message_handler
{

class CanMessageReceiverNode : public rclcpp::Node
{
public:
  explicit CanMessageReceiverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CanMessageReceiverNode();

private:
  // Node components
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscriber_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;

  // Publishers for wheel speed and steering angle
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wheel_speed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_angle_publisher_;
  
  // Variables to store parsed values
  double vcu_WheelSpeed_FL_ = 0.0;  // Wheel speed value from CAN ID 0x212
  double vcu_SteerAngle_ = 0.0;  // Steering angle value from CAN ID 0x212 (min: -35, max: 35, resolution: 0.01)

  // Methods
  void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg);
};

}  // namespace can_message_handler

#endif  // CAN_MESSAGE_HANDLER__CAN_MESSAGE_RECEIVER_NODE_HPP_ 