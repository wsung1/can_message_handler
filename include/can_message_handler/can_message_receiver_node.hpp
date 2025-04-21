#ifndef CAN_MESSAGE_HANDLER__CAN_MESSAGE_RECEIVER_NODE_HPP_
#define CAN_MESSAGE_HANDLER__CAN_MESSAGE_RECEIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/float64.hpp>
#include <memory>

#define KPH2MPS 1/3.6
#define DEG2RAD 0.0174533

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

  // Publishers for vehicle speed and steering angle
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vehicle_speed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_angle_publisher_;
  
  // Variables to store parsed values
  double vcu_SteerAngle_ = 0.0;  // Steering angle value from CAN ID 0x212 (min: -35, max: 35, resolution: 0.01)
  double vcu_VehicleSpeed_ = 0;  // Vehicle speed in km/h from CAN ID 0x211
  double prev_vcu_VehicleSpeed_ = 0;  // Previous vehicle speed for interpolation
  
  // Parameters
  double interpolation_factor_;  // Factor for speed interpolation (0.0 to 1.0)

  // Methods
  void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg);
};

}  // namespace can_message_handler

#endif  // CAN_MESSAGE_HANDLER__CAN_MESSAGE_RECEIVER_NODE_HPP_ 