#ifndef CAN_MESSAGE_HANDLER__CAN_MESSAGE_RECEIVER_NODE_HPP_
#define CAN_MESSAGE_HANDLER__CAN_MESSAGE_RECEIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
  void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg);
  void kinematicStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg); 

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscriber_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vehicle_speed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_angle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gear_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr vcu_speed_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr enco_speed_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_subscriber_;
  
  double vcu_VehicleSpeed_;
  double enco_VehicleSpeed_;
  double final_VehicleSpeed_;
  double vcu_SteerAngle_;
  int vcu_GearState_;
  double kinematic_state_linear_x_;
};

}  // namespace can_message_handler

#endif  // CAN_MESSAGE_HANDLER__CAN_MESSAGE_RECEIVER_NODE_HPP_ 
