#include "can_message_handler/can_message_receiver_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace can_message_handler
{

CanMessageReceiverNode::CanMessageReceiverNode(const rclcpp::NodeOptions & options)
: Node("can_message_receiver", options)
{
  can_subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus", 10,
    std::bind(&CanMessageReceiverNode::canMessageCallback, this, std::placeholders::_1));

  can_publisher_ = this->create_publisher<can_msgs::msg::Frame>(
    "to_can_bus", 10);

  vcu_status_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
    "vcu_status", 10);

  // Create publishers for wheel speed and steering angle
  wheel_speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
    "/twist_controller/input/velocity_status", 10);

  steer_angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
    "/twist_controller/input/steering_status", 10);

  RCLCPP_INFO(this->get_logger(), "CanMessageReceiverNode initialized");
}

CanMessageReceiverNode::~CanMessageReceiverNode()
{
  RCLCPP_INFO(this->get_logger(), "CanMessageReceiverNode destroyed");
}

void CanMessageReceiverNode::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  // Process CAN ID 0x211 for VCU status
  if (msg->id == 0x211) {
    bool vcu_ASDrvDisable = (msg->data[1] & 0x01) != 0;  // Check bit 0 of byte 1

    auto vcu_status_msg = std_msgs::msg::Bool();
    vcu_status_msg.data = vcu_ASDrvDisable;
    vcu_status_publisher_->publish(vcu_status_msg);

    RCLCPP_DEBUG(this->get_logger(), "VCU status: %s", vcu_ASDrvDisable ? "true" : "false");
  }
  // Process CAN ID 0x212 for wheel speed and steering angle
  else if (msg->id == 0x212) {
    // Parse wheel speed (byte 0, 8 bits)
    vcu_WheelSpeed_FL_ = msg->data[0];
    
    // Parse steering angle (bytes 4-5, starting from bit 32)
    // Convert raw value to actual angle using resolution
    int16_t raw_steer = (msg->data[5] << 8) | msg->data[4];
    vcu_SteerAngle_ = raw_steer * 0.01f;  // Apply resolution of 0.01

    // Publish wheel speed
    auto wheel_speed_msg = std_msgs::msg::Float64();
    wheel_speed_msg.data = vcu_WheelSpeed_FL_;
    wheel_speed_publisher_->publish(wheel_speed_msg);

    // Publish steering angle
    auto steer_angle_msg = std_msgs::msg::Float64();
    steer_angle_msg.data = vcu_SteerAngle_;
    steer_angle_publisher_->publish(steer_angle_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Wheel speed: %.2f, Steer angle: %.2f", 
                 vcu_WheelSpeed_FL_, vcu_SteerAngle_);
  }
}

}  // namespace can_message_handler 