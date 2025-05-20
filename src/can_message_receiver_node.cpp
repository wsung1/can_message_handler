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

  /* To AutowareInterface, TwistController */
  vehicle_speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
    "/can_message_receiver/velocity_status", 10);

  steer_angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
    "/can_message_receiver/steering_status", 10);

  gear_state_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
    "/can_message_receiver/gear_state", 10);

  /* Debug */
  vcu_speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
    "/can_message_receiver/debug/vcu_speed", 10);

  enco_speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
    "/can_message_receiver/debug/enco_speed", 10);
 
  kinematic_state_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/localization/kinematic_state", 10,
    std::bind(&CanMessageReceiverNode::kinematicStateCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "CanMessageReceiverNode initialized");
}

CanMessageReceiverNode::~CanMessageReceiverNode()
{
  RCLCPP_INFO(this->get_logger(), "CanMessageReceiverNode destroyed");
}

void CanMessageReceiverNode::kinematicStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{ 
  kinematic_state_linear_x_ = msg->twist.twist.linear.x;
  RCLCPP_DEBUG(this->get_logger(), "Kinematic state linear x: %.2f", kinematic_state_linear_x_);
}

void CanMessageReceiverNode::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  // Process CAN ID 0x100 for precise vehicle speed
  if (msg->id == 0x100) {
    // Parse vehicle speed from bytes 0-1 (16-bit value)
    // Resolution is 0.01 m/s
    int16_t raw_speed = (msg->data[1] << 8) | msg->data[0];
    enco_VehicleSpeed_ = raw_speed * 0.01 ; // * KPH2MPS;   // Need not convert to m/s

    // Apply direction based on kinematic state if available and in D gear
    if (std::abs(kinematic_state_linear_x_) > 0.1 && vcu_GearState_ == 4) {  // D gear
      // Use kinematic state's sign for direction
      enco_VehicleSpeed_ = std::abs(enco_VehicleSpeed_) * (kinematic_state_linear_x_ > 0 ? 1.0 : -1.0);
    }

    if (vcu_GearState_ == 2) {  // R gear
      enco_VehicleSpeed_ *= -1;  // Make speed negative for reverse
    }
    
    // Publish vehicle speed measured by encorder installed for better resolution
    auto enco_speed_msg = std_msgs::msg::Float64();
    enco_speed_msg.data = enco_VehicleSpeed_;
    enco_speed_publisher_->publish(enco_speed_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Speed: %.2f", enco_VehicleSpeed_);

    // Check if wheel encoder speed is reliable
    if (std::abs(enco_VehicleSpeed_ - vcu_VehicleSpeed_) < 1.0) {
      // If reliable, use average of both speeds
      final_VehicleSpeed_ = enco_VehicleSpeed_;
    } else {
      // If unreliable, use VCU speed and log error
      //final_VehicleSpeed_ = vcu_VehicleSpeed_;
      //final_VehicleSpeed_ = (enco_VehicleSpeed_ + vcu_VehicleSpeed_) / 2.0;
      final_VehicleSpeed_ = enco_VehicleSpeed_;
      RCLCPP_ERROR(this->get_logger(), 
        "Wheel encoder speed unreliable! Using VCU speed. VCU: %.2f m/s, Wheel Encoder: %.2f m/s", 
        vcu_VehicleSpeed_, enco_VehicleSpeed_);
    }

    // Publish final vehicle speed
    auto vehicle_speed_msg = std_msgs::msg::Float64();
    vehicle_speed_msg.data = final_VehicleSpeed_;
    vehicle_speed_publisher_->publish(vehicle_speed_msg);
  }

  // Process CAN ID 0x211 for gear state
  else if (msg->id == 0x211) {
    // Parse gear state (bits 4-6 of byte 0)
    vcu_GearState_ = (msg->data[0] >> 4) & 0x07;

    // Publish gear state
    auto gear_state_msg = std_msgs::msg::Int32();
    gear_state_msg.data = vcu_GearState_;
    gear_state_publisher_->publish(gear_state_msg);

    // Parse vehicle speed from byte 3 (8-bit value)
    // Resolution is 1 km/h
    int16_t raw_speed = msg->data[3];
    vcu_VehicleSpeed_ = raw_speed * KPH2MPS;

    // Apply direction based on kinematic state if available and in D gear
    if (std::abs(kinematic_state_linear_x_) > 0.1 && vcu_GearState_ == 4) {  // D gear
      // Use kinematic state's sign for direction
      vcu_VehicleSpeed_ = std::abs(vcu_VehicleSpeed_) * (kinematic_state_linear_x_ > 0 ? 1.0 : -1.0);
    }

    if (vcu_GearState_ == 2) {  // R gear
      vcu_VehicleSpeed_ *= -1;  // Make speed negative for reverse
    }

    // Publish vehicle speed measured by vcu
    auto vcu_speed_msg = std_msgs::msg::Float64();
    vcu_speed_msg.data = vcu_VehicleSpeed_;
    vcu_speed_publisher_->publish(vcu_speed_msg);
  }
  
  // Process CAN ID 0x212 for steering angle
  else if (msg->id == 0x212) {
    // Parse steering angle (bytes 4-5, starting from bit 32)
    // Convert raw value to actual angle using resolution
    int16_t raw_angle = (msg->data[5] << 8) | msg->data[4];
    vcu_SteerAngle_ = raw_angle * 0.01 * DEG2RAD * -1; // Apply resolution of 0.01
    // Publish steering angle
    auto steer_angle_msg = std_msgs::msg::Float64();
    steer_angle_msg.data = vcu_SteerAngle_;
    steer_angle_publisher_->publish(steer_angle_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Angle: %.2f", vcu_SteerAngle_);
  }
}

}  // namespace can_message_handler 