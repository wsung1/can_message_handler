#include "can_message_handler/can_message_receiver_node.hpp"
#include <rclcpp/rclcpp.hpp>

namespace can_message_handler
{

CanMessageReceiverNode::CanMessageReceiverNode(const rclcpp::NodeOptions & options)
: Node("can_message_receiver", options)
{
  // Declare parameters
  this->declare_parameter("interpolation_factor", 0.1);
  
  // Get parameters
  interpolation_factor_ = this->get_parameter("interpolation_factor").as_double();
  
  // Log parameter values
  RCLCPP_INFO(this->get_logger(), "Interpolation factor: %.2f", interpolation_factor_);
  
  can_subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus", 10,
    std::bind(&CanMessageReceiverNode::canMessageCallback, this, std::placeholders::_1));

  can_publisher_ = this->create_publisher<can_msgs::msg::Frame>(
    "to_can_bus", 10);

  /* To AutowareInterface, TwistController */
  vehicle_speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
    "/can_message_handler/velocity_status", 10);

  steer_angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
    "/can_message_handler/steering_status", 10);

  RCLCPP_INFO(this->get_logger(), "CanMessageReceiverNode initialized");
}

CanMessageReceiverNode::~CanMessageReceiverNode()
{
  RCLCPP_INFO(this->get_logger(), "CanMessageReceiverNode destroyed");
}

void CanMessageReceiverNode::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  // Update parameters if they have changed
  if (this->has_parameter("interpolation_factor") && 
      this->get_parameter("interpolation_factor").as_double() != interpolation_factor_) {
    interpolation_factor_ = this->get_parameter("interpolation_factor").as_double();
    RCLCPP_INFO(this->get_logger(), "Interpolation factor updated to: %.2f", interpolation_factor_);
  }
  
  // Process CAN ID 0x211 for vehicle speed
  if (msg->id == 0x211) {
    // Parse vehicle speed from bits 24-31 (bytes 3)
    int16_t raw_speed = msg->data[3];
    double new_speed = raw_speed * KPH2MPS;
    
    // Static variables for interpolation
    static double target_speed = 0.0;
    static double current_speed = 0.0;
    static bool first_message = true;
    
    // Initialize on first message
    if (first_message) {
      target_speed = new_speed;
      current_speed = new_speed;
      first_message = false;
      RCLCPP_DEBUG(this->get_logger(), "First speed message: %.2f", new_speed);
    } else {
      // Update target speed when it changes
      target_speed = new_speed;
      RCLCPP_DEBUG(this->get_logger(), "Speed target changed to: %.2f", target_speed);
      
      // Simple linear interpolation: move interpolation_factor_ of the remaining distance each step
      double diff = target_speed - current_speed;
      current_speed += diff * interpolation_factor_;  // Move interpolation_factor_ closer to target
      
      // Update the vehicle speed
      vcu_VehicleSpeed_ = current_speed;
    }
    
    // Publish vehicle speed
    auto vehicle_speed_msg = std_msgs::msg::Float64();
    vehicle_speed_msg.data = vcu_VehicleSpeed_;
    vehicle_speed_publisher_->publish(vehicle_speed_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Speed: %.2f (target: %.2f, current: %.2f)", 
                 vcu_VehicleSpeed_, target_speed, current_speed);
  }
  // Process CAN ID 0x212 for steering angle
  else if (msg->id == 0x212) {
    // Parse steering angle (bytes 4-5, starting from bit 32)
    // Convert raw value to actual angle using resolution
    int16_t raw_angle = (msg->data[5] << 8) | msg->data[4];
    vcu_SteerAngle_ = raw_angle * 0.01 * DEG2RAD; // Apply resolution of 0.01
    // Publish steering angle
    auto steer_angle_msg = std_msgs::msg::Float64();
    steer_angle_msg.data = vcu_SteerAngle_;
    steer_angle_publisher_->publish(steer_angle_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Angle: %.2f", vcu_SteerAngle_);
  }
}

}  // namespace can_message_handler 