#include "can_message_handler/can_message_sender_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <thread>
#include <atomic>

namespace can_message_handler
{
CanMessageSenderNode::CanMessageSenderNode(const rclcpp::NodeOptions & options)
: Node("can_message_sender", options)
{
  publisher_ = this->create_publisher<can_msgs::msg::Frame>(
    "to_can_bus", 10);

  can_subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus", 10,
    std::bind(&CanMessageSenderNode::canMessageCallback, this, std::placeholders::_1));

  // Create subscribers for commands
  throttle_command_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
    "/twist_controller/output/throttle_cmd", 1,
    std::bind(&CanMessageSenderNode::throttleCommandCallback, this, std::placeholders::_1));

  brake_command_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
    "/twist_controller/output/brake_cmd", 1,
    std::bind(&CanMessageSenderNode::brakeCommandCallback, this, std::placeholders::_1));

  steering_command_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
    "/twist_controller/output/steering_cmd", 1,
    std::bind(&CanMessageSenderNode::steeringCommandCallback, this, std::placeholders::_1));

  gear_command_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
    "/twist_controller/output/gear_cmd", 1,
    std::bind(&CanMessageSenderNode::gearCommandCallback, this, std::placeholders::_1));

  std::string can_ids_str = this->declare_parameter<std::string>("can_ids", "0x001");
  std::vector<uint32_t> can_ids;
  std::stringstream ss(can_ids_str);
  std::string id;
  while (std::getline(ss, id, ',')) {
    // Convert hex string to integer
    // Test
    uint32_t can_id;
    std::stringstream hex_ss;
    hex_ss << std::hex << id;
    hex_ss >> can_id;
    can_ids.push_back(can_id);
    RCLCPP_INFO(this->get_logger(), "Handling CAN ID: 0x%03x", can_id);
  }

  state_machine_.setCanIds(can_ids);  // New method to pass CAN IDs to state machine
  startPublishing();

  RCLCPP_INFO(this->get_logger(), "CanMessageSenderNode initialized");
}

CanMessageSenderNode::~CanMessageSenderNode()
{
  stopPublishing();
  RCLCPP_INFO(this->get_logger(), "CanMessageSenderNode destroyed");
}

void CanMessageSenderNode::startPublishing()
{
  is_running_ = true;
  publish_thread_ = std::thread(&CanMessageSenderNode::publishLoop, this);
}

void CanMessageSenderNode::stopPublishing()
{
  is_running_ = false;
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

void CanMessageSenderNode::throttleCommandCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  throttle_command_ = msg->data;
  RCLCPP_DEBUG(this->get_logger(), "Throttle command received: %.2f", throttle_command_);
}

void CanMessageSenderNode::brakeCommandCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  brake_command_ = msg->data;
  RCLCPP_DEBUG(this->get_logger(), "Brake command received: %.2f", brake_command_);
}

void CanMessageSenderNode::steeringCommandCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  steering_command_ = msg->data;
  RCLCPP_DEBUG(this->get_logger(), "Steering command received: %.2f", steering_command_);
}

void CanMessageSenderNode::gearCommandCallback(const std_msgs::msg::Int32::SharedPtr msg)
{
  // Validate gear command (0-4 range)
  if (msg->data >= 0 && msg->data <= 4) {
    gear_command_ = msg->data;
    const char* gear_names[] = {"None", "P", "R", "N", "D"};
    RCLCPP_DEBUG(this->get_logger(), "Gear command received: %s (%d)", 
                 gear_names[gear_command_], gear_command_);
  } else {
    RCLCPP_WARN(this->get_logger(), "Invalid gear command received: %d (valid range: 0-4)", 
                msg->data);
  }
}

void CanMessageSenderNode::publishLoop()
{
  while (is_running_) {
    auto messages = state_machine_.generateOutputMessages();
    for (const auto& msg : messages) {
      can_msgs::msg::Frame message = msg;  // Create a copy of the message
      if (message.id == 0x001) {
        // Throttle command: byte 1 (bits 8-15)
        message.data[1] = static_cast<uint8_t>(throttle_command_ * 100.0);
        
        // Brake command: byte 2 (bits 16-23)
        message.data[2] = static_cast<uint8_t>(brake_command_ * 100.0);
        
        // Steering command: bytes 3-4 (bits 24-39)
        // Convert steering angle to raw value (resolution 0.01)
        int16_t steer_angle_raw = static_cast<int16_t>(steering_command_ / 0.01);
        message.data[3] = steer_angle_raw & 0xFF;         // Lower byte
        message.data[4] = (steer_angle_raw >> 8) & 0xFF;  // Upper byte
        
        // Gear command: byte 0 (bits 3-5)
        message.data[0] |= (gear_command_ << 3);  // Set gear command
        
        // Calculate checksum (sum of bytes 0-6)
        uint8_t checksum = 0;
        for (int i = 0; i < 7; ++i) {
          checksum += message.data[i];
        }
        message.data[7] = checksum;
      }
      publisher_->publish(message);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void CanMessageSenderNode::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  state_machine_.updateState(msg);
}

}  // namespace can_message_handler 
