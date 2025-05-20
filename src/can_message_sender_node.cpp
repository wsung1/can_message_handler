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
  // Declare door command parameter
  this->declare_parameter("door_command", false);

  publisher_ = this->create_publisher<can_msgs::msg::Frame>(
    "to_can_bus", 10);

  door_state_publisher_ = this->create_publisher<std_msgs::msg::UInt8>(
    "/can_message_sender/door_status", 10);

  can_subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
    "from_can_bus", 10,
    std::bind(&CanMessageSenderNode::canMessageCallback, this, std::placeholders::_1));

  /* From TwistController */
  throttle_command_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
    "/twist_controller/throttle_cmd", 1,
    std::bind(&CanMessageSenderNode::throttleCommandCallback, this, std::placeholders::_1));

  brake_command_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
    "/twist_controller/brake_cmd", 1,
    std::bind(&CanMessageSenderNode::brakeCommandCallback, this, std::placeholders::_1));

  steering_command_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
    "/twist_controller/steering_cmd", 1,
    std::bind(&CanMessageSenderNode::steeringCommandCallback, this, std::placeholders::_1));
  
  /* From AutowareInterface */
  gear_command_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
    "/autoware_interface/gear_cmd", 1, 
    std::bind(&CanMessageSenderNode::gearCommandCallback, this, std::placeholders::_1));

  /* From Autoware API */
  route_state_subscriber_ = this->create_subscription<autoware_adapi_v1_msgs::msg::RouteState>(
    "/api/routing/state", 1,
    std::bind(&CanMessageSenderNode::routeStateCallback, this, std::placeholders::_1));

  clear_route_client_ = this->create_client<autoware_adapi_v1_msgs::srv::ClearRoute>(
    "/planning/mission_planning/clear_route");

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
  // Map input values to output values
  int32_t new_gear = 0;
  switch (msg->data) {
    case 22:  // P
      new_gear = 1;
      break;
    case 20:  // R
      new_gear = 2;
      break;
    case 0:   // N
      new_gear = 3;
      break;
    case 2:   // D
      new_gear = 4;
      break;
    default:  // Invalid value
      RCLCPP_WARN(this->get_logger(), "Invalid gear command received: %d (valid values: 22, 20, 0, 2)", 
                  msg->data);
      return;
  }

  // Check if this is a gear change to D or R
  if ((new_gear == 2 || new_gear == 4) && new_gear != gear_command_) {
    // Start gear change sequence
    is_gear_changing_ = true;
    target_gear_ = new_gear;
    
    // Apply brake immediately
    // brake_command_ = 0.5; // Apply brake immediately //HJK_250507
    RCLCPP_INFO(this->get_logger(), "Starting gear change sequence to %s with %.2f brake applied", 
                new_gear == 2 ? "R" : "D", brake_command_);
  } else {
    // Normal gear change without brake sequence
    gear_command_ = new_gear;
  }

  const char* gear_names[] = {"None", "P", "R", "N", "D"};
  RCLCPP_DEBUG(this->get_logger(), "Gear command received: %s (%d)", 
               gear_names[new_gear], new_gear);
}

void CanMessageSenderNode::publishLoop()
{
  static bool prev_door_cmd = false;  // Track previous door command state
  static int cnt = 0;      // Counter for delayed update

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
        int16_t steer_angle_raw = static_cast<int16_t>(steering_command_ * RAD2DEG / 0.01);
        message.data[3] = steer_angle_raw & 0xFF;         // Lower byte
        message.data[4] = (steer_angle_raw >> 8) & 0xFF;  // Upper byte
        
        // Gear command: byte 0 (bits 3-5)
        message.data[0] |= (gear_command_ << 3);  // Set gear command

        // Door open command: byte 5 (bit 0)
        // bool door_cmd = this->get_parameter("door_command").as_bool();
        bool door_cmd = (route_state_ == autoware_adapi_v1_msgs::msg::RouteState::ARRIVED);
        /*
        RCLCPP_INFO(this->get_logger(), "prev_door_cmd: %d, door_cmd: %d, Gear: %d, Counter: %d", 
                   prev_door_cmd, door_cmd, vcu_GearState_, cnt);
        RCLCPP_INFO(this->get_logger(), "Gear State: %d (0: None, 1: P, 2: R, 3: N, 4: D)", vcu_GearState_);
        RCLCPP_INFO(this->get_logger(), "Door Open: %d (0: Closed, 1: Open)", vcu_DoorOpen_);
        RCLCPP_INFO(this->get_logger(), "Door In Progress: %d (0: No, 1: Yes)", vcu_DoorInProgress_);
        RCLCPP_INFO(this->get_logger(), "Door State: %d", DoorState_);
        */
        // Only send door command in P gear
        if (vcu_GearState_ == 1) {  // In P gear
          if (DoorState_ < 2 && vcu_DoorOpen_ == 0) {
            // message.data[5] = (!prev_door_cmd && door_cmd) ? 0x01 : 0x00;  // Send 1 only on rising edge
            if (!prev_door_cmd && door_cmd) { // message.data[5] 0 -> 1
                message.data[5] = 0x01;
                if (vcu_DoorInProgress_ == 1) {
                  DoorState_ = 1;
                  RCLCPP_INFO(this->get_logger(), "Door state changed to OPENING");
                }
              } else {
                message.data[5] = 0x00;
              }

            // Update prev_door_cmd after 50 loops (500ms) // message.data[5] 1 -> 0
            if (DoorState_ == 1 && door_cmd) {
              if (cnt >= 50) {
                prev_door_cmd = door_cmd;
                cnt = 0;
                DoorState_ = 2;
                RCLCPP_INFO(this->get_logger(), "Door state changed to WAITING");
              } else {
                cnt++;
              }
            } else {
              cnt = 0;
            }
          }
          else {
            if (DoorState_ == 2 && vcu_DoorOpen_ == 1 && vcu_DoorInProgress_ == 0) {
              DoorState_ = 3;
              RCLCPP_INFO(this->get_logger(), "Door state changed to OPEN");
            }
            else if (DoorState_ == 3 && vcu_DoorInProgress_ == 1) {
              DoorState_ = 4;
              RCLCPP_INFO(this->get_logger(), "Door state changed to CLOSING");
            }
            else if (DoorState_ == 4 && vcu_DoorOpen_ == 0 && vcu_DoorInProgress_ == 0) {
              DoorState_ = 5;
              RCLCPP_INFO(this->get_logger(), "Door state changed to CLOSED");
            }  
            /*
            else {
              DoorState_ = 9;
              RCLCPP_INFO(this->get_logger(), "Door state in TRANSITION");
            }
            */
          }
        } else {
          message.data[5] = 0x00;  // No door command in other gears
          cnt = 0;      // Reset counter when not in P gear
          prev_door_cmd = false;
          DoorState_ = 0;
        }
        
        // Calculate checksum (sum of bytes 0-6)
        uint8_t checksum = 0;
        for (int i = 0; i < 7; ++i) {
          checksum += message.data[i];
        }
        message.data[7] = checksum;
      }
      publisher_->publish(message);
    }
    
    // Publish door state once per loop
    std_msgs::msg::UInt8 door_state_msg;
    door_state_msg.data = DoorState_;
    door_state_publisher_->publish(door_state_msg);
    
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void CanMessageSenderNode::canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
  state_machine_.updateState(msg);

  if (msg->id == 0x211) {
    // Parse gear state (bits 4-6 of byte 0)
    vcu_GearState_ = (msg->data[0] >> 4) & 0x07;
    
    // Parse door state (bit 3 of byte 2)
    vcu_DoorOpen_ = (msg->data[2] >> 3) & 0x01;
    
    // Parse door in progress state (bit 2 of byte 1)
    vcu_DoorInProgress_ = (msg->data[1] >> 2) & 0x01;
    
    // Parse brake position (bits 40-47 of byte 5)
    vcu_BrakePressurePct_ = static_cast<int32_t>(msg->data[5]);  // 0-100 range
    
    RCLCPP_DEBUG(this->get_logger(), "VCU Gear State: %d (0: None, 1: P, 2: R, 3: N, 4: D), Brake Pressure: %d%%", 
                 vcu_GearState_, vcu_BrakePressurePct_);
    
    RCLCPP_DEBUG(this->get_logger(), "VCU Door State: %d (0: Closed, 1: Open), Door In Progress: %d (0: No, 1: Yes)", 
                 vcu_DoorOpen_, vcu_DoorInProgress_);
    
    // Check if gear change is in progress
    if (is_gear_changing_) {
      // Check if brake pressure has reached 70% of target (0.5 * 100 * 0.7 = 35%)
      if (vcu_BrakePressurePct_ >= 35) {
        // Send gear change command
        gear_command_ = target_gear_;
        RCLCPP_INFO(this->get_logger(), "Brake pressure reached %d%%, sending gear change command to %d", 
                    vcu_BrakePressurePct_, target_gear_);
      }
      
      // Check if VCU gear state matches target gear
      if (vcu_GearState_ == target_gear_) {
        RCLCPP_INFO(this->get_logger(), "Gear change confirmed by VCU feedback: %d", vcu_GearState_);
        is_gear_changing_ = false;
        brake_command_ = 0.0;  // Release brake
        RCLCPP_INFO(this->get_logger(), "Gear change sequence completed with %.2f brake applied", brake_command_);
      }
    }
  }
}

void CanMessageSenderNode::routeStateCallback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg)
{
  route_state_ = msg->state;
  
  const char* state_names[] = {"UNKNOWN", "UNSET", "SET", "ARRIVED", "CHANGING"};
  RCLCPP_INFO(this->get_logger(), "Route state changed to: %s", state_names[msg->state]);
}

}  // namespace can_message_handler 
