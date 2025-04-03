#include "can_message_handler/aspc_state_machine.hpp"
#include <rclcpp/rclcpp.hpp>

namespace can_message_handler
{

ASPCStateMachine::ASPCStateMachine()
: current_state_(ASPCState::INIT)
{
}

void ASPCStateMachine::updateState(const can_msgs::msg::Frame::SharedPtr msg)
{
  if (msg->id != 0x211) {
    return;
  }

  ASPCState new_state = current_state_;
  uint8_t byte0 = msg->data[0];
  uint8_t byte1 = msg->data[1];

  switch (current_state_) {
    case ASPCState::INIT:
      // Check Vcu_ASDrvDisable (211h 1.0)
      if ((byte1 & 0x01) == 0) {
        new_state = ASPCState::READY;
      }
      break;

    case ASPCState::READY:
      // Check Vcu_ASDrvModeRq (211h 0.2~3)
      if ((byte0 & 0x0C) == 0x0C) {  // Value 3
        new_state = ASPCState::CONTROL_ENABLED;
      }
      break;

    case ASPCState::CONTROL_ENABLED:
      // Check Vcu_ASDrvModeState (211h 0.0~1)
      if ((byte0 & 0x03) == 0x03) {  // Value 3
        new_state = ASPCState::DRIVING;
      }
    
    case ASPCState::DRIVING:
      // Check Vcu_ASDrvModeRq (211h 0.2~3)
      if ((byte0 & 0x0C) == 0x00) {  // Value 0
        new_state = ASPCState::READY;
      }
      break;

    default:
      break;
  }

  if (new_state != current_state_) {
    current_state_ = new_state;
    const char* state_name;
    switch (new_state) {
      case ASPCState::INIT: state_name = "INIT"; break;
      case ASPCState::READY: state_name = "READY"; break;
      case ASPCState::CONTROL_ENABLED: state_name = "CONTROL_ENABLED"; break;
      case ASPCState::DRIVING: state_name = "DRIVING"; break;
      default: state_name = "UNKNOWN"; break;
    }
    RCLCPP_INFO(rclcpp::get_logger("ASPCStateMachine"), "State changed to: %s", state_name);
  }
}

std::vector<can_msgs::msg::Frame> ASPCStateMachine::generateOutputMessages()
{
  std::vector<can_msgs::msg::Frame> messages;
  
  // Calculate AliveCnt (4-bit counter, 0-15)
  uint8_t current_alive_cnt = alive_cnt_;
  alive_cnt_ = (alive_cnt_ + 1) & 0x0F;  // Increment and wrap around at 16
  
  for (const auto& can_id : can_ids_) {
    can_msgs::msg::Frame message;
    message.id = can_id;
    message.is_rtr = false;
    message.is_error = false;
    message.is_extended = false;
    message.dlc = 8;

    // Initialize all data bytes to 0
    for (int i = 0; i < 8; ++i) {
      message.data[i] = 0;
    }

    // Set AliveCnt in data[6] bits 4-7
    message.data[6] = (current_alive_cnt << 4);  // Shift to bits 4-7

    // Set control bits based on current state
    ASPCState state = current_state_.load();
    
    switch (state) {
      case ASPCState::READY:
        if (can_id == 0x001) {
          // Set Aspc_ASDrvRdy (001h 0.2)
          message.data[0] |= (1 << 2);
          // Reset Aspc_AclDclCtlOn (001h 0.0)
          message.data[0] &= ~(1 << 0);
          // Reset Aspc_SteerCtlOn (001h 0.1)
          message.data[0] &= ~(1 << 1);
          // Set GearCmd (001h 0.3~5)
          message.data[0] |= (1 << 3); // P
          message.data[3] = 0;
        }
        break;

      case ASPCState::CONTROL_ENABLED:
        if (can_id == 0x001) {
          // Set Aspc_AclDclCtlOn (001h 0.0)
          message.data[0] |= (1 << 0);
          // Set Aspc_SteerCtlOn (001h 0.1)
          message.data[0] |= (1 << 1);
        }
        break;

      case ASPCState::DRIVING:
        if (can_id == 0x001) {
          // Set all control bits
          message.data[0] |= 0x0F;  // Bits 0-3
          // Set GearCmd (001h 0.3~5)
          message.data[0] |= (4 << 3);  // D
          // Set AccelPosCmd (001h 1.0~7)
          message.data[1] = 0x01;
          // Set BrakePosCmd (001h 2.0~7)
          message.data[2] = 0x02;
          // Set SteerAngleCmd (001h 3.0~7)
          message.data[3] = 0x03;
        }
        break;

      default:
        break;
    }

    // Calculate checksum (sum of bytes 0-6)
    uint8_t checksum = 0;
    for (int i = 0; i < 7; ++i) {
      checksum += message.data[i];
    }
    message.data[7] = checksum;

    messages.push_back(message);
  }

  return messages;
}

}  // namespace can_message_handler 