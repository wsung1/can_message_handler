#ifndef CAN_MESSAGE_HANDLER__ASPC_STATE_MACHINE_HPP_
#define CAN_MESSAGE_HANDLER__ASPC_STATE_MACHINE_HPP_

#include <can_msgs/msg/frame.hpp>
#include <atomic>
#include <vector>

namespace can_message_handler
{

enum class ASPCState {
  INIT,
  READY,
  CONTROL_ENABLED,
  DRIVING
};

class ASPCStateMachine
{
public:
  ASPCStateMachine();
  ~ASPCStateMachine() = default;

  // State update based on VCU message
  void updateState(const can_msgs::msg::Frame::SharedPtr msg);
  
  // Get current state
  ASPCState getCurrentState() const { return current_state_; }

  // Generate output message based on current state
  std::vector<can_msgs::msg::Frame> generateOutputMessages();

  void setCanIds(const std::vector<uint32_t>& can_ids) { can_ids_ = can_ids; }

private:
  std::atomic<ASPCState> current_state_;
  std::vector<uint32_t> can_ids_;
  uint8_t alive_cnt_ = 0;  // Shared alive counter for all messages
};

}  // namespace can_message_handler

#endif  // CAN_MESSAGE_HANDLER__ASPC_STATE_MACHINE_HPP_ 