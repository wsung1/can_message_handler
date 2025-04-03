#ifndef CAN_MESSAGE_HANDLER__ASPC_STATE_MACHINE_HPP_
#define CAN_MESSAGE_HANDLER__ASPC_STATE_MACHINE_HPP_

#include <can_msgs/msg/frame.hpp>
#include <vector>
#include <atomic>

namespace can_message_handler
{

enum class ASPCState
{
  INIT,
  READY,
  CONTROL_ENABLED,
  DRIVING
};

class ASPCStateMachine
{
public:
  ASPCStateMachine();
  void updateState(const can_msgs::msg::Frame::SharedPtr msg);
  std::vector<can_msgs::msg::Frame> generateOutputMessages();
  void setCanIds(const std::vector<uint32_t>& can_ids) { can_ids_ = can_ids; }

private:
  bool validateMessage(const can_msgs::msg::Frame::SharedPtr msg);
  uint8_t calculateChecksum(const can_msgs::msg::Frame& msg);
  void updateAliveCnt();
  
  std::atomic<ASPCState> current_state_;
  std::vector<uint32_t> can_ids_;
  uint8_t alive_cnt_{0};
};

}  // namespace can_message_handler

#endif  // CAN_MESSAGE_HANDLER__ASPC_STATE_MACHINE_HPP_
