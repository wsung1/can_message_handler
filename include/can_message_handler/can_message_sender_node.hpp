#ifndef CAN_MESSAGE_HANDLER__CAN_MESSAGE_SENDER_NODE_HPP_
#define CAN_MESSAGE_HANDLER__CAN_MESSAGE_SENDER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <can_msgs/msg/frame.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <autoware_adapi_v1_msgs/msg/route_state.hpp>
#include <autoware_adapi_v1_msgs/srv/clear_route.hpp>
#include "can_message_handler/aspc_state_machine.hpp"
#include <thread>
#include <atomic>
#include <memory>

#define RAD2DEG 57.2958

namespace can_message_handler
{

class CanMessageSenderNode : public rclcpp::Node
{
public:
  explicit CanMessageSenderNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CanMessageSenderNode();

private:
  // CAN message parameters
  uint32_t can_id_;
  bool is_extended_;
  uint8_t dlc_;
  std::vector<uint8_t> data_pattern_;
  std::chrono::milliseconds period_ms_;

  // Node components
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr door_state_publisher_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ClearRoute>::SharedPtr clear_route_client_;

  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr throttle_command_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr brake_command_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr steering_command_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gear_command_subscriber_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::RouteState>::SharedPtr route_state_subscriber_;
  
  std::thread publish_thread_;
  std::atomic<bool> is_running_;
  ASPCStateMachine state_machine_;

  // Variables to store command values
  double throttle_command_ = 0.0;
  double brake_command_ = 0.0;
  double steering_command_ = 0.0;
  int32_t gear_command_ = 0;
  
  int32_t vcu_GearState_ = 0;  // Gear State (0: None, 1: P, 2: R, 3: N, 4: D)
  int32_t vcu_BrakePressurePct_ = 0;  // Brake pressure percentage from VCU (0-100)
  int32_t vcu_DoorOpen_ = 0;  // 0: Closed, 1: Open
  int32_t vcu_DoorInProgress_ = 0;  // 0: No, 1: Yes

  // Variables for gear change handling
  bool is_gear_changing_ = false;
  int32_t target_gear_ = 0;

  // Route state
  uint16_t route_state_ = autoware_adapi_v1_msgs::msg::RouteState::UNKNOWN;

  // Door state
  uint8_t DoorState_ = 0;  // Current door state
  
  // Methods
  void declareParameters();
  void startPublishing();
  void stopPublishing();
  void publishLoop();
  void canMessageCallback(const can_msgs::msg::Frame::SharedPtr msg);
  void throttleCommandCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void brakeCommandCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void steeringCommandCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void gearCommandCallback(const std_msgs::msg::Int32::SharedPtr msg);
  void routeStateCallback(const autoware_adapi_v1_msgs::msg::RouteState::SharedPtr msg);
};

}  // namespace can_message_handler

#endif  // CAN_MESSAGE_HANDLER__CAN_MESSAGE_SENDER_NODE_HPP_