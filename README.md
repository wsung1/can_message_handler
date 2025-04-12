# can_message_handler

ROS 2 package for handling CAN messages.

## Requirements
- ROS 2 Humble or later
- ros2_socketcan package

## Dependencies
- rclcpp
- can_msgs

## Features
### Current
- Basic CAN message publishing (ID: 0x001)
- 10ms publishing rate

### TODO
- [ ] Add message parsing functionality
- [ ] Add message validation
- [ ] Add error handling
- [ ] Add configuration parameters

## Usage
### Build
```bash
colcon build --packages-select can_message_handler
```

### Run
1. Set up the virtual CAN interface:
```bash
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
```

2. Start the CAN bridge:
```bash
ros2 launch ros2_socketcan socket_can_bridge.launch.xml interface:=vcan0
```

3. Run the publisher node:
```bash
ros2 run can_message_handler can_message_sender_node
```

4. Monitor CAN messages:
```bash
candump vcan0
```

## Changelog
### v0.0.1 (2024-04-02)
- Initial release
- Basic CAN message publishing functionality 