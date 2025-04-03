cmake_minimum_required(VERSION 3.8)
project(can_message_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# Create the publisher node executable
add_executable(can_message_publisher_node
  src/publisher_main.cpp
  src/can_message_publisher_node.cpp
  src/aspc_state_machine.cpp
)

# Create the receiver node executable
add_executable(can_message_receiver_node
  src/receiver_main.cpp
  src/can_message_receiver_node.cpp
)

# Link dependencies
ament_target_dependencies(can_message_publisher_node
  rclcpp
  can_msgs
  std_msgs
)

ament_target_dependencies(can_message_receiver_node
  rclcpp
  can_msgs
  std_msgs
)

# Install targets
install(TARGETS
  can_message_publisher_node
  can_message_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
