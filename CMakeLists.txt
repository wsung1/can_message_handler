cmake_minimum_required(VERSION 3.8)
project(can_message_handler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)

# Include directories
include_directories(include)

# Create the can_message_publisher_node executable
add_executable(can_message_publisher_node
  src/main.cpp
  src/can_message_publisher_node.cpp
)

# Link dependencies
ament_target_dependencies(can_message_publisher_node
  rclcpp
  can_msgs
)

# Install targets
install(TARGETS
  can_message_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package() 