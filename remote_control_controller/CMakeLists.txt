cmake_minimum_required(VERSION 3.8)
project(remote_control_controller)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(addverb_cobot_msgs REQUIRED)

# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Build Node executable
add_executable(gamepad_control_node src/gamepad_control_node.cpp)
target_include_directories(gamepad_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(gamepad_control_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_srvs
  addverb_cobot_msgs
)

# Install executable
install(TARGETS gamepad_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
