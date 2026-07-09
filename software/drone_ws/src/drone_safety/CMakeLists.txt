cmake_minimum_required(VERSION 3.8)
project(drone_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)

add_executable(hold_failsafe_node src/hold_failsafe_node.cpp)
ament_target_dependencies(hold_failsafe_node rclcpp std_msgs drone_interfaces)

install(TARGETS
  hold_failsafe_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()