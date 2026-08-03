cmake_minimum_required(VERSION 3.8)
project(drone_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(drone_control REQUIRED)

add_executable(hold_failsafe_node src/hold_failsafe_node.cpp)
ament_target_dependencies(hold_failsafe_node
    rclcpp
    drone_interfaces
)
target_include_directories(hold_failsafe_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(safety_supervision_node src/safety_supervision_node.cpp)
ament_target_dependencies(safety_supervision_node
    rclcpp
    std_msgs
    drone_interfaces
    drone_control
)
target_include_directories(safety_supervision_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS
  hold_failsafe_node
  safety_supervision_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # Same reasoning as the other packages: the project has its own consistent
  # style and reformatting safety code for whitespace buys nothing.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_hold_policy test/test_hold_policy.cpp)
  target_include_directories(test_hold_policy PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_hold_policy drone_interfaces)

  ament_add_gtest(test_safety_checks test/test_safety_checks.cpp)
  target_include_directories(test_safety_checks PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_safety_checks drone_interfaces)
endif()

ament_package()
