cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)


add_executable(follow_controller_node src/follow_controller_node.cpp)
add_executable(setpoint_validation_node src/setpoint_validation_node.cpp)

ament_target_dependencies(follow_controller_node
    rclcpp
    std_msgs
    geometry_msgs
    nav_msgs
    drone_interfaces
)

ament_target_dependencies(setpoint_validation_node
    rclcpp
    drone_interfaces
)

target_include_directories(setpoint_validation_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS
  follow_controller_node
  setpoint_validation_node
  DESTINATION lib/${PROJECT_NAME})

# drone_safety measures saturation against the same envelope this package
# clamps to, so the header has to leave the package.
install(DIRECTORY include/
  DESTINATION include)
ament_export_include_directories(include)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # uncrustify enforces the ROS 2 whitespace style. This code follows its own
  # consistent style, and reformatting flight-proven files purely for style
  # would churn safety-critical code for no behavioural gain. cppcheck,
  # lint_cmake and xmllint stay enabled - those catch real defects.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_setpoint_limits test/test_setpoint_limits.cpp)
  target_include_directories(test_setpoint_limits PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_target_dependencies(test_setpoint_limits drone_interfaces)
endif()

ament_package()
