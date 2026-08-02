cmake_minimum_required(VERSION 3.8)
project(drone_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(drone_interfaces REQUIRED)

# Python nodes. picamera2 and MediaPipe have no usable C++ API, so camera capture
# and detection stay in Python; tracking and estimation are C++.
install(PROGRAMS
  drone_perception/camera_driver_node.py
  drone_perception/person_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# C++ nodes
add_executable(person_tracker_node src/person_tracker_node.cpp)
ament_target_dependencies(person_tracker_node rclcpp sensor_msgs drone_interfaces)

add_executable(target_estimator_node src/target_estimator_node.cpp)
ament_target_dependencies(target_estimator_node rclcpp sensor_msgs drone_interfaces geometry_msgs)
target_include_directories(target_estimator_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include)

install(TARGETS
  person_tracker_node
  target_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_range_model test/test_range_model.cpp)
  target_include_directories(test_range_model PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

ament_package()
