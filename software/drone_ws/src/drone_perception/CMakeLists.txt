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
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Python nodes. Keep MediaPipe in Python; camera/preprocess/tracking/estimation stay C++.
install(PROGRAMS
  drone_perception/person_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# C++ nodes
add_executable(camera_driver_node src/camera_driver_node.cpp)
ament_target_dependencies(camera_driver_node rclcpp sensor_msgs cv_bridge OpenCV)

add_executable(image_preprocessing_node src/image_preprocessing_node.cpp)
ament_target_dependencies(image_preprocessing_node rclcpp sensor_msgs cv_bridge OpenCV)

# add_executable(person_tracker_node src/person_tracker_node.cpp)
# ament_target_dependencies(person_tracker_node rclcpp sensor_msgs drone_interfaces)

# add_executable(target_estimator_node src/target_estimator_node.cpp)
# ament_target_dependencies(target_estimator_node rclcpp sensor_msgs drone_interfaces geometry_msgs)

install(TARGETS
  camera_driver_node
  image_preprocessing_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
