cmake_minimum_required(VERSION 3.8)
project(drone_vision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)


add_executable(vision_node src/vision_node.cpp)

# 3. Länka biblioteken
# Här säger vi: "Programmet vision_node behöver OpenCV, ROS, etc för att fungera"
ament_target_dependencies(vision_node 
  rclcpp 
  sensor_msgs 
  cv_bridge 
  image_transport
  OpenCV
)

# 4. Installera
# Detta gör att kommandot 'ros2 run' kan hitta programmet
install(TARGETS
  vision_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
