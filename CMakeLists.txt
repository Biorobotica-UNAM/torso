cmake_minimum_required(VERSION 3.8)
project(torso)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(torso_interfaces REQUIRED)

add_executable(torso_node src/torso_node.cpp)

ament_target_dependencies(torso_node
  rclcpp
  std_msgs
  sensor_msgs
  torso_interfaces
)

install(TARGETS
  torso_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
