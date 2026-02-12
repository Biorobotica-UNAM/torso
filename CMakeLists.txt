cmake_minimum_required(VERSION 3.8)
project(torso)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(torso_node src/torso_node.cpp)

ament_target_dependencies(torso_node
  rclcpp
  std_msgs
)

install(TARGETS
  torso_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
