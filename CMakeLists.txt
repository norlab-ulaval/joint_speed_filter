cmake_minimum_required(VERSION 3.8)
project(joint_speed_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joint_speed_filter_node src/joint_speed_filter_node.cpp)
ament_target_dependencies(joint_speed_filter_node
  rclcpp
  std_msgs
  sensor_msgs
)

install(TARGETS joint_speed_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
