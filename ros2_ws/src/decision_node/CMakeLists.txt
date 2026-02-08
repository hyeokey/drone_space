cmake_minimum_required(VERSION 3.8)
project(decision_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(decision_node
  src/decision_node.cpp
)

ament_target_dependencies(decision_node
  rclcpp
  px4_msgs
)

install(TARGETS
  decision_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
