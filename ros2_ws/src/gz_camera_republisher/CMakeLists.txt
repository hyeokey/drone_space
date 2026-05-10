cmake_minimum_required(VERSION 3.8)
project(gz_camera_republisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GZ_TRANSPORT REQUIRED ignition-transport11)
pkg_check_modules(GZ_MSGS REQUIRED ignition-msgs8)

add_executable(gz_camera_republisher src/gz_camera_republisher.cpp)
target_include_directories(gz_camera_republisher PRIVATE
  ${GZ_TRANSPORT_INCLUDE_DIRS}
  ${GZ_MSGS_INCLUDE_DIRS}
)
target_link_directories(gz_camera_republisher PRIVATE
  ${GZ_TRANSPORT_LIBRARY_DIRS}
  ${GZ_MSGS_LIBRARY_DIRS}
)
target_link_libraries(gz_camera_republisher
  ${GZ_TRANSPORT_LIBRARIES}
  ${GZ_MSGS_LIBRARIES}
)
ament_target_dependencies(gz_camera_republisher
  rclcpp
  sensor_msgs
)

install(TARGETS gz_camera_republisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
