cmake_minimum_required(VERSION 3.8)
project(imu_publisher)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(imu_publisher_node
  src/imu_publisher_node.cpp
  src/icm20948.c
)

target_include_directories(imu_publisher_node PRIVATE src)
target_link_libraries(imu_publisher_node m)

ament_target_dependencies(imu_publisher_node rclcpp sensor_msgs)

install(TARGETS
  imu_publisher_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
