cmake_minimum_required(VERSION 3.5)
project(rm_serial_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(rm_msgs REQUIRED)

# 构建共享库作为组件节点
add_library(rm_serial_driver SHARED
  src/rm_serial_driver.cpp
  src/crc.cpp
)

target_include_directories(rm_serial_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(rm_serial_driver
  rclcpp
  rclcpp_components
  std_msgs
  serial
  rm_msgs
)

# 注册节点
rclcpp_components_register_nodes(rm_serial_driver "rm_serial_driver::RMSerialDriver")

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS rm_serial_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()