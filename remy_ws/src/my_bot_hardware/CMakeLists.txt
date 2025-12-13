cmake_minimum_required(VERSION 3.8)
project(my_bot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

add_library(my_bot_hardware SHARED
  src/bot_system.cpp
)

target_include_directories(my_bot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(my_bot_hardware
  rclcpp
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(
  hardware_interface
  bot_system_plugin.xml
)

install(
  TARGETS my_bot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES bot_system_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
