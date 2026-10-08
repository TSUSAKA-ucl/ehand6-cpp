cmake_minimum_required(VERSION 3.8)
project(ehand6_ctrl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(ehand6_msgs REQUIRED)

add_executable(ehand6_ctrl_node src/ehand6_ctrl_node.cpp)
target_include_directories(ehand6_ctrl_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(ehand6_ctrl_node rclcpp sensor_msgs std_srvs ehand6_msgs)

install(TARGETS ehand6_ctrl_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
