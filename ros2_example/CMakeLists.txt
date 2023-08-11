cmake_minimum_required(VERSION 3.14)
project(adv_to_server)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

string(REGEX REPLACE "(.*)/(.*)/(.*)" "\\1" PROJECT_INIT_PATH  ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND CMAKE_MODULE_PATH ${PROJECT_INIT_PATH}/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Mosquitto REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)
include_directories(include ${MOSQUITTO_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})

if (NOT MOSQUITTO_FOUND)
  message(FATAL_ERROR "libmosquitto-dev not found. Run\n  sudo apt-get install libmosquitto-dev")
endif()

file(GLOB_RECURSE SOURCES_SRCS *.cpp)
add_executable( adv_to_server src/main.cpp  ${SOURCES_SRCS})
target_link_libraries(adv_to_server ${MOSQUITTO_LIBRARY} ${Boost_LIBRARIES})

ament_target_dependencies(adv_to_server
  ament_index_cpp
  rclcpp
  rclpy
  std_msgs
  msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_include_directories(adv_to_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS adv_to_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/TLS/
  DESTINATION share/${PROJECT_NAME}/TLS/
)

ament_package()
