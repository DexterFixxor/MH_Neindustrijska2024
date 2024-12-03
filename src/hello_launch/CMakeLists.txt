cmake_minimum_required(VERSION 3.8)
project(hello_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(hello_world src/hello_world.cpp)

target_include_directories(hello_world PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(hello_world PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_world
  # --- depends ---
  rclcpp
  std_msgs
)

install(
  TARGETS 
  hello_world

  DESTINATION 
  lib/${PROJECT_NAME}
)

install(
  DIRECTORY 
  launch
  
  DESTINATION 
  share/${PROJECT_NAME})

ament_package()
