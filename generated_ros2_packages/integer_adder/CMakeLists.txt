cmake_minimum_required(VERSION 3.8)
project(integer_adder)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

# Server node
add_executable(adder_server src/adder_server.cpp)
ament_target_dependencies(adder_server rclcpp example_interfaces)

# Client node
add_executable(adder_client src/adder_client.cpp)
ament_target_dependencies(adder_client rclcpp example_interfaces)

install(TARGETS
  adder_server
  adder_client
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
