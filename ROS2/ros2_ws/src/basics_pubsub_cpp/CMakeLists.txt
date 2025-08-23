cmake_minimum_required(VERSION 3.8)
project(basics_pubsub_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(simple_pub_cpp src/simple_pub.cpp)
ament_target_dependencies(simple_pub_cpp rclcpp std_msgs)

add_executable(simple_sub_cpp src/simple_sub.cpp)
ament_target_dependencies(simple_sub_cpp rclcpp std_msgs)

install(TARGETS
  simple_pub_cpp
  simple_sub_cpp
  DESTINATION lib/${PROJECT_NAME})

# Install launch files so ros2 launch can find them
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()