cmake_minimum_required(VERSION 3.5)
project(ros2_pixi_playground)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
set(dependencies rclcpp::rclcpp ${std_msgs_TARGETS} message_filters::message_filters rosbag2_cpp::rosbag2_cpp ${sensor_msgs_TARGETS})

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${dependencies})

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${dependencies})