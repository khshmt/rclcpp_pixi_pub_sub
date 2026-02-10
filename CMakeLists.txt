cmake_minimum_required(VERSION 3.5)
project(pub_sub_example)


find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
set(dependencies rclcpp::rclcpp ${std_msgs_TARGETS})

add_executable(publisher src/publisher.cpp)
target_link_libraries(publisher ${dependencies})

add_executable(subscriber src/subscriber.cpp)
target_link_libraries(subscriber ${dependencies})