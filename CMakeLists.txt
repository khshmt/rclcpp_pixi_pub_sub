cmake_minimum_required(VERSION 3.8)
project(ros2_pixi_playground LANGUAGES CXX C)

# ------------------------------------------------------------------------------
# C++ Standard
# ------------------------------------------------------------------------------
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rerun_sdk REQUIRED)

include(FetchContent)
FetchContent_Declare(
  concurrentqueue
  GIT_REPOSITORY https://github.com/cameron314/concurrentqueue
  GIT_TAG        v1.0.4
)
FetchContent_MakeAvailable(concurrentqueue)

set(ROS2_DEPENDENCIES
    rclcpp::rclcpp
    ${std_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    rosbag2_cpp::rosbag2_cpp
    message_filters::message_filters
    ${OpenCV_LIBS}
    cv_bridge::cv_bridge
    image_transport::image_transport
    rerun_sdk
    concurrentqueue
)

# ------------------------------------------------------------------------------
# Publisher Executable
# ------------------------------------------------------------------------------
add_executable(publisher
    src/simple_publisher_subscriber/publisher.cpp
)

target_link_libraries(publisher
    ${ROS2_DEPENDENCIES}
)

# ------------------------------------------------------------------------------
# Subscriber Executable
# ------------------------------------------------------------------------------
add_executable(subscriber
    src/simple_publisher_subscriber/subscriber.cpp
)

target_link_libraries(subscriber
    ${ROS2_DEPENDENCIES}
)

# ------------------------------------------------------------------------------
# usbCameraPublisher Executable
# ------------------------------------------------------------------------------
add_executable(webCameraPublisher
    src/web_camera_publisher_subscriber/webCameraPublisher.cpp
)

target_link_libraries(webCameraPublisher
    ${ROS2_DEPENDENCIES}
)

# ------------------------------------------------------------------------------
# usbCameraSubscriber Executable
# ------------------------------------------------------------------------------
add_executable(webCameraSubscriber
    src/web_camera_publisher_subscriber/webCameraSubscriber.cpp
)

target_link_libraries(webCameraSubscriber
    ${ROS2_DEPENDENCIES}
)

# ------------------------------------------------------------------------------
# usbCameraFrameVisualizer Executable
# ------------------------------------------------------------------------------
add_executable(webCameraFrameVisualizer
    src/web_camera_publisher_subscriber/webCameraFrameVisualizer.cpp
)

target_link_libraries(webCameraFrameVisualizer
    ${ROS2_DEPENDENCIES}
)
