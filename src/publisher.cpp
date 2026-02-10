#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("publisher_node");
    auto pub_ = node->create_publisher<std_msgs::msg::String>("my_topic", 10);
    rclcpp::Rate loop_rate(10); // 1 Hz
    while (rclcpp::ok())
    {
        std_msgs::msg::String msg;
        msg.data = "Hello, ROS 2 with Pixi!";
        pub_->publish(msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}