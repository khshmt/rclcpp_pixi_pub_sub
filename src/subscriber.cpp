#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("subscriber_node");
    auto sub_ = node->create_subscription<std_msgs::msg::String>("my_topic", 10, [](const std_msgs::msg::String& msg) {
        RCLCPP_INFO(rclcpp::get_logger("subscriber_node"), "Received: '%s'", msg.data.c_str());
    });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}