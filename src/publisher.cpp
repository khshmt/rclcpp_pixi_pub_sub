#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

class SimplePublisher : public rclcpp::Node {
public:
  SimplePublisher() : Node("simple_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SimplePublisher::timer_callback, this));
  }

private:
  void timer_callback() {
    std_msgs::msg::String message;
    message.set__data("Hello, ROS 2 with Pixi!");
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}