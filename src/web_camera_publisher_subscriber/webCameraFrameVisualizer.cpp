#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rerun.hpp>

class RerunCameraNode : public rclcpp::Node {
public:
  RerunCameraNode() : Node("rerun_camera_node"), rec_("ros2_camera") {
    // Start Rerun viewer
    rec_.spawn(rerun::SpawnOptions{.memory_limit = "1GB"}).exit_on_failure();
    if(!rec_.is_enabled()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to start rerun viewer!!");
    }
    auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
    sub_ = image_transport::create_subscription(
      this, "/camera/image_raw",
      std::bind(&RerunCameraNode::imageCallback, this, std::placeholders::_1),
      "raw", qos.get_rmw_qos_profile());

    RCLCPP_INFO(this->get_logger(), "Rerun Camera Node Started.");
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    auto img = cv_bridge::toCvShare(msg, "rgb8")->image;
    rec_.log("camera/image", rerun::Image(img.data,
                                          {static_cast<uint32_t>(img.cols),
                                           static_cast<uint32_t>(img.rows)},
                                          rerun::datatypes::ColorModel::RGB));
  }

private:
  image_transport::Subscriber sub_;
  const rerun::RecordingStream rec_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RerunCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
