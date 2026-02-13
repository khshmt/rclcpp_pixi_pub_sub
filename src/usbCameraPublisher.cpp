#include <queue>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

using namespace std::chrono_literals;

class UsbCameraImagesPublisher : public rclcpp::Node {
public:
  UsbCameraImagesPublisher()
      : Node("opencv_image_publisher") {
    imagePublisher_ =
      this->create_publisher<sensor_msgs::msg::Image>("/image", 10);
    cameraInfoPublisher_ =
      this->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info", 10);
    getting_images_thread_ =
      std::thread(&UsbCameraImagesPublisher::getting_images_from_camera, this);

    timer_ = this->create_wall_timer(
      10ms, std::bind(&UsbCameraImagesPublisher::timer_callback, this));
  }
  ~UsbCameraImagesPublisher() {
    if (getting_images_thread_.joinable())
      getting_images_thread_.detach();
  }

private:
  void getting_images_from_camera() {
    while (true) {
      cv::Mat frame;
      if (cap_.isOpened()) {
        auto frame_arrived = cap_.read(frame);
        if (frame_arrived && imgs_queue_.size() < 20) {
          std::lock_guard<std::mutex> lock_(m_);
          imgs_queue_.emplace(frame);
        }
        count_++;
      } else {
        RCLCPP_INFO(this->get_logger(), "failed to open camera");
        return;
      }
    }
  }
  void timer_callback() {
    cv::Mat frame;
    if (!imgs_queue_.empty()) {
      imageMsg_ =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imgs_queue_.front())
          .toImageMsg();

      cameraInfoMsg_.header = imageMsg_->header;
      cameraInfoMsg_.height = imageMsg_->height;
      cameraInfoMsg_.width = imageMsg_->width;

      // Publish the image to the topic defined in the publisher
      imagePublisher_->publish(*imageMsg_.get());
      cameraInfoPublisher_->publish(cameraInfoMsg_);
      RCLCPP_INFO(this->get_logger(), "Image %ld published", count_);
      std::lock_guard<std::mutex> lock_(m_);
      imgs_queue_.pop();
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::Image::SharedPtr imageMsg_;
  sensor_msgs::msg::CameraInfo cameraInfoMsg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr imagePublisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
    cameraInfoPublisher_;
  size_t count_{0};
#ifdef _WIN32
  cv::VideoCapture cap_{1};
#elif __LINUX__
  cv::VideoCapture cap_{0};
#endif
  std::queue<cv::Mat> imgs_queue_;
  std::thread getting_images_thread_;
  std::mutex m_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UsbCameraImagesPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}