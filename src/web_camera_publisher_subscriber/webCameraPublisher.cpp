#include <mutex>
#include <queue>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

#include <concurrentqueue.h>

using namespace std::chrono_literals;

class FramePublisher : public rclcpp::Node {
public:
  FramePublisher() : Node("usb_camera_publisher") {
    if (!cap_.isOpened()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open camera!!");
      rclcpp::shutdown();
    }
    //Force MJPG
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    //Set resolution
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    //Set FPS
    cap_.set(cv::CAP_PROP_FPS, 30);
    //Reduce buffering
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
    imagePublisher_ = image_transport::CameraPublisher(
      this, "/camera/image_raw", qos.get_rmw_qos_profile());
    getting_images_thread_ =
      std::thread(&FramePublisher::getting_images_from_camera, this);

    timer_ = this->create_wall_timer(
      std::chrono::steady_clock::duration(20ms),
      std::bind(&FramePublisher::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "USB Camera Publisher Node Started.");
  }

  ~FramePublisher() {
    if (getting_images_thread_.joinable()) {
      stop_getting_frames_thread = true;
      getting_images_thread_.join();
    }
  }

private:
  void getting_images_from_camera() {
    while (!stop_getting_frames_thread) {
      cv::Mat frame;
      if (cap_.read(frame)) [[likely]] {
        imgs_queue_.enqueue(frame);
      } else [[unlikely]] {
        RCLCPP_INFO(this->get_logger(), "failed to open camera");
        return;
      }
    }
  }
  void timer_callback() {
    cv::Mat frame;
    auto not_empty = imgs_queue_.try_dequeue(frame);
    if (not_empty) {
      auto imageMsg_ =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame)
          .toImageMsg();

      sensor_msgs::msg::CameraInfo cameraInfoMsg_;
      cameraInfoMsg_.header = imageMsg_->header;
      cameraInfoMsg_.height = imageMsg_->height;
      cameraInfoMsg_.width = imageMsg_->width;

      imagePublisher_.publish(*imageMsg_, cameraInfoMsg_);
    }
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  image_transport::CameraPublisher imagePublisher_;
#ifdef _WIN32
  cv::VideoCapture cap_{0, cv::CAP_DSHOW};
#elif __LINUX__
  cv::VideoCapture cap_{0, cv::CAP_V4L2};
#endif
  moodycamel::ConcurrentQueue<cv::Mat> imgs_queue_;
  std::thread getting_images_thread_;
  std::atomic_bool stop_getting_frames_thread{false};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FramePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}