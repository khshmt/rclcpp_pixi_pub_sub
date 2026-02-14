#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/writer.hpp"

class FrameSubscriberRecorder : public rclcpp::Node {
public:
  FrameSubscriberRecorder() : Node("frame_subscriber_recorder") {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = "my_bag";
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options{
      rmw_get_serialization_format(), rmw_get_serialization_format()};

    writer_->open(storage_options, converter_options);

    writer_->create_topic({topic_data_.topic_name_, topic_data_.type_name_,
                           rmw_get_serialization_format(), ""});
    auto qos = rclcpp::SensorDataQoS().keep_last(1).best_effort();
    subscription_ = this->create_generic_subscription(
      topic_data_.topic_name_, topic_data_.type_name_, qos,
      [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        writer_->write(msg, topic_data_.topic_name_, topic_data_.type_name_,
                       this->now());
      });
  }

private:
  struct topicData {
    std::string topic_name_{"/camera/image_raw"};
    std::string type_name_{"sensor_msgs/msg/Image"};
  } topic_data_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::GenericSubscription::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrameSubscriberRecorder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
