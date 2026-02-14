#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/storage_options.hpp"
#include "rosbag2_cpp/writer.hpp"

class SimpleSubscriberRecorder : public rclcpp::Node {
public:
  SimpleSubscriberRecorder() : Node("simple_subscriber_recorder") {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = "my_bag";
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions converter_options{
      rmw_get_serialization_format(), rmw_get_serialization_format()};

    writer_->open(storage_options, converter_options);

    writer_->create_topic({topic_data_.topic_name_, topic_data_.type_name_,
                           rmw_get_serialization_format(), ""});

    subscription_ = this->create_generic_subscription(
      topic_data_.topic_name_, topic_data_.type_name_, rclcpp::QoS(10),
      [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {
        writer_->write(msg, topic_data_.topic_name_, topic_data_.type_name_,
                       this->now());
      });
  }

private:
  struct topicData {
    std::string topic_name_{"my_topic"};
    std::string type_name_{"std_msgs/msg/String"};
  } topic_data_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  rclcpp::GenericSubscription::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriberRecorder>());
  rclcpp::shutdown();
  return 0;
}
