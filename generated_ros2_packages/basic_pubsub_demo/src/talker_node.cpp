#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

// A simple node that periodically publishes textual data on a topic.
class TalkerNode : public rclcpp::Node
{
public:
  TalkerNode()
  : Node("talker_node"), count_(0)
  {
    // Create a publisher for std_msgs/String on topic "chatter".
    publisher_ = this->create_publisher<std_msgs::msg::String>(
      "chatter", 10);

    // Create a timer that fires every 500 ms and calls timer_callback().
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TalkerNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Talker node started. Publishing on 'chatter'.");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello from Talker, message #" + std::to_string(count_++);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TalkerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
