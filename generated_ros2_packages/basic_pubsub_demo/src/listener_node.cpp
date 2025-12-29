#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// A simple node that subscribes to textual data and prints it.
class ListenerNode : public rclcpp::Node
{
public:
  ListenerNode()
  : Node("listener_node")
  {
    // Subscribe to the same topic used by the talker: "chatter".
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      std::bind(&ListenerNode::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Listener node started. Subscribed to 'chatter'.");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // Print the received text to confirm successful message exchange.
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ListenerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
