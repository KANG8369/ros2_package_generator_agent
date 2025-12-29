#include <chrono>
#include <cstdlib>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

// A simple client node that sends a request to add two integers.
class AdderClient : public rclcpp::Node
{
public:
  AdderClient(int64_t a, int64_t b)
  : Node("adder_client"), a_(a), b_(b)
  {
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");

    // Wait for the service to be available.
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a_;
    request->b = b_;

    RCLCPP_INFO(this->get_logger(), "Sending request: a=%ld, b=%ld",
                static_cast<long>(a_), static_cast<long>(b_));

    auto future = client_->async_send_request(request);

    // Wait for the response synchronously for simplicity.
    if (rclcpp::spin_until_future_complete(shared_from_this(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
      auto result = future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %ld + %ld = %ld",
                  static_cast<long>(a_),
                  static_cast<long>(b_),
                  static_cast<long>(result->sum));
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to call service add_two_ints");
    }
  }

private:
  int64_t a_;
  int64_t b_;
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
    RCLCPP_ERROR(rclcpp::get_logger("adder_client"),
                 "Usage: adder_client <integer a> <integer b>");
    rclcpp::shutdown();
    return 1;
  }

  int64_t a = std::strtoll(argv[1], nullptr, 10);
  int64_t b = std::strtoll(argv[2], nullptr, 10);

  auto node = std::make_shared<AdderClient>(a, b);
  // The constructor already performs the call and logging; spinning here
  // keeps the node alive only while the call is processed.
  rclcpp::spin_some(node);

  rclcpp::shutdown();
  return 0;
}
