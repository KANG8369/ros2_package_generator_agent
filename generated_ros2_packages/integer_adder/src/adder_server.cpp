#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

// A simple service node that adds two integers.
class AdderServer : public rclcpp::Node
{
public:
  AdderServer()
  : Node("adder_server")
  {
    using ServiceT = example_interfaces::srv::AddTwoInts;

    service_ = this->create_service<ServiceT>(
      "add_two_ints",
      std::bind(&AdderServer::handle_service, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Adder service ready: 'add_two_ints'");
  }

private:
  void handle_service(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    // Perform the integer addition.
    response->sum = request->a + request->b;

    RCLCPP_INFO(
      this->get_logger(),
      "Incoming request: a=%ld, b=%ld -> sum=%ld",
      static_cast<long>(request->a),
      static_cast<long>(request->b),
      static_cast<long>(response->sum));
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdderServer>());
  rclcpp::shutdown();
  return 0;
}
