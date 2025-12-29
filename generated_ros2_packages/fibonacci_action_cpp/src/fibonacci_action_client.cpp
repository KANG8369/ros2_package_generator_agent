#include <chrono>
#include <cinttypes>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "fibonacci_action_cpp/action/fibonacci.hpp"

using namespace std::chrono_literals;

class FibonacciActionClient : public rclcpp::Node
{
public:
  using Fibonacci = fibonacci_action_cpp::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

  explicit FibonacciActionClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");

    this->declare_parameter<int>("order", 10);

    timer_ = this->create_wall_timer(
      1s, std::bind(&FibonacciActionClient::send_goal, this));
  }

  void send_goal()
  {
    timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = this->get_parameter("order").as_int();

    RCLCPP_INFO(this->get_logger(), "Sending Fibonacci goal with order %d", goal_msg.order);

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&FibonacciActionClient::result_callback, this, std::placeholders::_1);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleFibonacci::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleFibonacci::SharedPtr,
    const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    float progress = feedback->progress * 100.0f;
    int last_value = 0;
    if (!feedback->partial_sequence.empty()) {
      last_value = feedback->partial_sequence.back();
    }
    RCLCPP_INFO(this->get_logger(), "Feedback: progress=%.1f%%, last_value=%d, status='%s'",
      progress, last_value, feedback->status_message.c_str());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Result received:");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    std::string seq_str;
    seq_str.reserve(result.result->sequence.size() * 5);
    for (size_t i = 0; i < result.result->sequence.size(); ++i) {
      seq_str += std::to_string(result.result->sequence[i]);
      if (i + 1 < result.result->sequence.size()) {
        seq_str += ", ";
      }
    }
    RCLCPP_INFO(this->get_logger(), "Final Fibonacci sequence: [%s]", seq_str.c_str());

    rclcpp::shutdown();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FibonacciActionClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
