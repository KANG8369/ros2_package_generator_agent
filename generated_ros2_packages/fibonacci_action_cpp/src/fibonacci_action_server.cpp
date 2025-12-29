#include <memory>
#include <thread>
#include <vector>
#include <chrono>
#include <functional>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "fibonacci_action_cpp/action/fibonacci.hpp"

using namespace std::chrono_literals;

class FibonacciActionServer : public rclcpp::Node
{
public:
  using Fibonacci = fibonacci_action_cpp::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  explicit FibonacciActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("fibonacci_action_server", options)
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<Fibonacci>(
      this,
      "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this, _1, _2),
      std::bind(&FibonacciActionServer::handle_cancel, this, _1),
      std::bind(&FibonacciActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Fibonacci action server started.");
  }

private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Fibonacci::Goal> goal)
  {
    (void)uuid;
    if (goal->order <= 0) {
      RCLCPP_WARN(this->get_logger(), "Rejected goal with non-positive order: %d", goal->order);
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Accepted goal request with order %d", goal->order);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel Fibonacci goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    std::thread{std::bind(&FibonacciActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing Fibonacci goal");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->partial_sequence;
    auto & progress = feedback->progress;
    auto & status_message = feedback->status_message;

    auto result = std::make_shared<Fibonacci::Result>();

    sequence.clear();
    sequence.push_back(0);
    sequence.push_back(1);

    rclcpp::Rate rate(1.0);  // 1 Hz

    int32_t order = goal->order;
    if (order < 2) {
      sequence.resize(static_cast<size_t>(order));
    }

    for (int32_t i = 2; i < order && rclcpp::ok(); ++i) {
      // Check if goal is canceled
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled. Returning partial sequence.");
        return;
      }

      sequence.push_back(sequence[static_cast<size_t>(i - 1)] + sequence[static_cast<size_t>(i - 2)]);

      progress = static_cast<float>(i + 1) / static_cast<float>(order);
      status_message = "Computing Fibonacci sequence...";

      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Feedback: step %d/%d, last value %d", i + 1, order, sequence.back());

      rate.sleep();
    }

    if (!rclcpp::ok()) {
      RCLCPP_WARN(this->get_logger(), "Node is shutting down, aborting goal");
      return;
    }

    result->sequence = sequence;
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "Goal succeeded. Final sequence of length %zu computed.", result->sequence.size());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FibonacciActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
