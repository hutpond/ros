#include <inttypes.h>
#include <memory>
#include "example_msg/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = example_msg::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

rclcpp_action::GoalResponse handle_goal(
        const std::array<uint8_t, 16> & uuid, std::shared_ptr<const Fibonacci::Goal> goal)
{
    RCLCPP_INFO(rclcpp::get_logger("server"), "Got goal request with order %d", goal->order);
    (void)uuid;
    // Let's reject sequences that are over 9000
    if (goal->order > 9000)
    {
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    RCLCPP_INFO(rclcpp::get_logger("server"), "Got request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void execute(
        const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    RCLCPP_INFO(rclcpp::get_logger("server"), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto& sequence = feedback->sequence;
    sequence.push_back(0);
    sequence.push_back(1);
    auto result_response = std::make_shared<Fibonacci::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
    {
        // Check if there is a cancel request
        if (goal_handle->is_canceling())
        {
            result_response->sequence = sequence;
            goal_handle->set_canceled(result_response);
            RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Canceled");
            return;
        }
        // Update sequence
        sequence.push_back(sequence[i] + sequence[i - 1]);
        // Publish feedback
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(rclcpp::get_logger("server"), "Publish Feedback");

        loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
        result_response->sequence = sequence;
        goal_handle->set_succeeded(result_response);
        RCLCPP_INFO(rclcpp::get_logger("server"), "Goal Suceeded");
    }
}

void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle)
{
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{execute, goal_handle}.detach();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("example_action_server");

    // Create an action server with three callbacks
    //   'handle_goal' and 'handle_cancel' are called by the Executor (rclcpp::spin)
    //   'execute' is called whenever 'handle_goal' returns by accepting a goal
    //    Calls to 'execute' are made in an available thread from a pool of four.
    auto action_server = rclcpp_action::create_server<Fibonacci>(
                node,
                "fibonacci",
                handle_goal,
                handle_cancel,
                handle_accepted);

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
