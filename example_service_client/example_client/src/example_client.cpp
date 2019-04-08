#include <chrono>
#include <memory>
#include "example_msg/srv/add_two_floats.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoFloats = example_msg::srv::AddTwoFloats;

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("minimal_client");
    auto client = node->create_client<AddTwoFloats>("add_two_floats");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "client interrupted while waiting for service to appear.");
            return 1;
        }
        RCLCPP_INFO(node->get_logger(), "waiting for service to appear...");
    }
    auto request = std::make_shared<AddTwoFloats::Request>();
    request->a = 41.134;
    request->b = 1.259;
    auto result_future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node->get_logger(), "service call failed :(");
        return 1;
    }
    auto result = result_future.get();
    RCLCPP_INFO(node->get_logger(), "result of %lf + %lf = %lf",
                request->a, request->b, result->sum);
    rclcpp::shutdown();
    return 0;
}
