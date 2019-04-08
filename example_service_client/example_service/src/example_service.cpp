#include <memory>
#include "example_msg/srv/add_two_floats.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoFloats = example_msg::srv::AddTwoFloats;
extern rclcpp::Node::SharedPtr g_node;
rclcpp::Node::SharedPtr g_node = nullptr;

void handle_service(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<AddTwoFloats::Request> request,
        const std::shared_ptr<AddTwoFloats::Response> response)
{
    (void)request_header;
    RCLCPP_INFO(
                g_node->get_logger(),
                "request: %lf + %lf", request->a, request->b);
    response->sum = request->a + request->b;
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    g_node = rclcpp::Node::make_shared("minimal_service");
    auto server = g_node->create_service<AddTwoFloats>("add_two_floats", handle_service);
    rclcpp::spin(g_node);
    rclcpp::shutdown();
    g_node = nullptr;
    return 0;
}
