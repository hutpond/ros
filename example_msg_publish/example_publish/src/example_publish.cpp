#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "example_msg/msg/contact.hpp"

class ContactPublisher : public rclcpp::Node
{
public:
    ContactPublisher()
        : Node("address_book_publish")
    {
        contact_publisher_ = this->create_publisher<example_msg::msg::Contact>("example_msg_publish_test");

        auto publish_msg = [this]() -> void {
                auto msg = std::make_shared<example_msg::msg::Contact>();

                msg->first_name = "John";
                msg->last_name = "Doe";
                msg->age = 30;
                msg->gender = msg->MALE;
                msg->address = "unknown";

                std::cout << "Publishing Contact\nFirst:" << msg->first_name <<
                             "  Last:" << msg->last_name << std::endl;

                contact_publisher_->publish(msg);
        };
        timer_ = this->create_wall_timer(std::chrono::seconds(1), publish_msg);
    }

    virtual ~ContactPublisher() {}

private:
    rclcpp::Publisher<example_msg::msg::Contact>::SharedPtr contact_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto publisher_node = std::make_shared<ContactPublisher>();

    rclcpp::spin(publisher_node);

    return 0;
}
