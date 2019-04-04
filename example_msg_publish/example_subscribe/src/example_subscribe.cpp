#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "example_msg/msg/contact.hpp"

class ContactSubscriber : public rclcpp::Node
{
public:
    ContactSubscriber()
        : Node("address_book_subscribe")
    {
        contact_subscriber_ = this->create_subscription<example_msg::msg::Contact>(
                    "example_msg_publish_test",
                    [this](example_msg::msg::Contact::UniquePtr msg) {
                std::cout << "receive msg: " << std::endl;
                std::cout << msg->first_name << "." << msg->last_name << std::endl;
                std::cout << msg->address << std::endl;
                std::cout << msg->age << std::endl;
                std::cout << msg->gender << std::endl;
    });
    }

private:
    rclcpp::Subscription<example_msg::msg::Contact>::SharedPtr contact_subscriber_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto publisher_node = std::make_shared<ContactSubscriber>();

    rclcpp::spin(publisher_node);

    return 0;
}
