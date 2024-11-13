#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

class MyCPPListener: public rclcpp::Node
{
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr m_subscription;

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    

public:
    MyCPPListener(): Node("MyCPPListener")
    {
        this->m_subscription = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&MyCPPListener::topic_callback, this, _1));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyCPPListener>());
    rclcpp::shutdown();
    return 0;
}