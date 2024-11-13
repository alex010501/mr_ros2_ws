#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class MyCPPTalker: public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_publisher;
    size_t m_count;

    void timer_callback()
    {
        auto lv_message = std_msgs::msg::String();
        lv_message.data = "What's up, fagot? " + std::to_string(m_count++);
        RCLCPP_INFO(this->get_logger(), "Talk: '%s'", lv_message.data.c_str());
        this->m_publisher->publish(lv_message);
    }

public:
    MyCPPTalker(): Node("MyCPPTalker"), m_count(0)
    {
        this->m_publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
        this->m_timer = this->create_wall_timer(500ms, std::bind(&MyCPPTalker::timer_callback, this));
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyCPPTalker>());
    rclcpp::shutdown();
    return 0;
}
