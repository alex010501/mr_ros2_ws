#include <rclcpp/rclcpp.hpp>
#include <simple_controller/controller.h>

int main(int argc, char *argv[])
{
    // Initialize the ROS2 node
    rclcpp::init(argc, argv);

    // Start the ControllerNode
    rclcpp::spin(std::make_shared<simple_controller::Controller>());

    rclcpp::shutdown();
    return 0;
}
